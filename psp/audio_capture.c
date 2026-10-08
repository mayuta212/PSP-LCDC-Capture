/* LCDC60 A02 UNIVERSAL AUDIO
 *
 * Capture-only hooks for the public PSP PCM output paths.  Hooks never wait
 * for USB and never modify the caller's PCM.  Audio may begin arbitrarily late
 * (GT PSP is a useful example); there is deliberately no "no audio" timeout.
 *
 * Covered GAME paths:
 *   - normal 0..7 channels: Output/OutputBlocking/Panned/PannedBlocking
 *   - Output2
 *   - SRC
 *   - Vaudio
 *
 * Unsupported/unknown state is always transparent: the original call runs.
 */
#include <pspkernel.h>
#include <pspsdk.h>
#include <pspaudio.h>
#include <systemctrl.h>
#include <stdint.h>
#include <string.h>
#include "audio_capture.h"

#define AUDIO_STATE_FREE 0
#define AUDIO_STATE_WRITING 1
#define AUDIO_STATE_READY 2
#define AUDIO_STATE_IN_USB 3

/* sceAudio public NIDs (1.50 names, resolved by SystemControl on CFW). */
#define NID_CH_RESERVE              0x5EC81C55u
#define NID_CH_RELEASE              0x6FC46853u
#define NID_OUTPUT                  0x8C1009B2u
#define NID_OUTPUT_BLOCKING         0x136CAF51u
#define NID_PANNED                  0xE2D56B2Du
#define NID_PANNED_BLOCKING         0x13F592BCu
#define NID_SET_DATA_LEN            0xCB2E439Eu
#define NID_CHANGE_CONFIG           0x95FD0C2Du

#define NID_OUTPUT2_RESERVE         0x01562BA3u
#define NID_OUTPUT2_RELEASE         0x43196845u
#define NID_OUTPUT2_BLOCKING        0x2D53F36Eu
#define NID_OUTPUT2_CHANGE_LENGTH   0x63F2889Cu

#define NID_SRC_RESERVE             0x38553111u
#define NID_SRC_RELEASE             0x5C37C0AEu
#define NID_SRC_BLOCKING            0xE0727056u

#define NID_VAUDIO_RESERVE          0x03B6807Du
#define NID_VAUDIO_RELEASE          0x67585DFDu
#define NID_VAUDIO_BLOCKING         0x8986295Eu
#define VAUDIO_FORMAT_STEREO        2

/* Hook-mask bits. */
#define H_CH_RESERVE        (1u<<0)
#define H_CH_RELEASE        (1u<<1)
#define H_CH_LEN            (1u<<2)
#define H_CH_CONFIG         (1u<<3)
#define H_OUT               (1u<<4)
#define H_OUT_BLOCK         (1u<<5)
#define H_PAN               (1u<<6)
#define H_PAN_BLOCK         (1u<<7)
#define H_O2_RESERVE        (1u<<8)
#define H_O2_RELEASE        (1u<<9)
#define H_O2_BLOCK          (1u<<10)
#define H_O2_LEN            (1u<<11)
#define H_SRC_RESERVE       (1u<<12)
#define H_SRC_RELEASE       (1u<<13)
#define H_SRC_BLOCK         (1u<<14)
#define H_VA_RESERVE        (1u<<15)
#define H_VA_RELEASE        (1u<<16)
#define H_VA_BLOCK          (1u<<17)


struct AudioSlot {
    struct AudioWireHeader h;
    unsigned char pcm[AUDIO_FRAMES_PER_PACKET * 4u];
} __attribute__((aligned(64)));

struct SpecialState {
    int reserved;
    int frames;
    int rate;
    int channels;
};

struct InstalledPatch {
    void *addr;
    void *hook;
};

static struct AudioSlot g_audio_pkt[AUDIO_QUEUE_SLOTS] __attribute__((aligned(64)));
static struct UsbdDeviceReq g_audio_req[AUDIO_QUEUE_SLOTS] __attribute__((aligned(64)));
static volatile int g_audio_state[AUDIO_QUEUE_SLOTS];
static volatile u32 g_audio_write, g_audio_read, g_audio_inflight, g_audio_seq;
static volatile u32 g_hook_mask;
static volatile int g_audio_on;
static volatile int g_audio_stop;

static int g_frames[8], g_formats[8];
static struct SpecialState g_output2, g_src, g_vaudio;
static struct InstalledPatch g_patches[24];
static unsigned g_patch_count;

/* Normal channel function pointers. */
static int (*real_ch_reserve)(int,int,int);
static int (*real_ch_release)(int);
static int (*real_set_len)(int,int);
static int (*real_config)(int,int);
static int (*real_output)(int,int,void*);
static int (*real_output_block)(int,int,void*);
static int (*real_panned)(int,int,int,void*);
static int (*real_panned_block)(int,int,int,void*);

/* Single-stream function pointers. */
static int (*real_o2_reserve)(int);
static int (*real_o2_release)(void);
static int (*real_o2_block)(int,void*);
static int (*real_o2_len)(int);
static int (*real_src_reserve)(int,int,int);
static int (*real_src_release)(void);
static int (*real_src_block)(int,void*);
static int (*real_va_reserve)(int,int,int);
static int (*real_va_release)(void);
static int (*real_va_block)(int,void*);

static unsigned clamp_vol(int v)
{
    return v < 0 ? 0u : (v > 0x8000 ? 0x8000u : (unsigned)v);
}

static void *find_audio(u32 nid)
{
    void *p=(void*)(uintptr_t)sctrlHENFindFunction("sceAudio_Driver","sceAudio",nid);
    if(!p) p=(void*)(uintptr_t)sctrlHENFindFunction("sceAudio","sceAudio",nid);
    return p;
}

static void *find_vaudio(u32 nid)
{
    void *p=(void*)(uintptr_t)sctrlHENFindFunction("sceVaudio_Driver","sceVaudio",nid);
    if(!p) p=(void*)(uintptr_t)sctrlHENFindFunction("sceVaudio","sceVaudio",nid);
    return p;
}

/* Patch each concrete syscall target at most once.  Some firmware exports can
 * alias multiple NIDs to one implementation.  If two different wrappers want
 * the same address we leave the later path untouched instead of risking a
 * signature/state mismatch. */
static int install_unique(void *addr, void *hook, u32 bit)
{
    unsigned i;
    if(!addr || (g_hook_mask & bit)) return 0;
    for(i=0;i<g_patch_count;i++) {
        if(g_patches[i].addr==addr) {
            if(g_patches[i].hook==hook) {
                g_hook_mask |= bit;
                return 1;
            }
            return 0;
        }
    }
    if(g_patch_count >= sizeof(g_patches)/sizeof(g_patches[0])) return 0;
    sctrlHENPatchSyscall(addr,hook);
    g_patches[g_patch_count].addr=addr;
    g_patches[g_patch_count].hook=hook;
    g_patch_count++;
    g_hook_mask |= bit;
    return 1;
}

static int audio_done(struct UsbdDeviceReq *r, int a, int b)
{
    unsigned i;
    (void)a;(void)b;
    for(i=0;i<AUDIO_QUEUE_SLOTS;i++) if(r==&g_audio_req[i]) {
        int irq=pspSdkDisableInterrupts();
        if(g_audio_state[i]==AUDIO_STATE_IN_USB) {
            g_audio_state[i]=AUDIO_STATE_FREE;
            if(g_audio_inflight) g_audio_inflight--;
        }
        pspSdkEnableInterrupts(irq);
        return 0;
    }
    return 0;
}

void audio_init(void)
{
    unsigned i;
    g_audio_on=0;g_audio_stop=0;
    g_audio_write=g_audio_read=g_audio_inflight=g_audio_seq=0;
    g_hook_mask=0;
    g_patch_count=0;
    memset(g_patches,0,sizeof(g_patches));
    memset(&g_output2,0,sizeof(g_output2));
    memset(&g_src,0,sizeof(g_src));
    memset(&g_vaudio,0,sizeof(g_vaudio));
    for(i=0;i<AUDIO_QUEUE_SLOTS;i++)g_audio_state[i]=AUDIO_STATE_FREE;
    for(i=0;i<8;i++){g_frames[i]=0;g_formats[i]=0;}
}

void audio_enable(int enabled) { g_audio_on=enabled && !g_audio_stop; }

void audio_push_pcm(unsigned source, unsigned channel, const void *data,
                    unsigned frames, unsigned mono, unsigned volume_l,
                    unsigned volume_r, u32 timestamp_us)
{
    const unsigned char *p=(const unsigned char*)data;
    unsigned sample_bytes=mono?2u:4u;
    if(!p || !g_audio_on || g_audio_stop)return;
    while(frames) {
        unsigned n=frames>AUDIO_FRAMES_PER_PACKET?AUDIO_FRAMES_PER_PACKET:frames;
        unsigned index;u32 seq;int irq;struct AudioSlot *slot;
        if(((sizeof(struct AudioWireHeader)+n*sample_bytes)&511u)==0u && n>1u) n--;
        irq=pspSdkDisableInterrupts();
        index=g_audio_write % AUDIO_QUEUE_SLOTS;
        if(g_audio_state[index] != AUDIO_STATE_FREE) {
            pspSdkEnableInterrupts(irq);return;
        }
        g_audio_state[index]=AUDIO_STATE_WRITING;
        g_audio_write++;
        seq=g_audio_seq++;
        pspSdkEnableInterrupts(irq);
        slot=&g_audio_pkt[index];
        slot->h.magic=AUDIO_MAGIC;
        slot->h.sequence=seq;
        slot->h.timestamp_us=timestamp_us;
        slot->h.frames=(u16)n;
        slot->h.source=(u8)source;
        slot->h.channel=(u8)channel;
        slot->h.volume_l=(u16)volume_l;
        slot->h.volume_r=(u16)volume_r;
        slot->h.flags=mono?1u:0u;
        slot->h.version=AUDIO_VERSION;
        memcpy(slot->pcm,p,n*sample_bytes);
        sceKernelDcacheWritebackRange(slot,sizeof(slot->h)+n*sample_bytes);
        irq=pspSdkDisableInterrupts();
        g_audio_state[index]=AUDIO_STATE_READY;
        pspSdkEnableInterrupts(irq);
        p+=n*sample_bytes;frames-=n;
        timestamp_us += (u32)((n * 1000000u)/44100u);
    }
}

void audio_send_available(struct UsbEndpoint *ep,int max_packets)
{
    int tries;
    if(!g_audio_on||g_audio_stop)return;
    for(tries=0;tries<max_packets;tries++) {
        u32 index;int irq,ret;struct AudioSlot *slot;struct UsbdDeviceReq *req;
        irq=pspSdkDisableInterrupts();
        if(g_audio_inflight>=4u || g_audio_read==g_audio_write) {
            pspSdkEnableInterrupts(irq);return;
        }
        index=g_audio_read%AUDIO_QUEUE_SLOTS;
        if(g_audio_state[index]!=AUDIO_STATE_READY) {
            pspSdkEnableInterrupts(irq);return;
        }
        g_audio_state[index]=AUDIO_STATE_IN_USB;
        g_audio_inflight++;
        g_audio_read++;
        pspSdkEnableInterrupts(irq);
        slot=&g_audio_pkt[index];req=&g_audio_req[index];
        memset(req,0,sizeof(*req));
        req->endp=ep;
        req->data=slot;
        req->size=(int)sizeof(slot->h)+(int)slot->h.frames*((slot->h.flags&1u)?2:4);
        req->func=(void*)audio_done;
        ret=sceUsbbdReqSend(req);
        if(ret<0){
            irq=pspSdkDisableInterrupts();
            if(g_audio_state[index]==AUDIO_STATE_IN_USB){
                g_audio_state[index]=AUDIO_STATE_FREE;
                if(g_audio_inflight)g_audio_inflight--;
            }
            pspSdkEnableInterrupts(irq);
        }
    }
}

void audio_shutdown(struct UsbEndpoint *ep)
{
    g_audio_on=0;
    sceUsbbdReqCancelAll(ep);
}

/* Packets captured by hooks but not yet submitted to EP82.  This is an
 * occupancy signal, not a timer/load heuristic; GAME video yields to it. */
u32 audio_waiting_packets(void)
{
    u32 waiting;
    int irq=pspSdkDisableInterrupts();
    waiting=g_audio_write-g_audio_read;
    pspSdkEnableInterrupts(irq);
    return waiting;
}

/* ---- Normal 0..7 channel path ---- */
static int hook_ch_reserve(int channel,int frames,int fmt)
{
    int r=real_ch_reserve(channel,frames,fmt);
    if(r>=0&&r<8){g_frames[r]=frames;g_formats[r]=fmt;}
    return r;
}
static int hook_ch_release(int channel)
{
    int r=real_ch_release(channel);
    if(r>=0&&channel>=0&&channel<8){g_frames[channel]=0;g_formats[channel]=0;}
    return r;
}
static int hook_ch_len(int channel,int frames)
{
    int r=real_set_len(channel,frames);
    if(r>=0&&channel>=0&&channel<8)g_frames[channel]=frames;
    return r;
}
static int hook_ch_config(int channel,int fmt)
{
    int r=real_config(channel,fmt);
    if(r>=0&&channel>=0&&channel<8)g_formats[channel]=fmt;
    return r;
}

extern int audio_pops_ready(void);
static void tap_normal(int channel,int lv,int rv,void*data)
{
    int count,format;
    if(!g_audio_on||audio_pops_ready()||!data||channel<0||channel>=8)return;
    count=g_frames[channel];format=g_formats[channel];
    if(count<=0||count>4096)return;
    if(format!=PSP_AUDIO_FORMAT_STEREO && format!=PSP_AUDIO_FORMAT_MONO)return;
    audio_push_pcm(AUDIO_SOURCE_PSP,(unsigned)channel,data,(unsigned)count,
                   format==PSP_AUDIO_FORMAT_MONO,clamp_vol(lv),clamp_vol(rv),
                   sceKernelGetSystemTimeLow());
}
static int hook_output(int ch,int vol,void*buf){tap_normal(ch,vol,vol,buf);return real_output(ch,vol,buf);}
static int hook_output_block(int ch,int vol,void*buf){tap_normal(ch,vol,vol,buf);return real_output_block(ch,vol,buf);}
static int hook_panned(int ch,int lv,int rv,void*buf){tap_normal(ch,lv,rv,buf);return real_panned(ch,lv,rv,buf);}
static int hook_panned_block(int ch,int lv,int rv,void*buf){tap_normal(ch,lv,rv,buf);return real_panned_block(ch,lv,rv,buf);}

/* ---- Output2 / SRC / Vaudio single-stream paths ---- */
static void special_begin(struct SpecialState *s,int frames,int rate,int channels)
{
    s->reserved=1;s->frames=frames;s->rate=rate;s->channels=channels;
}
static void special_end(struct SpecialState *s){memset(s,0,sizeof(*s));}
static void tap_special(struct SpecialState *s,unsigned channel,int vol,void *buf)
{
    if(!g_audio_on||audio_pops_ready()||!buf||!s->reserved)return;
    if(s->frames<=0||s->frames>4096)return;
    if(s->channels!=1&&s->channels!=2)return;
    /* AUD1 v1/browser clock is 44.1 kHz.  Never mis-time another rate: leave
     * the game's audio untouched and simply skip capture until a future wire
     * protocol explicitly carries sample rate. */
    if(s->rate!=44100)return;
    audio_push_pcm(AUDIO_SOURCE_PSP,channel,buf,(unsigned)s->frames,
                   s->channels==1,clamp_vol(vol),clamp_vol(vol),
                   sceKernelGetSystemTimeLow());
}

static int hook_o2_reserve(int frames){int r=real_o2_reserve(frames);if(r>=0)special_begin(&g_output2,frames,44100,2);return r;}
static int hook_o2_release(void){int r=real_o2_release();if(r>=0)special_end(&g_output2);return r;}
static int hook_o2_len(int frames){int r=real_o2_len(frames);if(r>=0&&g_output2.reserved)g_output2.frames=frames;return r;}
static int hook_o2_block(int vol,void*buf){tap_special(&g_output2,8u,vol,buf);return real_o2_block(vol,buf);}

static int hook_src_reserve(int frames,int rate,int channels){int r=real_src_reserve(frames,rate,channels);if(r>=0)special_begin(&g_src,frames,rate,channels);return r;}
static int hook_src_release(void){int r=real_src_release();if(r>=0)special_end(&g_src);return r;}
static int hook_src_block(int vol,void*buf){tap_special(&g_src,9u,vol,buf);return real_src_block(vol,buf);}

static int hook_va_reserve(int frames,int rate,int fmt){int r=real_va_reserve(frames,rate,fmt);if(r>=0)special_begin(&g_vaudio,frames,rate,fmt==VAUDIO_FORMAT_STEREO?2:1);return r;}
static int hook_va_release(void){int r=real_va_release();if(r>=0)special_end(&g_vaudio);return r;}
static int hook_va_block(int vol,void*buf){tap_special(&g_vaudio,10u,vol,buf);return real_va_block(vol,buf);}

void audio_install_psp_hooks(void)
{
    int cache_dirty=0;
    /* Resolve fresh each passive scan; only missing bits are patched. */
    if(!(g_hook_mask&H_CH_RESERVE)) real_ch_reserve=(void*)find_audio(NID_CH_RESERVE);
    if(!(g_hook_mask&H_CH_RELEASE)) real_ch_release=(void*)find_audio(NID_CH_RELEASE);
    if(!(g_hook_mask&H_CH_LEN)) real_set_len=(void*)find_audio(NID_SET_DATA_LEN);
    if(!(g_hook_mask&H_CH_CONFIG)) real_config=(void*)find_audio(NID_CHANGE_CONFIG);
    if(!(g_hook_mask&H_OUT)) real_output=(void*)find_audio(NID_OUTPUT);
    if(!(g_hook_mask&H_OUT_BLOCK)) real_output_block=(void*)find_audio(NID_OUTPUT_BLOCKING);
    if(!(g_hook_mask&H_PAN)) real_panned=(void*)find_audio(NID_PANNED);
    if(!(g_hook_mask&H_PAN_BLOCK)) real_panned_block=(void*)find_audio(NID_PANNED_BLOCKING);

    /* Only hook normal outputs when state tracking is available. */
    if(real_ch_reserve&&real_ch_release&&real_set_len&&real_config){
        const u32 track=H_CH_RESERVE|H_CH_RELEASE|H_CH_LEN|H_CH_CONFIG;
        cache_dirty|=install_unique((void*)real_ch_reserve,(void*)hook_ch_reserve,H_CH_RESERVE);
        cache_dirty|=install_unique((void*)real_ch_release,(void*)hook_ch_release,H_CH_RELEASE);
        cache_dirty|=install_unique((void*)real_set_len,(void*)hook_ch_len,H_CH_LEN);
        cache_dirty|=install_unique((void*)real_config,(void*)hook_ch_config,H_CH_CONFIG);
        if((g_hook_mask&track)==track){
            if(real_output)cache_dirty|=install_unique((void*)real_output,(void*)hook_output,H_OUT);
            if(real_output_block)cache_dirty|=install_unique((void*)real_output_block,(void*)hook_output_block,H_OUT_BLOCK);
            if(real_panned)cache_dirty|=install_unique((void*)real_panned,(void*)hook_panned,H_PAN);
            if(real_panned_block)cache_dirty|=install_unique((void*)real_panned_block,(void*)hook_panned_block,H_PAN_BLOCK);
        }
    }

    if(!(g_hook_mask&H_O2_RESERVE))real_o2_reserve=(void*)find_audio(NID_OUTPUT2_RESERVE);
    if(!(g_hook_mask&H_O2_RELEASE))real_o2_release=(void*)find_audio(NID_OUTPUT2_RELEASE);
    if(!(g_hook_mask&H_O2_BLOCK))real_o2_block=(void*)find_audio(NID_OUTPUT2_BLOCKING);
    if(!(g_hook_mask&H_O2_LEN))real_o2_len=(void*)find_audio(NID_OUTPUT2_CHANGE_LENGTH);
    if(real_o2_reserve&&real_o2_release&&real_o2_block){
        cache_dirty|=install_unique((void*)real_o2_reserve,(void*)hook_o2_reserve,H_O2_RESERVE);
        cache_dirty|=install_unique((void*)real_o2_release,(void*)hook_o2_release,H_O2_RELEASE);
        if((g_hook_mask&(H_O2_RESERVE|H_O2_RELEASE))==(H_O2_RESERVE|H_O2_RELEASE)){
            cache_dirty|=install_unique((void*)real_o2_block,(void*)hook_o2_block,H_O2_BLOCK);
            if(real_o2_len)cache_dirty|=install_unique((void*)real_o2_len,(void*)hook_o2_len,H_O2_LEN);
        }
    }

    if(!(g_hook_mask&H_SRC_RESERVE))real_src_reserve=(void*)find_audio(NID_SRC_RESERVE);
    if(!(g_hook_mask&H_SRC_RELEASE))real_src_release=(void*)find_audio(NID_SRC_RELEASE);
    if(!(g_hook_mask&H_SRC_BLOCK))real_src_block=(void*)find_audio(NID_SRC_BLOCKING);
    if(real_src_reserve&&real_src_release&&real_src_block){
        cache_dirty|=install_unique((void*)real_src_reserve,(void*)hook_src_reserve,H_SRC_RESERVE);
        cache_dirty|=install_unique((void*)real_src_release,(void*)hook_src_release,H_SRC_RELEASE);
        if((g_hook_mask&(H_SRC_RESERVE|H_SRC_RELEASE))==(H_SRC_RESERVE|H_SRC_RELEASE))
            cache_dirty|=install_unique((void*)real_src_block,(void*)hook_src_block,H_SRC_BLOCK);
    }

    if(!(g_hook_mask&H_VA_RESERVE))real_va_reserve=(void*)find_vaudio(NID_VAUDIO_RESERVE);
    if(!(g_hook_mask&H_VA_RELEASE))real_va_release=(void*)find_vaudio(NID_VAUDIO_RELEASE);
    if(!(g_hook_mask&H_VA_BLOCK))real_va_block=(void*)find_vaudio(NID_VAUDIO_BLOCKING);
    if(real_va_reserve&&real_va_release&&real_va_block){
        cache_dirty|=install_unique((void*)real_va_reserve,(void*)hook_va_reserve,H_VA_RESERVE);
        cache_dirty|=install_unique((void*)real_va_release,(void*)hook_va_release,H_VA_RELEASE);
        if((g_hook_mask&(H_VA_RESERVE|H_VA_RELEASE))==(H_VA_RESERVE|H_VA_RELEASE))
            cache_dirty|=install_unique((void*)real_va_block,(void*)hook_va_block,H_VA_BLOCK);
    }

    if(cache_dirty)sctrlFlushCache();
}


void audio_restore_psp_hooks(void)
{
    int i;
    /* A syscall patch is restored by pointing the syscall back at its original
     * implementation address.  Restore each concrete address once. */
    for(i=(int)g_patch_count-1;i>=0;i--)
        if(g_patches[i].addr)sctrlHENPatchSyscall(g_patches[i].addr,g_patches[i].addr);
    if(g_patch_count)sctrlFlushCache();
    g_patch_count=0;g_hook_mask=0;
}
