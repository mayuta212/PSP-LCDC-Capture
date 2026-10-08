/*
 * PSP LCDC Capture — frozen 2026-10-08
 * GAME: A02 universal PCM capture + PUSHOUT02 game-first video transport.
 * POPS: retained A15 path. EP81 carries video A; EP82 carries video B + AUD1.
 * Native 16/32bpp LCDC scanout, no PSP-side pixel conversion, no video wait.
 */
#include <pspkernel.h>
#include <pspinit.h>
#include <pspsdk.h>
#include <pspusb.h>
#include <pspdisplay.h>
#include <pspmoduleinfo.h>
#include <systemctrl.h>
#include <string.h>
#include <stdint.h>
#include "pspusbbus_local.h"
#include "protocol.h"
#include "audio_capture.h"
extern void audio_pops_start(void);
extern void audio_pops_drain(void);
extern void audio_pops_stop(void);

PSP_MODULE_INFO("LCDC60", PSP_MODULE_KERNEL, 0, 60);
PSP_MAIN_THREAD_ATTR(0);
#define DRIVER "LCDC60Driver"
#define PID 0x01C9
#define START_DELAY_US 1000000u
#define LCDC_ADDR 0xBC800100u
#define LCDC_FMT 0xBC800104u
#define LCDC_WIDTH 0xBC800108u
#define LCDC_STRIDE_REG 0xBC80010Cu
#define LCDC_CTRL 0xBC800110u
#define MMIO32(addr) (*(volatile const u32 *)(addr))
#define VRAM_LO 0x04000000u
#define VRAM_HI 0x04400000u
#define RAM_LO 0x08000000u
#define RAM_HI 0x0C000000u
#define DMAC_NID 0x617F3FE6u
#define FREE 0
#define BUSY 1

typedef int (*DmacMemcpyFn)(void *, const void *, SceSize);
struct Scanout {u32 addr,fmt,width,stride,ctrl;};

static volatile int g_run=1,g_attached=0,g_registered=0,g_driver_started=0,g_activated=0;
/* 0=not a supported game, 1=GAME or ARK homebrew, 2=POPS.
 * Init mode is set once before starting USB/audio workers. */
#define LCDC_MODE_GAME 1
#define LCDC_MODE_POPS 2
static int g_mode=0;
static SceUID g_starter=-1,g_capture=-1,g_block=-1,g_audio_worker=-1,g_video_sender=-1;
static int g_alloc_attempted=0,g_next=0;
static unsigned char *g_frame[2]={NULL,NULL};
static volatile int g_busy[2]={FREE,FREE};
static volatile int g_pending[2]={0,0};
static struct UsbdDeviceReq g_hdr_req[2][2] __attribute__((aligned(64)));
static struct UsbdDeviceReq g_data_req[2][2] __attribute__((aligned(64)));
/* Each 24-byte header must have its OWN 64B-aligned USB source slot. */
union HeaderSlot {struct Lcdc60DualHeader hdr; u8 raw[64];} __attribute__((aligned(64)));
static union HeaderSlot g_headers[2][2] __attribute__((aligned(64)));
static DmacMemcpyFn g_dmac=NULL;
static volatile int g_dmac_active=0;
static u32 g_seq=0;
static volatile u32 g_send_fail=0,g_usb_fail=0;
static volatile int g_ready_bank=-1;
static volatile int g_video_submit_active=0;
static volatile int g_ready_fmt=0;
static volatile u32 g_ready_bytes=0;
static const char g_build_signature[] __attribute__((used)) = "PSP_LCDC_CAPTURE_20261008";

static struct UsbEndpoint g_ep[3]={{0,0,0},{1,0,0},{2,0,0}};
static struct UsbInterface g_if={-1,0,1};
static struct UsbData g_ud[2] __attribute__((aligned(64)));
static struct StringDescriptor g_str={14,3,{'L','C','D','C','6','0',0}};
static const struct DeviceDescriptor g_dev={18,1,0x0200,0,0,0,64,0,0,0x0100,0,0,0,1};
static const struct ConfigDescriptor g_cfg={9,2,9+9+7+7,1,1,0,0xC0,0};
static const struct InterfaceDescriptor g_inf={9,4,0,0,2,0xFF,0,0,1};
static const struct EndpointDescriptor g_hi[2]={{7,5,0x81,2,512,0},{7,5,0x82,2,512,0}};
static const struct EndpointDescriptor g_full[2]={{7,5,0x81,2,64,0},{7,5,0x82,2,64,0}};

static void usb_data_init(struct UsbData *u,int high)
{
    memset(u,0,sizeof(*u));
    memcpy(u->devdesc,&g_dev,sizeof(g_dev));
    u->config.pconfdesc=&u->confdesc;
    u->config.pinterfaces=&u->interfaces;
    u->config.pinterdesc=&u->interdesc;
    u->config.pendp=&u->endp[0];
    memcpy(u->confdesc.desc,&g_cfg,sizeof(g_cfg));
    u->confdesc.pinterfaces=&u->interfaces;
    u->interfaces.pinterdesc[0]=&u->interdesc;
    u->interfaces.pinterdesc[1]=NULL;
    u->interfaces.intcount=1;
    memcpy(u->interdesc.desc,&g_inf,sizeof(g_inf));
    u->interdesc.pendp=&u->endp[0];
    memcpy(u->endp[0].desc,high?&g_hi[0]:&g_full[0],sizeof(g_hi[0]));
    memcpy(u->endp[1].desc,high?&g_hi[1]:&g_full[1],sizeof(g_hi[1]));
    sceKernelDcacheWritebackRange(u,sizeof(*u));
}
static int usb_control(int a,int b,struct DeviceRequest*r){(void)a;(void)b;(void)r;return 0;}
static int usb_unknown(int a,int b,int c){(void)a;(void)b;(void)c;return 0;}
static void game_drop_ready(void);
static int usb_attach(int speed,void*a,void*b)
{
    (void)a;(void)b;
    (void)speed;
    g_attached=1;
    audio_enable(1);
    if(g_video_sender>=0)sceKernelWakeupThread(g_video_sender);
    return 0;
}
static int usb_detach(int a,int b,int c){(void)a;(void)b;(void)c;g_attached=0;audio_enable(0);if(g_mode==LCDC_MODE_GAME)game_drop_ready();return 0;}
static int usb_start(int n,void*a){(void)n;(void)a;usb_data_init(&g_ud[0],1);usb_data_init(&g_ud[1],0);return 0;}
static int usb_stop(int n,void*a){(void)n;(void)a;g_attached=0;audio_enable(0);if(g_mode==LCDC_MODE_GAME)game_drop_ready();return 0;}
static struct UsbDriver g_driver={DRIVER,3,g_ep,&g_if,
    &g_ud[0].devdesc[0],&g_ud[0].config,&g_ud[1].devdesc[0],&g_ud[1].config,
    &g_str,usb_control,usb_unknown,usb_attach,usb_detach,0,usb_start,usb_stop,NULL};

static void release_one(int bank)
{
    int old=pspSdkDisableInterrupts();
    if(g_pending[bank]>0) g_pending[bank]--;
    if(g_pending[bank]==0)g_busy[bank]=FREE;
    pspSdkEnableInterrupts(old);
}
static int data_done(struct UsbdDeviceReq*r,int a,int b)
{
    int bank,ep; (void)a;(void)b;
    for(bank=0;bank<2;bank++)for(ep=0;ep<2;ep++)if(r==&g_data_req[bank][ep]){
        if(r->retcode!=0 || r->recvsize!=r->size)g_usb_fail++;
        release_one(bank);return 0;
    }
    return 0;
}
static int header_done(struct UsbdDeviceReq*r,int a,int b)
{
    int bank,ep; (void)a;(void)b;
    for(bank=0;bank<2;bank++)for(ep=0;ep<2;ep++)if(r==&g_hdr_req[bank][ep]){
        if(r->retcode!=0 || r->recvsize!=r->size)g_usb_fail++;
        release_one(bank);return 0;
    }
    return 0;
}

static void read_scanout(struct Scanout*s)
{
    s->addr=MMIO32(LCDC_ADDR)&0x1FFFFFFFu;
    s->fmt=MMIO32(LCDC_FMT)&3u;
    s->width=MMIO32(LCDC_WIDTH);
    s->stride=MMIO32(LCDC_STRIDE_REG);
    s->ctrl=MMIO32(LCDC_CTRL);
}
static int source_format(const struct Scanout*s)
{
    if(s->fmt==1)return PSP_DISPLAY_PIXEL_FORMAT_565;
    if(s->fmt==2)return PSP_DISPLAY_PIXEL_FORMAT_5551;
    if(s->fmt==3)return PSP_DISPLAY_PIXEL_FORMAT_4444;
    return PSP_DISPLAY_PIXEL_FORMAT_8888;
}
static int scanout_valid(const struct Scanout*s,int*fmt,u32*bytes)
{
    u32 end,bpp;
    if(!(s->ctrl&1u)||s->width!=480u||s->stride!=512u)return 0;
    *fmt=source_format(s);
    bpp=(*fmt==PSP_DISPLAY_PIXEL_FORMAT_8888)?4u:2u;
    *bytes=LCDC60_STRIDE*LCDC60_HEIGHT*bpp;
    if(s->addr>0xFFFFFFFFu-*bytes)return 0;
    end=s->addr+*bytes;
    if(!((s->addr>=VRAM_LO&&end<=VRAM_HI)||(s->addr>=RAM_LO&&end<=RAM_HI)))return 0;
    return 1;
}
static int same_scanout(const struct Scanout*a,const struct Scanout*b)
{
    return a->addr==b->addr&&a->fmt==b->fmt&&a->width==b->width&&a->stride==b->stride;
}
/* DMAC12's early-latch policy: choose new front if it flips during VBlank. */
static int latch_scanout(struct Scanout*out,const struct Scanout*before)
{
    struct Scanout cur;int fmt;u32 size;
    if(!g_run||!g_attached)return -1;
    read_scanout(&cur);
    if(!same_scanout(&cur,before)&&scanout_valid(&cur,&fmt,&size)){*out=cur;return 0;}
    while(g_run&&g_attached&&sceDisplayIsVblank()){
        read_scanout(&cur);
        if(!same_scanout(&cur,before)&&scanout_valid(&cur,&fmt,&size)){*out=cur;return 0;}
        sceKernelDelayThread(50);
    }
    if(!g_run||!g_attached)return -1;
    read_scanout(&cur);
    if(!scanout_valid(&cur,&fmt,&size))return -1;
    *out=cur;return 0;
}
static int reserve_bank(void)
{
    int bank=-1,old=pspSdkDisableInterrupts();
    int first=g_next;
    if(g_busy[first]==FREE)bank=first;
    else if(g_busy[first^1]==FREE)bank=first^1;
    if(bank>=0){g_busy[bank]=BUSY;g_next=bank^1;}
    pspSdkEnableInterrupts(old);
    return bank;
}
static void release_unused_bank(int bank)
{
    int old=pspSdkDisableInterrupts();
    if(g_pending[bank]==0)g_busy[bank]=FREE;
    pspSdkEnableInterrupts(old);
}
/* PUSHOUT02: one video frame may be in USB and one completed frame may wait
 * in RAM.  There is still no target FPS or timing heuristic.  Capture never
 * waits for transport; the waiting frame is submitted by a very low-priority
 * sender as soon as USB/video and EP82 audio pressure permit. */
static int game_video_usb_busy(void)
{
    int busy,old=pspSdkDisableInterrupts();
    busy=(g_pending[0]!=0 || g_pending[1]!=0 || g_video_submit_active);
    pspSdkEnableInterrupts(old);
    return busy;
}
static int game_ready_exists(void)
{
    int ready,old=pspSdkDisableInterrupts();
    ready=(g_ready_bank>=0);
    pspSdkEnableInterrupts(old);
    return ready;
}
static void game_store_ready(int bank,int fmt,u32 bytes)
{
    int old=pspSdkDisableInterrupts();
    if(g_ready_bank<0){
        g_ready_bank=bank;
        g_ready_fmt=fmt;
        g_ready_bytes=bytes;
        bank=-1;
    }
    pspSdkEnableInterrupts(old);
    if(bank>=0){
        release_unused_bank(bank);
    }
    if(g_video_sender>=0)sceKernelWakeupThread(g_video_sender);
}
static int game_claim_ready_for_submit(int *bank,int *fmt,u32 *bytes)
{
    int old=pspSdkDisableInterrupts();
    if(g_ready_bank<0 || g_pending[0]!=0 || g_pending[1]!=0 || g_video_submit_active){
        pspSdkEnableInterrupts(old);
        return 0;
    }
    g_video_submit_active=1;
    *bank=g_ready_bank;
    *fmt=g_ready_fmt;
    *bytes=g_ready_bytes;
    g_ready_bank=-1;
    pspSdkEnableInterrupts(old);
    return 1;
}
static int game_claim_direct_submit(void)
{
    int ok=0,old=pspSdkDisableInterrupts();
    if(g_ready_bank<0 && g_pending[0]==0 && g_pending[1]==0 && !g_video_submit_active){
        g_video_submit_active=1;
        ok=1;
    }
    pspSdkEnableInterrupts(old);
    return ok;
}
static void game_finish_submit(void)
{
    int old=pspSdkDisableInterrupts();
    g_video_submit_active=0;
    pspSdkEnableInterrupts(old);
}
static void game_drop_ready(void)
{
    int bank=-1,old=pspSdkDisableInterrupts();
    if(g_ready_bank>=0){
        bank=g_ready_bank;
        g_ready_bank=-1;
    }
    g_video_submit_active=0;
    pspSdkEnableInterrupts(old);
    if(bank>=0)release_unused_bank(bank);
}
static void resolve_dmac(void)
{
    if(!g_dmac)g_dmac=(DmacMemcpyFn)(uintptr_t)sctrlHENFindFunction(
        "sceLowIO_Driver","sceDmac",DMAC_NID);
}
static int init_banks(void)
{
    uintptr_t head,aligned,phys;u32 size=2u*LCDC60_BYTES32;
    if(g_frame[0]&&g_frame[1])return 0;
    if(g_alloc_attempted)return -1;
    g_alloc_attempted=1;
    g_block=sceKernelAllocPartitionMemory(PSP_MEMORY_PARTITION_USER,
        "LCDC60Dual32",PSP_SMEM_High,size+63u,NULL);
    if(g_block<0)return -1;
    head=(uintptr_t)sceKernelGetBlockHeadAddr(g_block);
    aligned=(head+63u)&~(uintptr_t)63u;
    phys=aligned&0x1FFFFFFFu;
    if(!head||phys<RAM_LO||phys>=RAM_HI||size>RAM_HI-phys){
        sceKernelFreePartitionMemory(g_block);g_block=-1;return -1;
    }
    memset((void*)aligned,0,size);
    sceKernelDcacheWritebackInvalidateRange((void*)aligned,size);
    g_frame[0]=(unsigned char*)aligned;
    g_frame[1]=g_frame[0]+LCDC60_BYTES32;
    /* AUDIO07: allocation/clear identical to stable AUDIO06. */
    return 0;
}
/* No CPU copies/conversions. DMAC12 retry on a front-buffer flip during DMAC. */
static int capture_native(int bank,const struct Scanout*front,u32*size,int*fmt)
{
    struct Scanout a=*front,after;
    u32 bytes=0;int f=0,ret,attempt;
    int max_attempts=(g_mode==LCDC_MODE_GAME)?1:2;
    uintptr_t dest=(uintptr_t)g_frame[bank]&0x1FFFFFFFu;
    if(!g_dmac||!scanout_valid(&a,&f,&bytes))return -1;
    for(attempt=0;attempt<max_attempts;attempt++){
        if(attempt){
            if(!scanout_valid(&a,&f,&bytes))return -1;
        }
        g_dmac_active=1;
        /* A 32-bit frame is copied as TWO proven 0x44000-byte DMAC
         * operations instead of one untested 0x88000-byte operation. */
        ret=g_dmac((void*)dest,(const void*)(uintptr_t)a.addr,LCDC60_BYTES16);
        if(ret==0 && bytes==LCDC60_BYTES32)
            ret=g_dmac((void*)(dest+LCDC60_BYTES16),
                       (const void*)(uintptr_t)(a.addr+LCDC60_BYTES16),LCDC60_BYTES16);
        g_dmac_active=0;
        read_scanout(&after);
        if(ret!=0)return -1;
        if(same_scanout(&a,&after)&&a.ctrl==after.ctrl){
            *size=bytes;*fmt=f;
            /* LITE01/LITE02: frame banks are written by DMAC, not CPU. They were
             * writeback-invalidated once after allocation and the CPU never
             * writes frame payload bytes afterwards. Avoid flushing the
             * entire CPU D-cache at 60 Hz; USB consumes the DMAC-written RAM. */
            return 0;
        }
        if(after.fmt!=a.fmt||after.stride!=a.stride)return -1;
        if(g_mode==LCDC_MODE_GAME && max_attempts==1){
            /* Never pay for a second full-frame DMAC just to rescue capture.
             * A mid-copy flip drops this external frame; the game keeps the bus. */
            break;
        }
        a=after;
    }
    return -1;
}

/* POPS waits for a valid LCDC surface before starting video capture. */
static int queue_native_frame(int bank,int fmt,u32 bytes);
static int capture_thread(SceSize n,void*a);
static int video_sender_thread(SceSize n,void*a);
static void run_video_phase(void)
{
    struct Scanout s;
    u32 bytes=0, began;
    int fmt=0;
    if(!g_run||!g_attached||!g_frame[0]||!g_frame[1])return;

    /* POPS may still be switching display surfaces here. Keep the proven
     * strict LCDC validity gate, but no diagnostic framebuffer probes/log I/O. */
    began=sceKernelGetSystemTimeLow();
    while(g_run && g_attached){
        read_scanout(&s);
        if(scanout_valid(&s,&fmt,&bytes))break;
        if((u32)(sceKernelGetSystemTimeLow()-began)>=15000000u)return;
        sceKernelDelayThread(100000);
    }
    if(!g_run||!g_attached)return;

    resolve_dmac();
    if(!g_dmac)return;
    if(capture_native(0,&s,&bytes,&fmt)!=0)return;

    /* POPS keeps the established capture priority/path. */
    g_capture=sceKernelCreateThread("LCDC60VideoOnly",capture_thread,
                                     0x30,0x2000,0,NULL);
    if(g_capture>=0){
        int started=sceKernelStartThread(g_capture,0,NULL);
        if(started!=0){
            sceKernelDeleteThread(g_capture);
            g_capture=-1;
        }
    }
}

static int queue_native_frame(int bank,int fmt,u32 bytes)
{
    u32 part0=LCDC60_PART_SIZE(bytes),seq=++g_seq;
    unsigned char*base=g_frame[bank];
    int ep,k,ret,n=4;
    /* Both parts are 64B aligned. Their sizes are intentionally NOT
     * multiples of 512 to finish each request with a short packet.
     * part0 + part1 == exact native frame size, no missing byte. */
    for(ep=0;ep<2;ep++){
        struct Lcdc60DualHeader*h=&g_headers[bank][ep].hdr;
        memset(h,0,sizeof(*h));
        h->magic=LCDC60_MAGIC;h->tag=LCDC60_DUAL_TAG;
        h->version=LCDC60_DUAL_VERSION;
        h->sequence=seq;h->frame_bytes=bytes;
        h->part0_bytes=part0;h->format=(u8)fmt;h->endpoint=(u8)(ep+1);
        sceKernelDcacheWritebackRange(h,sizeof(*h));
        memset(&g_hdr_req[bank][ep],0,sizeof(struct UsbdDeviceReq));
        g_hdr_req[bank][ep].endp=&g_ep[ep+1];
        g_hdr_req[bank][ep].data=h;
        g_hdr_req[bank][ep].size=sizeof(*h);
        g_hdr_req[bank][ep].func=(void*)header_done;
        memset(&g_data_req[bank][ep],0,sizeof(struct UsbdDeviceReq));
        g_data_req[bank][ep].endp=&g_ep[ep+1];
        g_data_req[bank][ep].data=base+(ep?part0:0u);
        g_data_req[bank][ep].size=ep?(int)(bytes-part0):(int)part0;
        g_data_req[bank][ep].func=(void*)data_done;
    }
    /* Reserve ALL four callbacks first to prevent a small header's callback
     * from freeing this bank before the two payload requests are submitted. */
    g_pending[bank]=4;
    for(ep=0;ep<2;ep++){
        for(k=0;k<2;k++){
            struct UsbdDeviceReq*req=k?&g_data_req[bank][ep]:&g_hdr_req[bank][ep];
            ret=sceUsbbdReqSend(req);
            n--;
            if(ret<0){
                g_send_fail++;
                /* Only requests ALREADY submitted may run callbacks. */
                {
                    int old=pspSdkDisableInterrupts();
                    g_pending[bank]-=(n+1);
                    if(g_pending[bank]==0)g_busy[bank]=FREE;
                    pspSdkEnableInterrupts(old);
                }
                return -1;
            }
        }
    }
    return 0;
}
static int video_sender_thread(SceSize n,void*a)
{
    (void)n;(void)a;
    while(g_run){
        int bank,fmt;
        u32 bytes;
        if(!g_attached || !game_ready_exists()){
            sceKernelSleepThread();
            continue;
        }
        /* Only this short low-priority loop waits for transport slack.  It does
         * no DMAC/copy work.  Audio remains ahead of video on shared EP82. */
        if(game_video_usb_busy() || audio_waiting_packets()!=0){
            sceKernelDelayThread(250);
            continue;
        }
        if(!game_claim_ready_for_submit(&bank,&fmt,&bytes))continue;
        if(queue_native_frame(bank,fmt,bytes)<0){
            game_finish_submit();
            release_unused_bank(bank);
        }else{
            game_finish_submit();
        }
    }
    sceKernelExitDeleteThread(0);return 0;
}
static int capture_thread(SceSize n,void*a)
{
    (void)n;(void)a;
    while(g_run){
        struct Scanout before,front;int bank,fmt;
        u32 bytes;
        if(!g_attached){sceKernelDelayThread(10000);continue;}
        /* POPS keeps AUDIO15's USB health checks. GAME stays event/occupancy
         * driven and never polls USB state per frame. */
        if(g_mode==LCDC_MODE_POPS){
            if(g_send_fail || g_usb_fail){
                break;
            }
            if((sceUsbGetState() & 0x222u) != 0x222u){
                sceKernelDelayThread(10000);
                continue;
            }
        }
        if(init_banks()<0){sceKernelDelayThread(100000);continue;}
        read_scanout(&before);
        sceDisplayWaitVblankStart();
        if(!g_run||!g_attached)continue;

        if(g_mode==LCDC_MODE_GAME && game_ready_exists()){
            /* One completed waiting frame is the entire look-ahead budget.
             * Never build a longer queue and never overwrite it with more DMAC. */
            continue;
        }

        bank=reserve_bank();
        if(bank<0)continue;
        if(latch_scanout(&front,&before)<0){release_unused_bank(bank);continue;}
        if(capture_native(bank,&front,&bytes,&fmt)==0){
            if(g_mode==LCDC_MODE_GAME){
                /* Fast path: if transport is free and audio has no waiting
                 * packet, send this fresh frame immediately from this VBlank.
                 * Otherwise hold exactly this one completed frame in RAM; the
                 * sender pushes it the moment transport slack appears. */
                if(audio_waiting_packets()==0 && game_claim_direct_submit()){
                    if(queue_native_frame(bank,fmt,bytes)<0){
                        game_finish_submit();
                        release_unused_bank(bank);
                    }else game_finish_submit();
                }else{
                    game_store_ready(bank,fmt,bytes);
                }
            }else{
                if(queue_native_frame(bank,fmt,bytes)<0)
                    release_unused_bank(bank);
            }
        }else release_unused_bank(bank);
    }
    sceKernelExitDeleteThread(0);return 0;
}
/* GAME A02 UNIVERSAL AUDIO.  Module-start notification is deliberately
 * passive: the handler only requests another resolver pass.  The existing
 * audio worker performs the actual SystemControl lookup/patch work later.
 * No game name, retry timer, or no-audio timeout is used. */
static STMOD_HANDLER g_prev_start_handler=NULL;
static volatile int g_start_handler_installed=0;
static volatile int g_audio_rescan_pending=0;
static int lcdc60_start_module_handler(SceModule *mod);
static int lcdc60_start_module_handler(SceModule *mod)
{
    if(g_run && g_mode==LCDC_MODE_GAME && mod){
        g_audio_rescan_pending=1;
    }
    if(g_prev_start_handler)return g_prev_start_handler(mod);
    return 0;
}
/* Audio transport is independent from capture/VBlank. It does not call USB
 * inside PSP's audio hooks or POPS ME callback. Overflow drops audio only. */
static int audio_thread(SceSize n,void*a)
{
    (void)n;(void)a;
    while(g_run){
        if(g_mode==LCDC_MODE_GAME && g_audio_rescan_pending){
            int irq=pspSdkDisableInterrupts();
            g_audio_rescan_pending=0;
            pspSdkEnableInterrupts(irq);
            audio_install_psp_hooks();
        }
        if(g_attached){
            /* PSP: AUDIO01 sends raw captured channels. POPS: A15 drain. */
            if(g_mode==LCDC_MODE_POPS)audio_pops_drain();
            /* Share EP82 with video. One audio request per wake avoids audio bursts
             * jumping too far ahead of video while still allowing >600 packets/s. */
            audio_send_available(&g_ep[AUDIO_EP_INDEX],1);
            sceKernelDelayThread(1500);
        }else sceKernelDelayThread(10000);
    }
    sceKernelExitDeleteThread(0);return 0;
}
static void shutdown_usb(void)
{
    g_attached=0;
    audio_enable(0);
    if(g_registered){sceUsbbdReqCancelAll(&g_ep[1]);audio_shutdown(&g_ep[2]);}
    if(g_activated){sceUsbDeactivate(PID);g_activated=0;}
    if(g_driver_started){sceUsbStop(DRIVER,0,NULL);g_driver_started=0;}
    /* DMAC12 policy: leave shared USB bus alone during GAME unload. */
    if(g_registered){sceUsbbdUnregister(&g_driver);g_registered=0;}
}
static int usb_start_capture(void)
{
    int ret;
    /* AUDIO01 resolves DMAC before activating USB; POPS uses A15 order. */
    if(g_mode==LCDC_MODE_GAME)resolve_dmac();
    /* Retain AUDIO15's shared-bus handling (already-started USB bus). */
    ret=sceUsbbdRegister(&g_driver);
    if(ret<0)return ret;g_registered=1;
    ret=sceUsbStart(PSP_USBBUS_DRIVERNAME,0,NULL);
    if(ret==0){
        /* We intentionally leave the shared USB bus running on unload. */
    }else if((u32)ret==0x80243001u){
        /* SCE_USB_ERROR_ALREADY: POPS/VSH started the shared USB bus. */
        /* Do not claim ownership or stop the shared bus on unload. */
    }else goto fail;
    ret=sceUsbStart(DRIVER,0,NULL);
    if(ret!=0)goto fail;g_driver_started=1;
    ret=sceUsbActivate(PID);
    if(ret!=0)goto fail;g_activated=1;
    if(g_mode==LCDC_MODE_GAME){
        /* Exactly AUDIO01's direct launch and thread resource settings. */
        /* LITE02 GAME-FIRST: capture is deliberately lower priority than the
         * unchanged 0x30 audio transport. Under load, drop capture work before
         * stealing time from game/audio. */
        g_video_sender=sceKernelCreateThread("LCDC60VideoSend",video_sender_thread,0x3C,0x1800,0,NULL);
        if(g_video_sender<0){ret=(int)g_video_sender;goto fail;}
        ret=sceKernelStartThread(g_video_sender,0,NULL);
        if(ret!=0){sceKernelDeleteThread(g_video_sender);g_video_sender=-1;goto fail;}
        g_capture=sceKernelCreateThread("LCDC60Dual32",capture_thread,0x38,0x4000,0,NULL);
        if(g_capture<0){ret=(int)g_capture;goto fail;}
        ret=sceKernelStartThread(g_capture,0,NULL);
        if(ret!=0){sceKernelDeleteThread(g_capture);g_capture=-1;goto fail;}
        g_audio_worker=sceKernelCreateThread("LCDC60AudioUSB",audio_thread,0x30,0x2000,0,NULL);
        if(g_audio_worker>=0){
            int ar=sceKernelStartThread(g_audio_worker,0,NULL);
            if(ar!=0){sceKernelDeleteThread(g_audio_worker);g_audio_worker=-1;}
        }
    }
    /* POPS capture thread is still started solely by run_video_phase(). */
    return 0;
fail:shutdown_usb();return ret;
}
static int starter(SceSize n,void*a)
{
    (void)n;(void)a;
    sceKernelDelayThread(START_DELAY_US);
    if(g_run && usb_start_capture()==0 && g_mode==LCDC_MODE_POPS){
        int tries=0;
        while(g_run && !g_attached && tries<200){
            sceKernelDelayThread(10000);
            tries++;
        }
        if(g_run && g_attached && init_banks()==0)
            run_video_phase();
    }
    sceKernelExitDeleteThread(0);return 0;
}

int module_start(SceSize n,void*a)
{
    int key, api;
    (void)n;(void)a;
    g_run=1;g_attached=0;
    key=sceKernelInitKeyConfig();
    api=sceKernelInitApitype();
    /* GAME=0x200, POPS=0x300; ARK Touhou homebrew reported VSH=0x100
     * plus apitype=0x210.  Never activate in the actual XMB/VSH. */
    if(key==PSP_INIT_KEYCONFIG_POPS)g_mode=LCDC_MODE_POPS;
    else if(key==PSP_INIT_KEYCONFIG_GAME ||
            (key==PSP_INIT_KEYCONFIG_VSH && api==0x210))g_mode=LCDC_MODE_GAME;
    else g_mode=0;
    if(!g_mode)return 0; /* no capture/hooks in the actual XMB */
    audio_init();
    if(g_mode==LCDC_MODE_POPS){
        /* AUDIO15: only POPS ME tap; install early before registration. */
        audio_pops_start();
        g_audio_worker=sceKernelCreateThread("LCDC60PopsAudio",audio_thread,0x35,0x2000,0,NULL);
        if(g_audio_worker>=0){
            int ar=sceKernelStartThread(g_audio_worker,0,NULL);
            if(ar!=0){sceKernelDeleteThread(g_audio_worker);g_audio_worker=-1;}
        }
    }else{
        /* A02: install every currently available public PCM path now, then
         * keep one passive StartModuleHandler for paths that appear later.
         * Audio may remain silent for an arbitrary time without changing state. */
        audio_install_psp_hooks();
        g_prev_start_handler=sctrlHENSetStartModuleHandler(lcdc60_start_module_handler);
        g_start_handler_installed=1;
        /* Guarantee one resolver pass from the worker after USB startup even if
         * all interesting module starts happened during the first second. */
        g_audio_rescan_pending=1;
        init_banks();
        /* AUDIO01 starts audio USB worker AFTER USB activation. */
    }
    g_starter=sceKernelCreateThread("LCDC60DualStart",starter,0x30,0x2000,0,NULL);
    if(g_starter>=0)sceKernelStartThread(g_starter,0,NULL);
    return 0;
}
int module_stop(SceSize n,void*a)
{
    (void)n;(void)a;g_run=0;
    if(g_mode==0)return 0;
    if(g_capture>=0){sceKernelTerminateDeleteThread(g_capture);g_capture=-1;}
    if(g_video_sender>=0){sceKernelTerminateDeleteThread(g_video_sender);g_video_sender=-1;}
    game_drop_ready();
    if(g_audio_worker>=0){sceKernelTerminateDeleteThread(g_audio_worker);g_audio_worker=-1;}
    /* Remove our SystemControl module-start callback before this PRX unloads,
     * then restore only the hook family actually installed in GAME. */
    if(g_mode==LCDC_MODE_GAME){
        if(g_start_handler_installed){
            sctrlHENSetStartModuleHandler(g_prev_start_handler);
            g_prev_start_handler=NULL;
            g_start_handler_installed=0;
        }
        audio_restore_psp_hooks();
    }else audio_pops_stop();
    if(g_starter>=0){sceKernelTerminateDeleteThread(g_starter);g_starter=-1;}
    shutdown_usb();
    if(g_block>=0&&!g_dmac_active&&g_pending[0]==0&&g_pending[1]==0&&g_ready_bank<0){
        sceKernelFreePartitionMemory(g_block);g_block=-1;
    }
    return 0;
}
