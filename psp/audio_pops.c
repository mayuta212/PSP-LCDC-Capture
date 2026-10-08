/* PSP POPS Media Engine stereo PCM tap, adapted from v0.19 verified pass-through.
 * Does NOT modify sample values.  No USB I/O on the ME callback.
 * Firmware/game specific; refuses patch unless exactly one known callsite found.
 */
#include <pspkernel.h>
#include <psploadcore.h>
#include <pspsysmem_kernel.h>
#include <stdint.h>
#include <string.h>
#include <systemctrl.h>
#include "audio_capture.h"

#define CORE_MODULE "pops"
#define TARGET_LIBRARY "sceMeAudio"
#define TARGET_NID 0xDE630CD2U
#define USER_BLOCK_SIZE 0xA000u
#define RELAY_OFFSET 0x9000u
#define WRAPPER_OFFSET 0x9200u
#define SAMPLE_RING 8192u

typedef struct {
    volatile uint32_t registration_calls;
    volatile uint32_t original_cb;
    volatile uint32_t ring_index;
    volatile uint32_t samples[SAMPLE_RING];
} SharedTrace;

static SceUID g_user_block_id=-1;
static uint8_t *g_user_base=NULL;
static SharedTrace *g_trace_cached=NULL;
static volatile SharedTrace *g_trace_uncached=NULL;
static uintptr_t g_stub_addr, g_callsite, g_relay_addr, g_wrapper_addr;
static uint32_t g_stub_word0,g_stub_word1,g_original_call_insn;
static int g_stub_shape_ok,g_site_found,g_relay_ok,g_wrapper_ok,g_patch_ok,g_ready;
static int g_stub_kind;
static SceUID g_thread=-1;
static volatile int g_stop;
static u32 g_pop_read;




static uintptr_t decode_jump_target(uintptr_t pc, uint32_t insn)
{
    return ((pc + 4U) & 0xF0000000U)
         | (((uintptr_t)(insn & 0x03FFFFFFU)) << 2);
}

static uint32_t encode_jal(uintptr_t target)
{
    return 0x0C000000U
         | (uint32_t)((target >> 2) & 0x03FFFFFFU);
}

static uint32_t encode_j(uintptr_t target)
{
    return 0x08000000U
         | (uint32_t)((target >> 2) & 0x03FFFFFFU);
}

static int same_jump_region(uintptr_t pc, uintptr_t target)
{
    return (((pc + 4U) & 0xF0000000U)
            == (target & 0xF0000000U));
}

static uint32_t hi16(uintptr_t x)
{
    return (uint32_t)((x >> 16) & 0xFFFFU);
}

static uint32_t lo16(uintptr_t x)
{
    return (uint32_t)(x & 0xFFFFU);
}

/* ------------------------------------------------------------------ */
/* find DE630CD2 import stub and its exact direct JAL                  */
/* ------------------------------------------------------------------ */

static int find_stub(SceModule *mod)
{
    uintptr_t cur;
    uintptr_t end;

    if (!mod || !mod->stub_top ||
        mod->stub_size < sizeof(SceLibraryStubTable))
        return -1;

    cur = (uintptr_t)mod->stub_top;
    end = cur + mod->stub_size;

    while (cur + sizeof(SceLibraryStubTable) <= end) {
        SceLibraryStubTable *st = (SceLibraryStubTable *)cur;
        uint32_t entry_bytes;
        int i;

        if (st->len == 0)
            return -2;

        entry_bytes = (uint32_t)st->len * 4U;

        if (entry_bytes < 20U || cur + entry_bytes > end)
            return -3;

        if (st->libname &&
            strcmp(st->libname, TARGET_LIBRARY) == 0 &&
            st->nidtable &&
            st->stubtable) {

            uint32_t *words = (uint32_t *)st->stubtable;

            for (i = 0; i < st->stubcount; ++i) {
                if (st->nidtable[i] != TARGET_NID)
                    continue;

                g_stub_addr =
                    (uintptr_t)st->stubtable + (uintptr_t)(i * 8);

                g_stub_word0 = words[i * 2 + 0];
                g_stub_word1 = words[i * 2 + 1];

                g_stub_shape_ok =
                    (g_stub_word0 == 0x03E00008U) &&
                    ((g_stub_word1 & 0x3FU) == 0x0CU);
                if (g_stub_shape_ok) {
                    g_stub_kind = 1; /* JR RA + syscall */
                } else if (g_stub_addr == 0x08BC1FF4U &&
                           g_stub_word0 == 0x0A2EA2C8U &&
                           g_stub_word1 == 0x34020340U) {
                    uintptr_t text = (uintptr_t)mod->text_addr;
                    uintptr_t text_end = text + (uintptr_t)mod->text_size;
                    uintptr_t target = decode_jump_target(g_stub_addr,
                                                          g_stub_word0);
                    /* Only authorize exactly the verified J + ORI form,
                     * when the jump target is inside the loaded POPS text.
                     * Refuse any different firmware/resolver patch. */
                    if (mod->text_size >= 8U &&
                        text_end >= text &&
                        target >= text &&
                        target <= text_end - 8U) {
                        g_stub_shape_ok = 1;
                        g_stub_kind = 2; /* resolved J trampoline */
                    }
                }

                return 0;
            }
        }

        cur += entry_bytes;
    }

    return -4;
}

static int find_callsite(SceModule *mod)
{
    uintptr_t pc;
    uintptr_t end;
    int count = 0;

    if (!mod || !mod->text_addr || mod->text_size < 4)
        return -1;

    end = (uintptr_t)mod->text_addr
        + (uintptr_t)(mod->text_size & ~3U);

    for (pc = (uintptr_t)mod->text_addr; pc < end; pc += 4U) {
        uint32_t insn = *(const volatile uint32_t *)pc;

        if ((insn >> 26) != 0x03U)
            continue;

        if (decode_jump_target(pc, insn) == g_stub_addr) {
            g_callsite = pc;
            g_original_call_insn = insn;
            count++;
        }
    }

    if (count == 1) {
        g_site_found = 1;
        return 0;
    }

    return -2;
}

/* ------------------------------------------------------------------ */
/* shared USER block                                                  */
/* ------------------------------------------------------------------ */

static int alloc_user_block(void)
{
    uintptr_t uncached;

    g_user_block_id = sceKernelAllocPartitionMemory(
        PSP_MEMORY_PARTITION_USER,
        "POPSMEFinalSampleTap",
        PSP_SMEM_Low,
        USER_BLOCK_SIZE,
        NULL);

    if (g_user_block_id < 0)
        return -1;

    g_user_base =
        (uint8_t *)sceKernelGetBlockHeadAddr(g_user_block_id);

    if (!g_user_base)
        return -2;

    memset(g_user_base, 0, USER_BLOCK_SIZE);

    g_trace_cached = (SharedTrace *)g_user_base;

    /*
     * PSP RAM uncached alias.  The ME-side callback also uses 0x4xxxxxxx
     * aliases, so this is the safest way to observe cross-core writes.
     */
    uncached = ((uintptr_t)g_user_base) | 0x40000000U;
    g_trace_uncached = (volatile SharedTrace *)uncached;

    g_relay_addr = (uintptr_t)g_user_base + RELAY_OFFSET;
    g_wrapper_addr = (uintptr_t)g_user_base + WRAPPER_OFFSET;

    return 0;
}

/* ------------------------------------------------------------------ */
/* registration relay                                                 */
/* ------------------------------------------------------------------ */

static void emit_registration_relay(void)
{
    volatile uint32_t *p =
        (volatile uint32_t *)g_relay_addr;

    uintptr_t trace =
        ((uintptr_t)g_trace_cached) | 0x40000000U;

    uint32_t th = hi16(trace);
    uint32_t tl = lo16(trace);
    uint32_t wh = hi16(g_wrapper_addr);
    uint32_t wl = lo16(g_wrapper_addr);

    if (!same_jump_region(g_relay_addr + 8U * 4U,
                          g_stub_addr))
        return;

    /*
     * t0/t1 only.
     *
     *   lui   t0, hi(trace_uncached)
     *   ori   t0, t0, lo(trace_uncached)
     *   lw    t1, 0(t0)
     *   addiu t1, t1, 1
     *   sw    t1, 0(t0)          registration_calls++
     *   sw    a0, 4(t0)          original callback
     *   lui   a0, hi(wrapper)
     *   ori   a0, a0, lo(wrapper)
     *   j     original import stub
     *   nop
     */
    p[0] = 0x3C080000U | th;        /* lui t0 */
    p[1] = 0x35080000U | tl;        /* ori t0,t0 */
    p[2] = 0x8D090000U;             /* lw t1,0(t0) */
    p[3] = 0x25290001U;             /* addiu t1,t1,1 */
    p[4] = 0xAD090000U;             /* sw t1,0(t0) */
    p[5] = 0xAD040004U;             /* sw a0,4(t0) */
    p[6] = 0x3C040000U | wh;        /* lui a0 */
    p[7] = 0x34840000U | wl;        /* ori a0,a0 */
    p[8] = encode_j(g_stub_addr);
    p[9] = 0x00000000U;

    g_relay_ok = 1;
}

/* ------------------------------------------------------------------ */
/* ME pass-through wrapper                                            */
/* ------------------------------------------------------------------ */

static void emit_me_wrapper(void)
{
    volatile uint32_t *p =
        (volatile uint32_t *)g_wrapper_addr;

    uintptr_t trace =
        ((uintptr_t)g_trace_cached) | 0x40000000U;

    uint32_t th = hi16(trace);
    uint32_t tl = lo16(trace);

    /*
     * ABI:
     *   original callback returns packed stereo int16 in v0.
     *
     * Wrapper:
     *   - calls original callback
     *   - appends returned v0 to the shared sample ring
     *   - returns v0 unchanged
     *
     * Only caller-saved t0/t2/t3/t9 are used.
     * RA is preserved on the existing ME stack.
     *
     * sample ring address:
     *   trace + 0x0c + ((ring_index & 8191) << 2)
     */
    int n = 0;

    p[n++] = 0x27BDFFF0U;            /* addiu sp,sp,-16 */
    p[n++] = 0xAFBF0000U;            /* sw ra,0(sp) */

    p[n++] = 0x3C080000U | th;       /* lui t0,hi(trace) */
    p[n++] = 0x35080000U | tl;       /* ori t0,t0,lo */
    p[n++] = 0x8D190004U;            /* lw t9,4(t0) */
    p[n++] = 0x0320F809U;            /* jalr t9 */
    p[n++] = 0x00000000U;            /* nop */

    /* original callback may clobber every caller-saved register.
     * Keep only the functional ring append; all per-sample diagnostics
     * were removed for the frozen build. */
    p[n++] = 0x3C080000U | th;       /* lui t0,hi(trace) */
    p[n++] = 0x35080000U | tl;       /* ori t0,t0,lo */
    p[n++] = 0x8D0A0008U;            /* lw t2,8(t0): ring_index */
    p[n++] = 0x314B1FFFU;            /* andi t3,t2,8191 */
    p[n++] = 0x000B5880U;            /* sll t3,t3,2 */
    p[n++] = 0x01685821U;            /* addu t3,t3,t0 */
    p[n++] = 0xAD62000CU;            /* sw v0,12(t3): samples[index] */
    p[n++] = 0x254A0001U;            /* addiu t2,t2,1 */
    p[n++] = 0xAD0A0008U;            /* sw t2,8(t0): ring_index */

    p[n++] = 0x8FBF0000U;            /* lw ra,0(sp) */
    p[n++] = 0x27BD0010U;            /* addiu sp,sp,16 */
    p[n++] = 0x03E00008U;            /* jr ra */
    p[n++] = 0x00000000U;            /* nop */

    if (n <= 64)
        g_wrapper_ok = 1;
}

/* ------------------------------------------------------------------ */
/* patch / restore                                                    */
/* ------------------------------------------------------------------ */

static void install_patch(void)
{
    uint32_t jal;

    if (!g_relay_ok ||
        !g_wrapper_ok ||
        !g_site_found)
        return;

    if (!same_jump_region(g_callsite, g_relay_addr))
        return;

    if (*(const volatile uint32_t *)g_callsite
        != g_original_call_insn)
        return;

    sctrlFlushCache();

    jal = encode_jal(g_relay_addr);
    *(volatile uint32_t *)g_callsite = jal;

    sctrlFlushCache();

    if (*(const volatile uint32_t *)g_callsite == jal)
        g_patch_ok = 1;
}

static void restore_callsite(void)
{
    if (!g_site_found || !g_callsite)
        return;

    /* Never undo a different owner's subsequent patch. */
    if (*(const volatile uint32_t *)g_callsite == encode_jal(g_relay_addr)) {
        *(volatile uint32_t *)g_callsite = g_original_call_insn;
        sctrlFlushCache();
    }
}


static int pops_install_thread(SceSize args,void *argp)
{
    SceModule *mod=NULL;
    int retry, res;
    (void)args;(void)argp;

    for(retry=0;retry<80&&!g_stop;retry++){
        mod=sceKernelFindModuleByName(CORE_MODULE);
        if(mod)break;
        sceKernelDelayThread(100000);
    }
    if(g_stop || !mod)return 0;

    res=find_stub(mod);
    if(res<0 || !g_stub_shape_ok)return 0;

    res=find_callsite(mod);
    if(res<0)return 0;

    /* The one direct callsite in AUDIO14 matched this verified POPS build.
     * Confirm the original JAL and instruction have not moved or changed. */
    if (g_stub_kind == 2 &&
        (g_callsite != 0x08B9DF18U ||
         g_stub_addr != 0x08BC1FF4U ||
         g_original_call_insn != encode_jal(g_stub_addr))) {
        return 0;
    }

    res=alloc_user_block();
    if(res<0)return 0;

    emit_registration_relay();
    emit_me_wrapper();
    if(!g_relay_ok || !g_wrapper_ok)return 0;

    sctrlFlushCache();
    install_patch();
    if(g_patch_ok)g_ready=1;
    return 0;
}
void audio_pops_start(void)
{
    g_stop=0;g_pop_read=0;
    g_thread=sceKernelCreateThread("LCDC60PopsTap",pops_install_thread,0x38,0x3000,0,NULL);
    if(g_thread>=0){
        int ret=sceKernelStartThread(g_thread,0,NULL);
        if(ret<0){
            sceKernelDeleteThread(g_thread);
            g_thread=-1;
        }
    }
}
int audio_pops_ready(void){return g_ready && g_patch_ok && g_trace_uncached &&
                                  g_trace_uncached->registration_calls!=0;}
void audio_pops_drain(void)
{
    u32 now,available,pack[512],count,i,base,t;
    int loops=0;
    if(!audio_pops_ready())return;
    now=g_trace_uncached->ring_index;
    available=now-g_pop_read;
    if(available>SAMPLE_RING)g_pop_read=now-SAMPLE_RING;
    while((now-g_pop_read)>=256u && loops++<4){
        available=now-g_pop_read;
        count=available>512u?512u:available;
        base=g_pop_read;
        for(i=0;i<count;i++) pack[i]=g_trace_uncached->samples[(base+i)&(SAMPLE_RING-1u)];
        /* Entire POPS output is already a final stereo mix. */
        t=sceKernelGetSystemTimeLow();
        audio_push_pcm(AUDIO_SOURCE_POPS,0xffu,pack,count,0,0x8000,0x8000,t);
        g_pop_read+=count;
    }
}
void audio_pops_stop(void)
{
    g_stop=1;
    if(g_patch_ok)restore_callsite();
    if(g_thread>=0){sceKernelTerminateDeleteThread(g_thread);g_thread=-1;}
    /* Intentionally retain code/shared block until process exit.  An ME
     * callback already in-flight must never execute freed memory. */
}
