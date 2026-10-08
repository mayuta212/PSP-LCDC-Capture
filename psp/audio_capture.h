#ifndef LCDC60_AUDIO_CAPTURE_H
#define LCDC60_AUDIO_CAPTURE_H
#include <psptypes.h>
#include "pspusbbus_local.h"

#define AUDIO_EP_INDEX 2
#define AUDIO_EP_ADDR 0x82
#define AUDIO_MAGIC 0x31445541u /* AUD1 */
#define AUDIO_VERSION 1u
#define AUDIO_SOURCE_POPS 1u
#define AUDIO_SOURCE_PSP 2u
#define AUDIO_FRAMES_PER_PACKET 512u
#define AUDIO_QUEUE_SLOTS 64u

struct AudioWireHeader {
    u32 magic, sequence, timestamp_us;
    u16 frames;
    u8 source, channel;
    u16 volume_l, volume_r;
    u16 flags, version; /* flags bit0=mono */
} __attribute__((packed));
typedef char audio_wire_header_size[(sizeof(struct AudioWireHeader) == 24) ? 1 : -1];

void audio_init(void);
void audio_install_psp_hooks(void);
void audio_restore_psp_hooks(void);
void audio_enable(int enabled);
void audio_push_pcm(unsigned source, unsigned channel, const void *data,
                    unsigned frames, unsigned mono, unsigned volume_l,
                    unsigned volume_r, u32 timestamp_us);
void audio_send_available(struct UsbEndpoint *ep, int max_packets);
void audio_shutdown(struct UsbEndpoint *ep);
u32 audio_waiting_packets(void);
#endif
