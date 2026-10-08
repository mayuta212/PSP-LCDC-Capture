#ifndef LCDC60_DUAL32_PROTOCOL_H
#define LCDC60_DUAL32_PROTOCOL_H
#include <psptypes.h>

#define LCDC60_MAGIC 0x3036434Cu /* LC60 */
#define LCDC60_DUAL_TAG 0x324D5246u /* FRM2 */
#define LCDC60_DUAL_VERSION 2u
#define LCDC60_WIDTH 480u
#define LCDC60_STRIDE 512u
#define LCDC60_HEIGHT 272u
#define LCDC60_BYTES16 (LCDC60_STRIDE * LCDC60_HEIGHT * 2u)
#define LCDC60_BYTES32 (LCDC60_STRIDE * LCDC60_HEIGHT * 4u)
#define LCDC60_PART_SIZE(total) ((total) / 2u - 64u)

/* One short control packet followed by one short-terminated data packet
 * on EACH IN endpoint. Both data buffers are 64B aligned; no padded/cropped
 * data on the PSP. Every byte of the original DMAC image is sent exactly once.
 * Format matches sceDisplayGetFrameBuf: 0=565, 1=5551, 2=4444, 3=8888.
 */
struct Lcdc60DualHeader {
    u32 magic, tag, sequence, frame_bytes, part0_bytes;
    u8 format, endpoint; /* 1=EP81, 2=EP82 */
    u16 version;
} __attribute__((packed));
typedef char hdr_size_assert[(sizeof(struct Lcdc60DualHeader)==24) ? 1 : -1];
#endif
