/* Force-included into every BL616 translation unit (CMakeLists.txt).
 *
 * TinyUSB's endpoint buffers are read and written by the USB controller's DMA
 * (VDMA). Placing them in the SDK's non-cacheable RAM section keeps the CPU's
 * data cache out of the picture, as the SDK's own USB buffers are placed
 * (USB_NOCACHE_RAM_SECTION). The section is NOLOAD; TinyUSB clears what it
 * keeps there before use. */
#ifndef DS5_BL616_TUSB_MEM_H
#define DS5_BL616_TUSB_MEM_H

#define CFG_TUSB_MEM_SECTION __attribute__((section(".noncacheable")))

#endif
