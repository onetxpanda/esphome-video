#ifdef USE_ESP32
#include "frame.h"

#include <esp_heap_caps.h>

namespace esphome {
namespace video {

size_t dma_alignment() {
  // Buffers written by DMA must not share a cache line with anything else, on
  // either cache level. 128 covers the ESP32-P4's L2 line; 64 elsewhere.
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  return 128;
#else
  return 64;
#endif
}

BufferAllocator default_allocator() {
  BufferAllocator a;
  a.alloc = [](size_t size, size_t align) -> void * {
    return heap_caps_aligned_alloc(align < sizeof(void *) ? sizeof(void *) : align, size, MALLOC_CAP_DEFAULT);
  };
  a.free = [](void *p) { heap_caps_free(p); };
  return a;
}

BufferAllocator dma_allocator() {
  BufferAllocator a;
  a.alloc = [](size_t size, size_t align) -> void * {
    if (align < dma_alignment())
      align = dma_alignment();
    void *p = heap_caps_aligned_calloc(align, 1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (p == nullptr)
      p = heap_caps_aligned_calloc(align, 1, size, MALLOC_CAP_SPIRAM);
    if (p == nullptr)
      p = heap_caps_aligned_calloc(align, 1, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    return p;
  };
  a.free = [](void *p) { heap_caps_free(p); };
  return a;
}

}  // namespace video
}  // namespace esphome
#endif
