#include "frame.h"

#include <cstdlib>

namespace esphome {
namespace video {

#ifndef USE_ESP32  // ESP32: frame_esp.cpp
void *large_alloc(size_t bytes) { return std::calloc(1, bytes); }
void large_free(void *p) { std::free(p); }
#endif

#ifndef USE_ESP32  // ESP32: frame_esp.cpp (newlib has no std::aligned_alloc)
BufferAllocator default_allocator() {
  BufferAllocator a;
  a.alloc = [](size_t size, size_t align) -> void * {
    if (align < sizeof(void *))
      align = sizeof(void *);
    const size_t rounded = (size + align - 1) / align * align;
    return std::aligned_alloc(align, rounded);
  };
  a.free = [](void *p) { std::free(p); };
  return a;
}
#endif

/* ---------------- FrameRef ---------------- */
FrameRef::FrameRef(FrameBuffer *b) : b_(b) {
  if (this->b_ != nullptr)
    this->b_->refs.fetch_add(1, std::memory_order_relaxed);
}
FrameRef::FrameRef(const FrameRef &o) : FrameRef(o.b_) {}
FrameRef::FrameRef(FrameRef &&o) noexcept : b_(o.b_) { o.b_ = nullptr; }
FrameRef &FrameRef::operator=(const FrameRef &o) {
  if (this != &o) {
    FrameBuffer *b = o.b_;
    if (b != nullptr)
      b->refs.fetch_add(1, std::memory_order_relaxed);
    this->reset();
    this->b_ = b;
  }
  return *this;
}
FrameRef &FrameRef::operator=(FrameRef &&o) noexcept {
  if (this != &o) {
    this->reset();
    this->b_ = o.b_;
    o.b_ = nullptr;
  }
  return *this;
}
FrameRef::~FrameRef() { this->reset(); }
void FrameRef::reset() {
  FrameBuffer *b = this->b_;
  this->b_ = nullptr;
  if (b != nullptr && b->refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
    b->pool->release_(b);
}

/* ---------------- FramePool ---------------- */
FramePool::FramePool(size_t count, size_t size, size_t align, BufferAllocator alloc)
    : alloc_(std::move(alloc)) {
  if (align == 0)
    align = 1;
  size = (size + align - 1) / align * align;
  this->size_ = size;
  this->ok_ = count > 0 && size > 0;
  for (size_t i = 0; i < count && this->ok_; i++) {
    auto b = std::make_unique<FrameBuffer>();
    b->data = static_cast<uint8_t *>(this->alloc_.alloc(size, align));
    b->capacity = size;
    b->pool = this;
    if (b->data == nullptr) {
      this->ok_ = false;
      break;
    }
    this->free_.push_back(b.get());
    this->buffers_.push_back(std::move(b));
  }
}

FramePool::~FramePool() {
  for (auto &b : this->buffers_)
    if (b->data != nullptr)
      this->alloc_.free(b->data);
}

FrameRef FramePool::acquire() {
  FrameBuffer *b = nullptr;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!this->ok_ || this->free_.empty())
      return FrameRef();
    b = this->free_.back();
    this->free_.pop_back();
  }
  return FrameRef(b);
}

size_t FramePool::free_count() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->free_.size();
}

void FramePool::release_(FrameBuffer *b) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->free_.push_back(b);
}

}  // namespace video
}  // namespace esphome
