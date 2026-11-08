#pragma once

// Frames passed between esphome-video components, and the source/sink
// interfaces that connect them. No ESPHome or ESP-IDF dependency (the buffer
// allocator is injected), so this is unit-tested on a host.
//
// Two kinds of frame:
//   RadiometricFrame  temperatures in degrees C, one float per pixel
//   ImageFrame        pixels in an ImageFormat
//
// Buffers come from a FramePool and are reference counted (FrameRef). A sink
// that needs a frame after its on_*_frame() call returns (e.g. a hardware
// encoder working on another task) keeps the FrameRef; the pool reuses a
// buffer only once every reference is gone. Producers therefore never copy a
// frame for a consumer, and a slow consumer can never see its pixels change.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace esphome {
namespace video {

enum class ImageFormat : uint8_t {
  RGB565_LE,  // 16-bit 5-6-5, low byte first (the ESP32-P4 encoder's and PPA's native order)
  RGB565_BE,  // 16-bit 5-6-5, high byte first (esp32-camera JPEG encoder)
  RGB888,     // 24-bit, R G B byte order
  GRAY8,
};

inline size_t bytes_per_pixel(ImageFormat f) {
  switch (f) {
    case ImageFormat::RGB888:
      return 3;
    case ImageFormat::GRAY8:
      return 1;
    default:
      return 2;
  }
}

// Allocator for pool buffers: returns memory aligned to at least `align`
// bytes, or nullptr. On the device this is DMA/cache-line aligned PSRAM.
struct BufferAllocator {
  std::function<void *(size_t size, size_t align)> alloc;
  std::function<void(void *)> free;
};
BufferAllocator default_allocator();  // aligned heap memory (host and fallback)
#ifdef USE_ESP32
// Cache-line aligned, DMA-capable memory: PSRAM when present, else internal.
BufferAllocator dma_allocator();
size_t dma_alignment();  // L1/L2 cache line size, for DMA engines (PPA, JPEG, H.264)
#endif

// Memory for large working buffers (whole-frame scratch, tables), zeroed:
// PSRAM on the device when there is any, else internal RAM. nullptr when out
// of memory.
void *large_alloc(size_t bytes);
void large_free(void *p);

// A fixed-size array of plain data in large_alloc() memory. Unlike
// std::vector, running out of memory is reported (ensure() returns false)
// instead of aborting the firmware.
template<typename T> class LargeBuffer {
  static_assert(std::is_trivially_copyable<T>::value && std::is_trivially_default_constructible<T>::value,
                "LargeBuffer holds plain data only");

 public:
  LargeBuffer() = default;
  ~LargeBuffer() { this->reset(); }
  LargeBuffer(const LargeBuffer &) = delete;
  LargeBuffer &operator=(const LargeBuffer &) = delete;

  // Makes the buffer hold `count` elements: unchanged if it already does,
  // else newly allocated and zeroed. False (and empty) when out of memory.
  bool ensure(size_t count) {
    if (count == this->count_ && (this->data_ != nullptr || count == 0))
      return true;
    this->reset();
    if (count == 0)
      return true;
    this->data_ = static_cast<T *>(large_alloc(count * sizeof(T)));
    if (this->data_ == nullptr)
      return false;
    this->count_ = count;
    return true;
  }
  void reset() {
    if (this->data_ != nullptr)
      large_free(this->data_);
    this->data_ = nullptr;
    this->count_ = 0;
  }
  T *data() { return this->data_; }
  const T *data() const { return this->data_; }
  size_t size() const { return this->count_; }
  T &operator[](size_t i) { return this->data_[i]; }
  const T &operator[](size_t i) const { return this->data_[i]; }

 private:
  T *data_{nullptr};
  size_t count_{0};
};

class FramePool;

// One pooled buffer. Owned by its pool; handed out as FrameRef.
struct FrameBuffer {
  uint8_t *data{nullptr};
  size_t capacity{0};
  FramePool *pool{nullptr};
  std::atomic<int> refs{0};
};

// A counted reference to a FrameBuffer. Copying adds a reference; the last
// reference to go returns the buffer to its pool.
class FrameRef {
 public:
  FrameRef() = default;
  explicit FrameRef(FrameBuffer *b);
  FrameRef(const FrameRef &o);
  FrameRef(FrameRef &&o) noexcept;
  FrameRef &operator=(const FrameRef &o);
  FrameRef &operator=(FrameRef &&o) noexcept;
  ~FrameRef();
  uint8_t *data() const { return this->b_ ? this->b_->data : nullptr; }
  size_t capacity() const { return this->b_ ? this->b_->capacity : 0; }
  explicit operator bool() const { return this->b_ != nullptr; }
  void reset();

 private:
  FrameBuffer *b_{nullptr};
};

// A fixed set of equally sized buffers (size rounded up to a multiple of
// `align`, as DMA engines such as the P4's PPA require). acquire() returns a free buffer or an
// empty FrameRef when all are in use (the producer then drops that frame).
class FramePool {
 public:
  FramePool(size_t count, size_t size, size_t align = 64, BufferAllocator alloc = default_allocator());
  ~FramePool();
  FramePool(const FramePool &) = delete;
  FramePool &operator=(const FramePool &) = delete;

  bool ok() const { return this->ok_; }
  size_t buffer_size() const { return this->size_; }
  FrameRef acquire();
  size_t free_count() const;

 private:
  friend class FrameRef;
  void release_(FrameBuffer *b);

  BufferAllocator alloc_;
  size_t size_;
  bool ok_{false};
  std::vector<std::unique_ptr<FrameBuffer>> buffers_;
  mutable std::mutex mutex_;
  std::vector<FrameBuffer *> free_;
};

// A point of interest carried with a frame (e.g. the hottest pixel), with a
// value in the frame's unit. x, y are pixel indices in the frame that carries
// it; stages that change geometry move it with the pixels. Overlays draw
// annotations; nothing else interprets them.
enum class AnnotationKind : uint8_t { MIN = 0, MAX = 1, CENTER = 2, OTHER = 3 };
struct Annotation {
  float x{0}, y{0};
  float value{0};
  AnnotationKind kind{AnnotationKind::OTHER};
};
struct Annotations {
  static constexpr uint8_t CAPACITY = 4;
  Annotation items[CAPACITY];
  uint8_t count{0};
  bool add(const Annotation &a) {
    if (this->count >= CAPACITY)
      return false;
    this->items[this->count++] = a;
    return true;
  }
};

struct RadiometricFrame {
  FrameRef buffer;  // width * height floats, row-major
  uint16_t width{0};
  uint16_t height{0};
  uint32_t timestamp_ms{0};  // capture time of the newest data in the frame
  uint32_t sequence{0};
  Annotations annotations;
  const float *temps() const { return reinterpret_cast<const float *>(this->buffer.data()); }
  float *temps() { return reinterpret_cast<float *>(this->buffer.data()); }
};

struct ImageFrame {
  FrameRef buffer;
  uint16_t width{0};
  uint16_t height{0};
  uint32_t stride{0};  // bytes per row
  ImageFormat format{ImageFormat::RGB565_LE};
  uint32_t timestamp_ms{0};
  uint32_t sequence{0};
  Annotations annotations;  // image pixel coordinates
  size_t length() const { return static_cast<size_t>(this->stride) * this->height; }
};

// Sinks are called on the producer's task. A sink must not block for long; to
// work on a frame later it keeps (copies) the frame, which holds the buffer.
class RadiometricSink {
 public:
  virtual ~RadiometricSink() = default;
  virtual void on_radiometric_frame(const RadiometricFrame &frame) = 0;
  // False while nobody downstream needs frames, so producers may idle.
  virtual bool wants_frames() const { return true; }
};

class RadiometricSource {
 public:
  virtual ~RadiometricSource() = default;
  virtual void add_radiometric_sink(RadiometricSink *sink) = 0;
};

class ImageSink {
 public:
  virtual ~ImageSink() = default;
  virtual void on_image_frame(const ImageFrame &frame) = 0;
  virtual bool wants_frames() const { return true; }
  // True for a sink that draws into the frames it receives (an overlay). It
  // must then be its source's only sink, so no other sink sees a frame change.
  virtual bool modifies_frames() const { return false; }
  // The source now shows different content (a switch chose another camera):
  // an encoder starts afresh with a keyframe.
  virtual void on_source_change() {}
};

class ImageSource {
 public:
  virtual ~ImageSource() = default;
  virtual void add_image_sink(ImageSink *sink) = 0;
};

// Fan-out helper for sources.
template<typename Sink> class SinkList {
 public:
  void add(Sink *s) { this->sinks_.push_back(s); }
  bool any_wants() const {
    for (auto *s : this->sinks_)
      if (s->wants_frames())
        return true;
    return false;
  }
  template<typename Frame, typename Fn> void deliver(const Frame &f, Fn call) const {
    for (auto *s : this->sinks_)
      if (s->wants_frames())
        call(s, f);
  }
  template<typename Fn> void each(Fn fn) const {
    for (auto *s : this->sinks_)
      fn(s);
  }
  bool empty() const { return this->sinks_.empty(); }
  size_t size() const { return this->sinks_.size(); }
  // False if a sink that modifies frames shares its source with another sink.
  bool modifying_sink_is_exclusive() const {
    if (this->sinks_.size() <= 1)
      return true;
    for (auto *s : this->sinks_)
      if (s->modifies_frames())
        return false;
    return true;
  }

 private:
  std::vector<Sink *> sinks_;
};

}  // namespace video
}  // namespace esphome
