#pragma once

#include <mutex>

namespace esphome {
namespace h264_encoder {

// Hands frames from a source to the encoder through three slots, so that
// neither side ever waits for the other's work: the encoder owns one slot
// (the frame it encodes, and repeats), one slot holds the newest finished
// frame, and the source copies into the third. Only the bookkeeping below is
// under the lock, never a copy or an encode.
class SlotExchange {
 public:
  static constexpr int SLOTS = 3;
  static constexpr int NONE = -1;

  // Source: a slot to fill, neither the encoder's nor the ready one; NONE
  // while another fill is still going on (a second source mid-switch).
  int begin_write() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->writing_ != NONE)
      return NONE;
    for (int i = 0; i < SLOTS; i++) {
      if (i != this->front_ && i != this->ready_) {
        this->writing_ = i;
        return i;
      }
    }
    return NONE;  // unreachable with three slots
  }

  // Source: the slot is filled; it becomes the ready frame. Returns the
  // ready frame it replaced, which the encoder never took (it is dropped), or
  // NONE. A replaced slot stays the caller's until release_done(), so it can
  // let go of what the slot holds without another fill starting there.
  int finish_write(int slot) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    const int replaced = this->ready_;
    this->ready_ = slot;
    this->writing_ = replaced;
    return replaced;
  }

  // Source: done with the slot finish_write() returned.
  void release_done() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->writing_ = NONE;
  }

  // Source: the slot was not filled after all.
  void abandon_write() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->writing_ = NONE;
  }

  // Encoder: takes the ready frame, if any, as its own slot (its old slot
  // becomes free). Returns the encoder's slot, and whether it is new.
  int take(bool *fresh) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    *fresh = this->ready_ != NONE;
    if (*fresh) {
      this->front_ = this->ready_;
      this->ready_ = NONE;
    }
    return this->front_;
  }

 private:
  std::mutex mutex_;
  int front_{NONE};
  int ready_{NONE};
  int writing_{NONE};
};

}  // namespace h264_encoder
}  // namespace esphome
