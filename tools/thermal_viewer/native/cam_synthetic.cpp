// A source with no camera: a room-temperature scene with two warm objects
// moving through it and sensor noise, 160 x 120 at 25 frames/s. For trying the
// processing and the viewer itself.

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include "tv_camera.h"

namespace tv {
namespace {

class Synthetic : public Camera {
 public:
  void run(Host &host, const std::string &) override {
    static constexpr int W = 160, H = 120;
    std::vector<float> t(static_cast<size_t>(W) * H);
    std::mt19937 rng(1);
    std::normal_distribution<float> noise(0.0f, 0.15f);
    host.status("Synthetic scene, 160 x 120");
    const auto t0 = std::chrono::steady_clock::now();
    while (!host.stopping()) {
      const float s = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
      const float ax = W * (0.5f + 0.35f * std::cos(s * 0.7f)), ay = H * (0.5f + 0.3f * std::sin(s * 0.9f));
      const float bx = W * (0.5f + 0.4f * std::sin(s * 0.3f)), by = H * 0.75f;
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          const float da = ((x - ax) * (x - ax) + (y - ay) * (y - ay)) / (2.0f * 9.0f * 9.0f);
          const float db = ((x - bx) * (x - bx) + (y - by) * (y - by)) / (2.0f * 4.0f * 4.0f);
          float v = 21.0f + 2.0f * x / W + 1.0f * y / H;  // a wall, warmer to one side
          v += 13.0f * std::exp(-da) + 30.0f * std::exp(-db);
          if (x > 20 && x < 50 && y > 15 && y < 40)
            v -= 4.0f;  // a cool panel with sharp edges
          t[static_cast<size_t>(y) * W + x] = v + noise(rng);
        }
      }
      host.frame(t.data(), W, H, nullptr, 0.95f, 22.0f);
      if (!host.wait(40))
        break;
    }
    host.status("Stopped");
  }
};

}  // namespace

CameraType synthetic_type() {
  return {"synthetic", "Synthetic scene (no camera)", "", "", {}, 4, 10.0f, []() -> Camera * { return new Synthetic(); }};
}

}  // namespace tv
