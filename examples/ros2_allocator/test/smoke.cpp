// smoke.cpp
// Standalone check of the generated header, no ROS 2 required. Verifies that
// allocate() reproduces the Julia reference for a sample wrench.
//
//   g++ -std=c++17 -I ../include smoke.cpp -o smoke && ./smoke
//
// Reference produced by ThrusterHelper.jl for the default bluerov_vehicle():
//   allocate([30, 8, -5, 0, 0, 4]) (minimum_norm, saturate=true) =
//   [1.0, 0.274131, -0.660232, -0.6139, -0.075079, -0.075079, -0.075079, -0.075079]
// Regenerate the header for a different vehicle and this reference no longer
// applies; update the expected values from your own Julia allocate() output.

#include "thruster_allocation.hpp"
#include <array>
#include <cmath>
#include <cstdio>

int main() {
  const std::array<float, 6> tau = {30.0f, 8.0f, -5.0f, 0.0f, 0.0f, 4.0f};
  const std::array<float, 8> expected = {
      1.0f, 0.274131f, -0.660232f, -0.6139f,
      -0.075079f, -0.075079f, -0.075079f, -0.075079f};

  const auto f = thruster_allocation::allocate(tau);
  static_assert(thruster_allocation::kNumActuators == 8, "expected 8 actuators");

  int bad = 0;
  for (std::size_t i = 0; i < f.size(); ++i) {
    if (std::fabs(f[i] - expected[i]) > 1e-4f) {
      std::printf("MISMATCH [%zu]: got %.6f expected %.6f\n", i, f[i], expected[i]);
      ++bad;
    }
  }
  std::printf(bad == 0 ? "OK: allocate() matches Julia reference\n"
                       : "%d mismatch(es)\n", bad);
  return bad == 0 ? 0 : 1;
}
