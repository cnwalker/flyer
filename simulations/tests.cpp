#include "simulation.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Computing force follows f = ma", "[compute_force]") {
  Vec3 position = Vec3(3.0, 4.0, 5.0);
  Vec3 velocity = Vec3(0, 5.0, 0.0);
  Particle p = Particle(2.0, position, velocity);
  Vec3 force = compute_force(p);
  Vec3 expected_force = Vec3(0, -19.62, 0);
  REQUIRE(force.x == expected_force.x);
  REQUIRE(force.y == expected_force.y);
  REQUIRE(force.z == expected_force.z);
};