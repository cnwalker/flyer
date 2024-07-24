#include "telem.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Make sure nodes are created with an incrementing id",
          "[create_node]") {
  TelemetrySender test_ts = TelemetrySender();
  REQUIRE(test_ts.createNode("example_node_one"));
};
