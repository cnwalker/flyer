#include "telem.h"
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <limits>

TEST_CASE("All nodes are created with an incrementing id", "[create_node]") {
  TelemetrySender test_ts = TelemetrySender();

  REQUIRE(test_ts.createNode("example_node_one"));
  NodeDescriptor *ex_node_one = test_ts.getNodeDescriptor("example_node_one");
  REQUIRE(ex_node_one != NULL);
  REQUIRE(ex_node_one->node_id == 1);

  REQUIRE(test_ts.createNode("example_node_two"));
  NodeDescriptor *ex_node_two = test_ts.getNodeDescriptor("example_node_two");
  REQUIRE(ex_node_two != NULL);
  REQUIRE(ex_node_two->node_id == 2);
};

TEST_CASE("No two nodes can be created with the same name",
          "[create_node_dupes]") {
  TelemetrySender test_ts = TelemetrySender();
  REQUIRE(test_ts.createNode("example_one"));
  REQUIRE(test_ts.createNode("example_one") == false);
};

TEST_CASE("Attempting to fetch an invalid node results in an error",
          "[get_node_descriptor]") {
  TelemetrySender test_ts = TelemetrySender();
  REQUIRE(test_ts.getNodeDescriptor("non_existing_node") == NULL);
};

TEST_CASE("Creating a channel with a valid node works", "[create_channel]") {
  TelemetrySender test_ts = TelemetrySender();
  REQUIRE(test_ts.createNode("example_node_one"));
  NodeDescriptor *nd = test_ts.getNodeDescriptor("example_node_one");
  REQUIRE(test_ts.createChannel("example_channel_one", nd->node_id));
  REQUIRE(test_ts.createChannel("example_channel_with_invalid_node", 20) ==
          false);
};

TEST_CASE("Trying to create more than the maximum number of nodes fails",
          "[create_node_overflow]") {
  TelemetrySender *test_ts = new TelemetrySender();
  for (int i = 0; i < 65534; i++) {
    std::string node_name = "example_node_" + std::to_string(i);
    test_ts->createNode(node_name);
  }
  REQUIRE(test_ts->createNode("last_valid_node"));

  REQUIRE(test_ts->createNode("overflow_node") == false);
};

TEST_CASE("Integers are correctly converted to bytes", "[int_to_bytes]") {
  std::vector<std::byte> result = int_to_bytes(43);
  std::vector<std::byte> expected{
      std::byte{0x2B},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
  };
  REQUIRE(result == expected);
};

TEST_CASE("Telemetry values are properly serialized",
          "[telemetry_value_serialize]") {
  TelemetryValue *telem_val = new TelemetryValue(43, 2.5);
  std::vector<std::byte> result = telem_val->serialize();

  std::vector<std::byte> expected{
      /**
       * Channel id
       */
      std::byte{0x2B}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},

      /**
       * Value
       */
      std::byte{0x40}, std::byte{0x20}, std::byte{0x00}, std::byte{0x00}};

  REQUIRE(result == expected);
};

TEST_CASE("Test serializing multiple telemetry values",
          "[multiple_telemetry_value_serialize]") {
  TelemetryValue *telem_val_one = new TelemetryValue(41, 2.5);
  TelemetryValue *telem_val_two = new TelemetryValue(42, 19.5);

  std::vector<TelemetryValue *> telem_values{
      telem_val_one,
      telem_val_two,
  };

  TelemetrySender test_ts = TelemetrySender();

  std::vector<std::byte> result = test_ts.serializeTelemetry(telem_values);

  std::vector<std::byte> expected{
      /**
       * Number of telemetry values (2)
       */
      std::byte{0x02},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},

      /**
       * Telem Val One
       * Channel ID
       */
      std::byte{0x29},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      /**
       * Value
       */
      std::byte{0x40},
      std::byte{0x20},
      std::byte{0x00},
      std::byte{0x00},

      /**
       * Telem Val Two
       * Channel ID
       */
      std::byte{0x2A},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      /**
       * Value
       */
      std::byte{0x41},
      std::byte{0x9c},
      std::byte{0x00},
      std::byte{0x00},
  };

  REQUIRE(result == expected);
};