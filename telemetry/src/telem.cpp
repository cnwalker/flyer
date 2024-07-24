#include "telem.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

/**
 * NodeDescriptor constructor
 */
NodeDescriptor::NodeDescriptor(unsigned short input_node_id,
                               std::string input_node_name) {
  node_id = input_node_id;
  node_name = input_node_name;
};

/**
 * Creates a new node.
 */
bool TelemetrySender::createNode(std::string input_node_name) {
  unsigned short max_node_id = std::numeric_limits<unsigned short>::max();
  if (last_node_id == max_node_id) {
    /**
     * If we've exhausted all last_node_ids, return false.
     */
    std::cerr << "No more possible node ids. Max node id is: " << max_node_id
              << std::endl;
    return false;
  }

  /**
   * Make sure this node isn't in any other nodes.
   */
  if (node_descriptors_by_name.find(input_node_name) !=
      node_descriptors_by_name.end()) {
    return false;
  }

  last_node_id++;
  NodeDescriptor *nd = new NodeDescriptor(last_node_id, input_node_name);
  node_descriptors_by_name[nd->node_name] = nd;
  node_descriptors_by_id[nd->node_id] = nd;
  return true;
}

/**
 * Gets a node descriptor.
 */
NodeDescriptor *TelemetrySender::getNodeDescriptor(std::string node_name) {
  auto node_iter = node_descriptors_by_name.find(node_name);
  if (node_iter == node_descriptors_by_name.end()) {
    return NULL;
  }
  return node_iter->second;
}

/**
 * TelemetryChannelDescriptor constructor.
 */
TelemetryChannelDescriptor::TelemetryChannelDescriptor(
    std::string input_channel_name, unsigned int input_channel_id,
    unsigned short input_node_id) {
  channel_name = input_channel_name;
  channel_id = input_channel_id;
  node_id = input_node_id;
};

/**
 * Creates a telemetry channel.
 */
bool TelemetrySender::createChannel(std::string input_channel_name,
                                    unsigned short input_node_id) {
  /**
   * First check that the node id exists
   */
  NodeDescriptor *node_for_channel = NULL;

  auto cur_node_iter = node_descriptors_by_id.find(input_node_id);
  if (cur_node_iter == node_descriptors_by_id.end()) {
    std::cerr << "No node found with id " << input_node_id << std::endl;
    return false;
  }
  node_for_channel = cur_node_iter->second;

  /**
   * Ensure that no channels on the same node share the same name;
   * And get the largest global channel id.
   */
  unsigned int largest_channel_id = 0;
  for (int i = 0; i < telem_channel_descriptors.size(); i++) {
    TelemetryChannelDescriptor *chan_desc = telem_channel_descriptors.at(i);
    if (largest_channel_id < chan_desc->channel_id) {
      largest_channel_id = chan_desc->channel_id;
    }

    if (chan_desc->node_id != input_node_id) {
      continue;
    }

    if (chan_desc->channel_name == input_channel_name) {
      std::cerr << "Channel name " << input_channel_name
                << " already exists for " << node_for_channel->node_name
                << " (id " << node_for_channel->node_id << ")." << std::endl;
      return false;
    }

    if (chan_desc->channel_id > largest_channel_id) {
      largest_channel_id = chan_desc->channel_id;
    }
  }

  TelemetryChannelDescriptor *new_chan_desc = new TelemetryChannelDescriptor(
      input_channel_name, largest_channel_id + 1, input_node_id);

  telem_channel_descriptors.push_back(new_chan_desc);

  return true;
}

/**
 * Convert integer to array of bytes
 */
std::vector<std::byte> int_to_bytes(int input_integer) {
  std::vector<std::byte> serialized_bytes;
  int lower_bit_mask = 0x000000FF;

  for (int i = 0; i < 4; i++) {
    std::byte cur_byte = std::byte((lower_bit_mask << (i * 8)) & input_integer);
    serialized_bytes.push_back(cur_byte);
  }
  return serialized_bytes;
}

/**
 * Serialize bytes
 */
std::vector<std::byte> TelemetryValue::serialize() {
  /**
   * First serialize the channel id
   */
  std::vector<std::byte> serialized_value = int_to_bytes(channel_id);

  /**
   * Then serialize the value
   */
  char *float_as_bytes = reinterpret_cast<char *>(&value);
  for (int i = 3; i >= 0; i--) {
    serialized_value.push_back(std::byte(float_as_bytes[i]));
  }

  return serialized_value;
};

/**
 * Serialize the telemetry values into a compact format.
 */
std::vector<std::byte> TelemetrySender::serializeTelemetry(
    std::vector<TelemetryValue *> telem_values) {

  int num_values = telem_values.size();
  std::vector<std::byte> serialized_values = int_to_bytes(num_values);

  for (int i = 0; i < telem_values.size(); i++) {
    std::vector<std::byte> serialized_val = telem_values.at(i)->serialize();
    serialized_values.insert(serialized_values.end(), serialized_val.begin(),
                             serialized_val.end());
  }

  return serialized_values;
}

/**
 * Deserialize telemetry values.
 */
std::vector<TelemetryValue *>
deserializeTelemetry(std::vector<TelemetryValue *> input_telem_values) {
  std::vector<TelemetryValue *> telem_values;
  return telem_values;
}

/**
 * Serialize and send the telemetry values over the network.
 */
bool TelemetrySender::sendTelemetry(
    std::vector<TelemetryValue *> telem_values) {
  return true;
}

/**
 * TelemetryValue constructor.
 */
TelemetryValue::TelemetryValue(unsigned int input_channel_id,
                               float input_value) {
  channel_id = input_channel_id;
  value = input_value;
};
