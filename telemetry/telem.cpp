#include "telem.h"
#include <limits>
#include <string>
#include <vector>

NodeDescriptor::NodeDescriptor(unsigned short input_node_id,
                               std::string input_node_name) {
  node_id = input_node_id;
  node_name = input_node_name;
};

bool TelemetrySender::createNode(std::string input_node_name) {
  if (last_node_id == std::numeric_limits<unsigned short>::max()) {
    /**
     * If we've exhausted all last_node_ids, return false
     **/
    return false;
  }

  last_node_id++;
  NodeDescriptor nd = NodeDescriptor(last_node_id, input_node_name);
  node_descriptors.push_back(nd);
  return true;
}

bool TelemetrySender::createChannel(std::string input_channel_name) {
  return false;
}

bool TelemetrySender::finalize() {
  /**
   * Make sure that:
   *  1. All telemetry channels have a unique id
   *  2. Every telemetry channel has a descriptor
   *  3. All nodes are unique, and have a name
   *  4. All channels are unique and have a name
   *  5.
   */
  return false;
}