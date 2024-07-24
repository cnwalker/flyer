#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

std::vector<std::byte> int_to_bytes(int input_integer);

struct NodeDescriptor {
  unsigned short node_id;
  std::string node_name;
  NodeDescriptor(unsigned short input_node_id, std::string input_node_name);
};

struct TelemetryChannelDescriptor {
  unsigned int channel_id;
  unsigned short node_id;
  std::string channel_name;
  TelemetryChannelDescriptor(std::string input_channel_name,
                             unsigned int input_channel_id,
                             unsigned short input_node_id);
};

struct TelemetryValue {
  unsigned int channel_id;
  float value;
  TelemetryValue(unsigned int input_channel_id, float input_value);
  std::vector<std::byte> serialize();
};

class TelemetrySender {
private:
  std::vector<TelemetryChannelDescriptor *> telem_channel_descriptors;
  std::unordered_map<std::string, NodeDescriptor *> node_descriptors_by_name;
  std::unordered_map<unsigned short, NodeDescriptor *> node_descriptors_by_id;
  unsigned short last_node_id;
  bool finalized;

public:
  bool createNode(std::string node_name);
  NodeDescriptor *getNodeDescriptor(std::string input_node_name);
  bool createChannel(std::string input_channel_name, unsigned short node_id);
  bool sendTelemetry(std::vector<TelemetryValue *> telem_values);
  std::vector<std::byte>
  serializeTelemetry(std::vector<TelemetryValue *> telem_values);
};