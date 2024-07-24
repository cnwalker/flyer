#include <string>
#include <vector>

struct NodeDescriptor {
  unsigned short node_id;
  std::string node_name;
  NodeDescriptor(unsigned short input_node_id, std::string input_node_name);
};

struct TelemetryDescriptor {
  unsigned int channel_id;
  unsigned short node_id;
  std::string channel_name;
  TelemetryDescriptor(std::string input_channel_name);
};

struct TelemetryValue {
  unsigned int channel_id;
  double value;
  TelemetryValue(unsigned int input_channel_id, double input_value);
};

class TelemetrySender {
private:
  unsigned short last_node_id;

public:
  std::vector<TelemetryDescriptor> telem_descriptors;
  std::vector<NodeDescriptor> node_descriptors;
  bool createNode(std::string node_name);
  bool createChannel(std::string channel_name);
  bool finalize();
};