#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * 9.81 m/s^2
 */
double ACCELERATION_DUE_TO_EARTH_GRAVITY = -9.81;
double one_billion = 1000000000;

Vec3::Vec3(double input_x, double input_y, double input_z) {
  x = input_x;
  y = input_y;
  z = input_z;
};

void Vec3::print(std::string label = "") {
  if (!label.empty()) {
    std::cout << label << ": ";
  }
  std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
};

Box::Box(double input_x_length, double input_y_length,
                     double input_z_length, double input_mass,
                     Vec3 &input_position, Vec3 &input_velocity) {
  x_length = input_x_length;
  y_length = input_y_length;
  z_length = input_z_length;
  mass = input_mass;
  cm_pos = new Vec3(input_position.x, input_position.y, input_position.z);
  velocity = new Vec3(input_velocity.x, input_velocity.y, input_velocity.z);
};

void Box::set_id(unsigned int input_id) { id = input_id; };

void Box::print() {
  std::cout << "Box id " << id << std::endl;
  std::cout << "Position: "
            << "(" << cm_pos->x << ", " << cm_pos->y << ", " << cm_pos->z << ")"
            << std::endl;
  std::cout << "Velocity: "
            << "(" << velocity->x << ", " << velocity->y << ", " << velocity->z
            << ")" << std::endl;
};

/**
 * Creates a box from YAML
 */
Box create_box_from_yaml(YAML::Node config) {
  double mass = config["mass"].as<double>();

  double x_length = config["x_length"].as<double>();
  double y_length = config["y_length"].as<double>();
  double z_length = config["z_length"].as<double>();

  YAML::Node pos_node = config["position"].as<YAML::Node>();
  double pos_x = pos_node["x"].as<double>();
  double pos_y = pos_node["y"].as<double>();
  double pos_z = pos_node["z"].as<double>();

  Vec3 position = Vec3(pos_x, pos_y, pos_z);

  YAML::Node vel_node = config["velocity"].as<YAML::Node>();
  double vel_x = vel_node["x"].as<double>();
  double vel_y = vel_node["y"].as<double>();
  double vel_z = vel_node["z"].as<double>();

  Vec3 velocity = Vec3(vel_x, vel_y, vel_z);

  return Box(x_length, y_length, z_length, mass, position, velocity);
};

/**
 * Simulation constructor.
 */
Simulation::Simulation(double step_interval, Clock *input_clock) {
  step_interval_s = step_interval;
  num_steps = 0;
  clock = input_clock;
};

void Simulation::parse_boxes(YAML::Node config,
                                  std::string box_key) {
  if (!config[box_key]) {
    return;
  }
  YAML::Node rect_confs = config[box_key];

  int boxes_added = 0;
  for (YAML::const_iterator it = rect_confs.begin(); it != rect_confs.end();
       it++) {
    Box b = create_box_from_yaml(it->second.as<YAML::Node>());
    b.set_id(boxes_added);
    add_box(b);
    boxes_added++;
  }

  std::cout << "Added " << boxes_added << " boxes" << std::endl;
}

/**
 * Initializes a simulation based on config.
 */
void Simulation::initialize_from_config(const std::string config_path) {
  std::cout << "Reading config from: " << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  parse_boxes(config, "boxes");
};

/**
 * Runs the simulation.
 */
void Simulation::run() {
  long long int last_step_time = 0;
  long long int current_time = 0;
  long long int step_interval_ns = one_billion * step_interval_s;
  while (true) {
    current_time = clock->get_current_time_ns();
    if (current_time - last_step_time >= step_interval_ns) {
      run_step();
      last_step_time = current_time;
    }
  };
};

/**
 * Adds a box to the sim.
 */
void Simulation::add_box(Box b) { boxes.push_back(b); };

/**
 * Prints all  in the sim
 */
void Simulation::print_elements() {
  /**
   * Next print the boxes
   */
  for (int i = 0; i < boxes.size(); i++) {
    Box b = boxes.at(i);
    b.print();
    std::cout << std::endl;
  }
}

/**
 * Computes the force on a particle.
 */
Vec3 compute_force(Box &b) {
  /**
   * Currently we only support the force of gravity.
   */
  return Vec3(0, b.mass * ACCELERATION_DUE_TO_EARTH_GRAVITY, 0);
};

/**
 * Runs a single step of the simulation
 */
void Simulation::run_step() {
  for (size_t i = 0; i < boxes.size(); i++) {
    Box b = boxes.at(i);
    Vec3 force = compute_force(b);
    // F = ma, a = F/m
    Vec3 accel = Vec3(force.x / b.mass, force.y / b.mass, force.z / b.mass);
    // v = a(dt)
    b.velocity->x += accel.x * step_interval_s;
    b.velocity->y += accel.y * step_interval_s;
    b.velocity->z += accel.z * step_interval_s;
    // pos = v(dt)
    b.cm_pos->x += b.velocity->x * step_interval_s;
    b.cm_pos->y += b.velocity->y * step_interval_s;
    b.cm_pos->z += b.velocity->z * step_interval_s;
  }

  std::cout << "Step " << num_steps << " (current time "
            << clock->get_current_time_ns() << ")"
            << ":" << std::endl;
  print_elements();
  std::cout << std::endl;
  num_steps++;
};