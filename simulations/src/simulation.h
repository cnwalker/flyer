#include "clock.h"
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class Vec3 {
public:
  double x;
  double y;
  double z;

  Vec3(double input_x, double input_y, double input_z);

  void print(std::string label);
};

/**
 * Box, should support a simple drone body for now
 */
struct Box {
  /**
   * Box ID
   */
  unsigned int id;

  // Position of the center of mass of the rigid body
  Vec3 *cm_pos;
  Vec3 *velocity;
  double mass;

  double x_length;
  double y_length;
  double z_length;

  Box(double input_x_length, double input_y_length, double input_z_length,
      double input_mass, Vec3 &input_position, Vec3 &input_velocity);

  void set_id(unsigned int input_id);

  void print();
};

/**
 * Computes the force on a particle.
 */
Vec3 compute_force(Box &b);

class Simulation {
public:
  /**
   * Constructor for the simulation
   */
  Simulation(double step_interval, Clock *input_clock);

  /**
   * Initializes a simulation based on config.
   */
  void initialize_from_config(const std::string config_path);

  /**
   * Function for parsing particles from the sim config YAML
   */
  void parse_particles(YAML::Node config, std::string particle_key);

  /**
   * Function for parsing boxes from the sim config YAML
   */
  void parse_boxes(YAML::Node config, std::string box_key);

  /**
   * Runs the simulation.
   */
  void run();

  /**
   * Adds a box to the sim.
   */
  void add_box(Box b);

  /**
   * Prints all elements in the sim
   */
  void print_elements();

private:
  /**
   * Rate for the simulation to run.
   */
  double step_interval_s;

  /**
   * Number of steps in the simulation.
   */
  long long int num_steps;

  /**
   * Clock returns the current time.
   */
  Clock *clock;

  /**
   * List of boxes to simulate
   */
  std::vector<Box> boxes;

  /**
   * Strictly increases number of added boxes
   */
  unsigned int num_boxes_added;

  /**
   * Runs a single step of the simulation
   */
  void run_step();
};