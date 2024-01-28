#include "simulation.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>

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

Particle::Particle(double input_mass, Vec3 &input_position,
                   Vec3 &input_velocity) {
  mass = input_mass;
  position = new Vec3(input_position.x, input_position.y, input_position.z);
  velocity = new Vec3(input_velocity.x, input_velocity.y, input_velocity.z);
};

void Particle::print() {
  std::cout << "Position: "
            << "(" << position->x << ", " << position->y << ", " << position->z
            << ")" << std::endl;
  std::cout << "Velocity: "
            << "(" << velocity->x << ", " << velocity->y << ", " << velocity->z
            << ")" << std::endl;
};

/**
 * Creates a particle from YAML
 */
Particle create_particle_from_yaml(YAML::Node config) {
  double mass = config["mass"].as<double>();

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

  return Particle(mass, position, velocity);
};

/**
 * Simulation constructor.
 */
Simulation::Simulation(double step_interval, Clock *input_clock) {
  step_interval_s = step_interval;
  num_steps = 0;
  clock = input_clock;
};

/**
 * Initializes a simulation based on config.
 */
void Simulation::initialize_from_config(const std::string config_path) {
  std::cout << "Reading config from: " << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node particles = config["particles"].as<YAML::Node>();

  int num_particles = 0;
  for (YAML::const_iterator it = particles.begin(); it != particles.end();
       it++) {
    Particle p = create_particle_from_yaml(it->second.as<YAML::Node>());
    add_particle(p);
    num_particles++;
  }

  std::cout << "Added " << num_particles << " particles" << std::endl;
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
 * Adds a particle to the sim.
 */
void Simulation::add_particle(Particle p) { particles.push_back(p); };

/**
 * Prints all particles in the sim
 */
void Simulation::print_particles() {
  for (int i = 0; i < particles.size(); i++) {
    Particle p = particles.at(i);
    p.print();
    std::cout << std::endl;
  }
}

/**
 * Computes the force on a particle.
 */
Vec3 compute_force(Particle &particle) {
  /**
   * Currently we only support the force of gravity.
   */
  return Vec3(0, particle.mass * ACCELERATION_DUE_TO_EARTH_GRAVITY, 0);
};

/**
 * Runs a single step of the simulation
 */
void Simulation::run_step() {
  for (size_t i = 0; i < particles.size(); i++) {
    Particle p = particles.at(i);
    Vec3 force = compute_force(p);
    // F = ma, a = F/m
    Vec3 accel = Vec3(force.x / p.mass, force.y / p.mass, force.z / p.mass);
    // v = a(dt)
    p.velocity->x += accel.x * step_interval_s;
    p.velocity->y += accel.y * step_interval_s;
    p.velocity->z += accel.z * step_interval_s;
    // pos = v(dt)
    p.position->x += p.velocity->x * step_interval_s;
    p.position->y += p.velocity->y * step_interval_s;
    p.position->z += p.velocity->z * step_interval_s;
  }

  std::cout << "Step " << num_steps << " (current time "
            << clock->get_current_time_ns() << ")"
            << ":" << std::endl;
  print_particles();
  std::cout << std::endl;
  num_steps++;
};