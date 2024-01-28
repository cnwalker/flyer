#include "clock.h"
#include <iostream>
#include <string>
#include <vector>

class Vec3 {
public:
  double x;
  double y;
  double z;

  Vec3(double input_x, double input_y, double input_z);

  void print(std::string label);
};

class Particle {
public:
  double mass;
  Vec3 *position;
  Vec3 *velocity;

  Particle(double input_mass, Vec3 &input_position, Vec3 &input_velocity);

  void print();
};

/**
 * Computes the force on a particle.
 */
Vec3 compute_force(Particle &particle);

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
   * Runs the simulation.
   */
  void run();

  /**
   * Adds a particle to the sim.
   */
  void add_particle(Particle p);

  /**
   * Prints all particles in the sim
   */
  void print_particles();

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
   * List of particles to simulate.
   */
  std::vector<Particle> particles;

  /**
   * Runs a single step of the simulation
   */
  void run_step();
};