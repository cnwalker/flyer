#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
  int num_expected_args = 3;
  if (argc != num_expected_args) {
    std::cout << "Expected " << num_expected_args << " args, received " << argc
              << std::endl;
    std::cout << "Usage: ./simulator <rate> <path_to_config>" << std::endl;
    return 1;
  }

  double step_rate = atof(argv[1]);
  std::string config_path = std::string(argv[2]);
  SystemClock *clock = new SystemClock();

  Simulation sim = Simulation(step_rate, clock);

  sim.initialize_from_config(config_path);
  sim.run();

  return 0;
}