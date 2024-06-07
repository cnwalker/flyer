# Simulations
You can configure the sim repo for cmake with:

```
cmake .
```

Build the engine with:

```
cmake --build .
```

Run the simulator with `./simulator <rate> <path_to_config>` where rate is the cycle time in seconds. So to run the simulator at 10Hz you would run `./simulator 0.02 /path/to/config.yaml`

Run the tests with just `./tests`