# Format sims
find simulations/src | grep ".*[cc|cpp|h|hpp]$" | xargs clang-format -i
find telemetry/src | grep ".*[cc|cpp|h|hpp]$" | xargs clang-format -i