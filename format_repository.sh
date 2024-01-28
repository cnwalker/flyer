# Format sims
find simulations/ | grep ".*[cc|cpp|h|hpp]$" | xargs clang-format -i