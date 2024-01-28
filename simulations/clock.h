#include <chrono>

/**
 * Returns the current time in nanoseconds
 */
class Clock {
public:
  virtual ~Clock(){};

  virtual unsigned long long int get_current_time_ns() { return 0; }
};

/**
 * System clock that returns system time.
 */
class SystemClock : virtual public Clock {
public:
  virtual unsigned long long int get_current_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
};