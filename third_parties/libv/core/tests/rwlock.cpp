#include <stdexcept>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>

#include <libv/core/rwlock.hpp>

using namespace v::core;

namespace
{
  typedef std::chrono::system_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;

  TimePoint now()
  {
    return Clock::now();
  }

  const TimePoint start = now();

  unsigned elapsed_ms()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now() - start).count();
  }

  void sleep_ms(unsigned ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  RwLock rwlock;
  volatile int shared = 0;

  int read(unsigned ms)
  {
    RwLock::Reader r(rwlock);
    sleep_ms(ms);
    return shared;
  }

  int write(int x, unsigned ms)
  {
    RwLock::Writer w(rwlock);
    const int old = shared;
    sleep_ms(ms);
    shared += x;
    return old;
  }

  void fail(const std::string &s)
  {
    std::cerr << s << std::endl;
    exit(1);
  }

  void check(bool b, const std::string &s)
  {
    if (!b)
      fail(s);
  }

  /*
   * SCENARIO
   * 0    R1 and R2 start reading
   * 500  W1 tries to acquire the lock and gets blocked
   * 1000 W2, R1 and R2 try to acquire the lock, one of the writers gets it
   * 2000 First writer (either W1 or W2) done, now the 2nd writer gets the lock
   * 3000 2nd writer done, both readers acquire the lock
   * 4000 Both readers release the lock
   */

  void reader1()
  {
    check(read(1000) == 0, "R1_1");
    check(read(1000) == 11, "R1_2");
    check(elapsed_ms() >= 4000 && elapsed_ms() < 4300, "R1_3");
  }

  void reader2()
  {
    check(read(1000) == 0, "R2_1");
    check(read(1000) == 11, "R2_2");
    check(elapsed_ms() >= 4000 && elapsed_ms() < 4300, "R2_3");
  }

  void writer1()
  {
    sleep_ms(500);
    write(1, 1000);
  }

  void writer2()
  {
    sleep_ms(1000);
    write(10, 1000);
  }
}

int main()
{
  std::thread r1(reader1);
  std::thread r2(reader2);
  std::thread w1(writer1);
  std::thread w2(writer2);

  w2.join();
  w1.join();
  r2.join();
  r1.join();
}

