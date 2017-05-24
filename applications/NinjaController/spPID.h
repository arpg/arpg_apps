#ifndef SPPID_H__
#define SPPID_H__

#include <vector>
#include <chrono>

typedef std::chrono::high_resolution_clock::time_point spTimestamp;

class spPID {
public:
  spPID();
  ~spPID();

  void SetGainP(double p);
  void SetGainI(double i);
  void SetGainD(double d);
  void SetPlantError(double error/*desired-current*/);
  double GetControlOutput();

private:
  double gain_p_;
  double gain_i_;
  double gain_d_;
  double integral_value;
  double prev_error_;
  spTimestamp prev_error_time_;
  double curr_error_;
  spTimestamp curr_error_time_;
  double TickTock_ms(spTimestamp tick_time,spTimestamp tock_time); 
};

#endif  // SPPID_H__
