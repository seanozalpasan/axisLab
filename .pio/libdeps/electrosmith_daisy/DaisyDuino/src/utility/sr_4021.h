#pragma once
#ifndef DEV_SR_4021_H
#define DEV_SR_4021_H
#include "Arduino.h"
#include "system.h"

namespace daisy {
/** @brief Device Driver for CD4021 shift register
 ** @author shensley
 ** @addtogroup shiftregister
 **
 ** CD4021B-Q1: CMOS 8-STAGE STATIC SHIFT REGISTER
 **
 ** Supply Voltage: 3V to 18V
 ** Clock Freq: 3MHz at 5V (less at 3v3) -> 8.5MHz at 15V
 ** Pin Descriptions:
 ** - Parallel Data[1-8] - 7, 6, 5, 4, 13, 14, 115, 1
 ** - Serial Data        - 11
 ** - Clock              - 10
 ** - P/!S               - 9
 ** - Q[6-8]             - 2, 12, 3
 **
 ** Driver has support for daisy chaining and running up to 2 same-sized
 ** chains in parallel from a single set of clk/latch pins to reduce
 ** pin/code overhead when using multiple devices.
 **
 ** When dealing with multiple parallel/daisy-chained devices the
 ** states of all inputs will be filled in the following order (example uses two
 *chained and two parallel):
 ** data[chain0,parallel0], data[chain1,parallel0], data[chain0,parallel1],
 *data[chain1,parallel1];
 **
 ** When combining multiple daisy chained and parallel devices the number of
 *devices chained should match
 ** for each parallel device chain.
 **
 ***/
template <size_t num_daisychained = 1, size_t num_parallel = 1>
class ShiftRegister4021 {
public:
  /** Configuration Structure for handling the pin setting of the device */
  struct Config {
    uint32_t clk;   /**< Clock pin to attach to pin 10 of device(s) */
    uint32_t latch; /**< Latch pin to attach to pin 9 of device(s) */
    uint32_t data[num_parallel]; /**< Data Pin(s) */
  };

  ShiftRegister4021() {}
  ~ShiftRegister4021() {}

  /** Initializes the Device(s) */
  void Init(const Config &cfg) {
    config_ = cfg;

    // Init GPIO
    clk_ = cfg.clk;
    pinMode(clk_, OUTPUT);

    latch_ = cfg.latch;
    pinMode(latch_, OUTPUT);

    for (size_t i = 0; i < num_parallel; i++) {
      data_[i] = cfg.data[i];
      pinMode(data_[i], INPUT);
    }

    // Init States
    for (size_t i = 0; i < kTotalStates; i++) {
      states_[i] = false;
    }
  }

  /** Reads the states of all pins on the connected device(s) */
  void Update() {
    digitalWrite(clk_, 0);
    digitalWrite(latch_, 1);

    delayMicroseconds(1);

    digitalWrite(latch_, 0);
    uint32_t idx;
    for (size_t i = 0; i < 8 * num_daisychained; i++) {
      digitalWrite(clk_, 0);

      delayMicroseconds(1);

      for (size_t j = 0; j < num_parallel; j++) {
        idx = (8 * num_daisychained - 1) - i;
        idx += (8 * num_daisychained * j);
        states_[idx] = digitalRead(data_[j]);
      }
      digitalWrite(clk_, 1);

      delayMicroseconds(1);
    }
  }

  /** returns the last read state of the input at the index.
   ** true indicates the pin is held HIGH.
   **
   ** See above for the layout of data when using multiple
   ** devices in series or parallel.
   ***/
  inline const bool State(int index) const { return states_[index]; }

  inline const Config &GetConfig() const { return config_; }

private:
  static constexpr int kTotalStates = 8 * num_daisychained * num_parallel;
  Config config_;
  bool states_[kTotalStates];
  uint32_t clk_;
  uint32_t latch_;
  uint32_t data_[num_parallel];
};

} // namespace daisy

#endif