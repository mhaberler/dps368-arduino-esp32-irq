#pragma once
#include <Dps3xx.h>

// range-compatible with DPS__* status codes - see dps_config.h
#define DPSPROBE_INTERRUPTS_OK      0 // as expected
#define DPSPROBE_NOT_FOUND        -10 // no response on given I2C address
#define DPSPROBE_NO_INTERRUPTS    -11 // device found but no interrupts detected
#define DPSPROBE_IRQ_LINE_RINGING -12 // unexplained high number of interrupts
#define DPSPROBE_INCONSISTENT_INTERRUPTS -13 // some interrupts seen, but less than expected

int16_t dps368_probe(TwoWire &w, uint8_t i2c_address, uint8_t irq_pin);
void dps368_perror(int16_t result);