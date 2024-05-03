#include "dps3xxprobe.hpp"

#define TEST_DURATION 100 //ms
#define NUM_TESTS 3

static volatile int32_t irq_counter;
static Dps3xx dps3 = Dps3xx();

static void IRAM_ATTR irqtest_handler(void) {
    irq_counter = irq_counter + 1;
}


static bool i2c_probe(TwoWire &w, uint8_t addr) {
    w.beginTransmission(addr);
    return (w.endTransmission() == 0);
}

// verify that
//      the device is responding
//      initialisation works
//      interrupts are detected
//      the number of interrupts seen makes sense
int16_t dps368_probe(TwoWire &w, uint8_t i2c_address, uint8_t irq_pin) {
    uint8_t polarity;
    uint32_t now;
    uint32_t prev_irq_counter;
    uint32_t num_tests = NUM_TESTS;
    int16_t result;
    int16_t rc;
    // Dps3xx dps3 = Dps3xx();  // crashes in dtor at end of function
    irq_counter = 0;

    log_d("probing at 0x%x, IRQ pin %u", i2c_address, irq_pin);

    if (!i2c_probe(w, i2c_address)) {
        return DPSPROBE_NOT_FOUND;
    }

    pinMode(irq_pin, INPUT);
    bool irq_pin_idle = digitalRead(irq_pin);
    // The sensor's address is 0x77 (default) or  (if the SDO pin is pulled-down to GND)
    // if the irq pin reads as low, the device must be on the secondary address
    log_d("irq_pin level=%d", digitalRead(irq_pin));

    if (i2c_address == 0x77) {
        // on standard address
        pinMode(IRQ_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(irq_pin), irqtest_handler, FALLING );
        polarity = 1;
    } else {
        pinMode(IRQ_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(irq_pin), irqtest_handler, RISING);
        polarity = 0;
    }
    dps3.begin(w, i2c_address);
    dps3.standby();
    if ((rc = dps3.setInterruptSources(DPS3xx_BOTH_INTR, polarity)) != 0) {
        log_e("setInterruptSources: %d", rc);
        goto UNWIND;
    }
    dps3.getIntStatusPrsReady();

    prev_irq_counter = irq_counter;
    while (num_tests--) {
        dps3.startMeasurePressureOnce(0); // no oversampling
        now = millis();
        while (millis() - now < TEST_DURATION) {
            if (dps3.getIntStatusPrsReady()) {
                log_d("conversion ready..");

            }
            if (prev_irq_counter ^ irq_counter) {
                float result;
                int16_t rc ;
                rc = dps3.getSingleResult(result);
                log_d("IRQ detected, result=%f", result);
                prev_irq_counter = irq_counter;
            }
            yield();
        }
    }
    if ((irq_counter - NUM_TESTS) > 1) {
        result = DPSPROBE_IRQ_LINE_RINGING;
    } else if (irq_counter == 0) {
        result = DPSPROBE_NO_INTERRUPTS;
    } else if ((irq_counter > 0) && (irq_counter < NUM_TESTS)) {
        result = DPSPROBE_INCONSISTENT_INTERRUPTS;
    } else {
        result = DPSPROBE_INTERRUPTS_OK;
    }
UNWIND:
    log_d("irq_counter = %u", irq_counter);
    dps3.setInterruptSources(DPS3xx_NO_INTR, 0);
    dps3.end();
    detachInterrupt(irq_pin);
    return result;
}

void dps368_perror(int16_t result) {
    switch (result) {
        case DPSPROBE_NOT_FOUND:
            log_e("no I2C device found at given address");
            break;
        case DPSPROBE_NO_INTERRUPTS:
            log_e("I2C device found, no interrupts detected");
            break;
        case DPSPROBE_INTERRUPTS_OK:
            log_e("I2C device found, interrupts working");
            break;
        case DPSPROBE_IRQ_LINE_RINGING:
            log_e("unexplained high number of interrupts");
            break;
        case DPSPROBE_INCONSISTENT_INTERRUPTS:
            log_e("some interrupts seen, but less than expected");
            break;
        case DPS__FAIL_UNKNOWN:
            log_e("dpslib error: unknown failure");
            break;
        case DPS__FAIL_INIT_FAILED:
            log_e("dpslib error: init failed");
            break;
        case DPS__FAIL_TOOBUSY:
            log_e("dpslib error: too busy");
            break;
        case DPS__FAIL_UNFINISHED:
            log_e("dpslib error: fail unfinished");
            break;
    }
}
