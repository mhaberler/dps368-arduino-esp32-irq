#include <M5Unified.h>
#include <Dps3xx.h>
#include "dps3xxprobe.hpp"


void setup() {

    delay(3000);
    M5.begin();
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);
    uint8_t addr = 0x76;

    int16_t result = dps368_probe(Wire, addr, IRQ_PIN);
    dps368_perror(result);
}

void loop() {
    delay(1);
}
