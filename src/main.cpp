#include <M5Unified.h>
#include <Dps3xx.h>

typedef struct {
    uint32_t timestamp;
    int16_t pin;
} irqmsg_t;

QueueHandle_t irq_queue;
Dps3xx Dps3xxPressureSensor = Dps3xx();
int16_t irq_pin = IRQ_PIN;
uint32_t count;

// first level interrupt handler
// only notify 2nd level handler task passing any parameters
static void IRAM_ATTR  irq_handler(void *param) {
    int16_t *pin = (int16_t *)param;
    static uint32_t counter;
    irqmsg_t msg;
    msg.pin = *pin;
    msg.timestamp = micros();
    xQueueSendFromISR(irq_queue, (const void*) &msg, NULL);
}

// 2nd level interrupt handler
// in user context - can do Wire I/O etc
void soft_irq(void* arg) {
    irqmsg_t msg;
    uint8_t pressureCount = 20;
    float pressure[pressureCount];
    uint8_t temperatureCount = 20;
    float temperature[temperatureCount];
    for (;;) {
        if (xQueueReceive(irq_queue, &msg, portMAX_DELAY)) {
            float value;
            int16_t rc ;
            rc = Dps3xxPressureSensor.getSingleResult(value);
            if (Dps3xxPressureSensor.getIntStatusPrsReady()) {
                log_i("pin=%d timestamp=%u pressure=%.2f kPa rc=%d",
                      msg.pin, msg.timestamp, value, rc);
            } else {
                log_i("pin=%d timestamp=%u temperature=%.2f° rc=%d",
                      msg.pin, msg.timestamp, value, rc);
            };

            // start a new measurent
            // every 8 pressure measurements we start a temperature measurement to keep
            // correction accurate
            count++;
            if (!(count & 7)) {
                log_i("start temperature measurement");
                rc = Dps3xxPressureSensor.startMeasureTempOnce(TEMP_OVERSAMPLING_RATE);
            } else {
                rc = Dps3xxPressureSensor.startMeasurePressureOnce(PRS_OVERSAMPLING_RATE);
            }
        }
    }
}


void setup() {
    int rc;

    delay(3000);
    M5.begin();
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    irq_queue = xQueueCreate(10, sizeof(irqmsg_t));
    xTaskCreate(soft_irq, "soft_irq", 2048, NULL, 10, NULL);

    pinMode(IRQ_PIN, INPUT);
    attachInterruptArg(digitalPinToInterrupt(irq_pin), irq_handler,(void *)&irq_pin, FALLING);

    Dps3xxPressureSensor.begin(Wire, DPS3XX_I2C_ADDR);

    rc = Dps3xxPressureSensor.setInterruptSources(DPS3xx_BOTH_INTR, 0);
    if (rc) {
        log_i("setInterruptSources: %d",rc);
    }
    // clear interrupt flags by reading the IRQ status register
    rc = Dps3xxPressureSensor.getIntStatusPrsReady();
    if (rc) {
        log_i("getIntStatusPrsReady: %d",rc);
    }

    // one-shot conversion
    rc = Dps3xxPressureSensor.startMeasurePressureOnce(PRS_OVERSAMPLING_RATE);
    if (rc) {
        log_e("startMeasurePressureOnce: %d",rc);
    }
}

void loop() {
    delay(1);
}
