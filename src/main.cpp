#include <M5Unified.h>
#include <Dps3xx.h>

#define TEMP_COUNT_MASK 7

typedef struct {
    uint32_t timestamp;
    uint8_t device_id;
} irqmsg_t;

QueueHandle_t irq_queue;
Dps3xx dps3 = Dps3xx();
int16_t irq_pin = IRQ_PIN;
uint32_t count;
uint32_t irq_queue_full;
uint32_t device = 42;

// first level interrupt handler
// only notify 2nd level handler task passing any parameters
static void IRAM_ATTR  irq_handler(void *param) {
    irqmsg_t msg;
    msg.device_id = ((uint32_t)param) % 0xff;
    msg.timestamp = micros();
    if (xQueueSendFromISR(irq_queue, (const void*) &msg, NULL) != pdTRUE) {
        irq_queue_full++;
    }
}

// 2nd level interrupt handler
// runs in user context - can do Wire I/O, log etc
void soft_irq(void* arg) {
    irqmsg_t msg;

    for (;;) {
        if (xQueueReceive(irq_queue, &msg, portMAX_DELAY)) {
            float value;
            int16_t ret ;

            if ((ret = dps3.getSingleResult(value)) != 0) {
                log_e("getSingleResult: %d",ret);
                continue;
            }

            if ((ret = dps3.getIntStatusPrsReady()) < 0) {
                log_e("getIntStatusPrsReady: %d",ret);
                continue;
            }
            // ret == 0 || ret == 1
            log_i("dev=%d timestamp=%u %s=%.2f %s",
                  msg.device_id, msg.timestamp,
                  ret ? "pressure" : "temperature",
                  value,
                  ret ? "kPa"  : "°C");

            // start a new measurement cycle
            // every count & TEMP_COUNT_MASK pressure measurements start a temperature measurement
            // to keep correction accurate
            count++;
            if (!(count & TEMP_COUNT_MASK)) {
                if ((ret = dps3.startMeasureTempOnce(TEMP_OVERSAMPLING_RATE)) != 0) {
                    log_e("startMeasureTempOnce: %d",ret);
                }
            } else {
                if ((ret = dps3.startMeasurePressureOnce(PRS_OVERSAMPLING_RATE)) != 0) {
                    log_e("startMeasurePressureOnce: %d",ret);
                }
            }
        }
    }
}


void setup() {
    int16_t ret;

    delay(3000);
    M5.begin();
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    irq_queue = xQueueCreate(10, sizeof(irqmsg_t));
    xTaskCreate(soft_irq, "soft_irq", 2048, NULL, 10, NULL);

    pinMode(IRQ_PIN, INPUT);
    attachInterruptArg(digitalPinToInterrupt(irq_pin), irq_handler,(void *)device, FALLING);

    dps3.begin(Wire, DPS3XX_I2C_ADDR);

    // identify the device
    log_i("DPS3xx: product 0x%x revision 0x%x",
          dps3.getProductId(),
          dps3.getRevisionId());

    // measure temperature once for pressure compensation
    float temperature;
    uint8_t oversampling = TEMP_OVERSAMPLING_RATE;
    if ((ret = dps3.measureTempOnce(temperature, oversampling)) != 0) {
        log_e("measureTempOnce failed ret=%d", ret);
    } else {
        log_i("dps3xx compensating for %.2f°", temperature);
    }

    // define what causes an interrupt: both temperature and pressure conversion
    if ((ret = dps3.setInterruptSources(DPS3xx_BOTH_INTR, 0)) != 0) {
        log_i("setInterruptSources: %d",ret);
    }

    // clear interrupt flags by reading the IRQ status register
    if ((ret = dps3.getIntStatusPrsReady()) != 0) {
        log_i("getIntStatusPrsReady: %d",ret);
    }

    // start one-shot conversion
    // interrupt will be posted at end of conversion
    if ((ret = dps3.startMeasurePressureOnce(PRS_OVERSAMPLING_RATE)) != 0) {
        log_e("startMeasurePressureOnce: %d",ret);
    }
}

void loop() {
    delay(1);
}
