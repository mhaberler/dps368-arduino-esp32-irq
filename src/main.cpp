// example for interrupt-driven operation of multiple DPS3xx sensors
// supports arbitrary number of sensors
// any per-sensor configuration

#include <M5Unified.h>
#include <Dps3xx.h>

typedef struct  {
    uint8_t i2caddr;
    uint8_t irq_pin;
    uint8_t status;
    bool initialized;
    Dps3xx *sensor;
    uint32_t softirq_count;
    uint32_t temp_measure_mask;
    TwoWire *wire;
    int16_t temp_mr;
    int16_t temp_osr;
    int16_t prs_mr;
    int16_t prs_osr;
} dps_sensors_t;

typedef struct {
    uint32_t timestamp;
    dps_sensors_t *dev;
} irqmsg_t;

QueueHandle_t irq_queue;
uint32_t irq_queue_full;
dps_sensors_t dps_sensors[] = {
    {
        .i2caddr = 0x76,
        .irq_pin = IRQ_PIN,
        .status = 0,
        .initialized = false,
        .sensor = nullptr,
        .temp_measure_mask = 7,
        .wire = &Wire,
        .temp_mr = TEMP_MEASURMENT_RATE,
        .temp_osr = TEMP_OVERSAMPLING_RATE,
        .prs_mr = PRS_MEASURMENT_RATE,
        .prs_osr = PRS_OVERSAMPLING_RATE,
    }
};
#define NUM_DPS (sizeof(dps_sensors)/sizeof(dps_sensors[0]))

// first level interrupt handler
// only notify 2nd level handler task passing any parameters
static void IRAM_ATTR  irq_handler(void *param) {
    irqmsg_t msg;
    msg.dev = static_cast<dps_sensors_t *>(param);
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
            dps_sensors_t *dev = msg.dev;
            Dps3xx *dps = dev->sensor;

            if ((ret = dps->getSingleResult(value)) != 0) {
                log_e("getSingleResult: %d",ret);
                continue;
            }

            if ((ret = dps->getIntStatusPrsReady()) < 0) {
                log_e("getIntStatusPrsReady: %d",ret);
                continue;
            }

            // ret == 0 || ret == 1
            log_i("dev=0x%x timestamp=%u %s=%.2f %s",
                  dev->i2caddr, msg.timestamp,
                  ret ? "pressure" : "temperature",
                  value,
                  ret ? "kPa"  : "°C");

            // start a new measurement cycle
            // every count & TEMP_COUNT_MASK pressure measurements start a temperature measurement
            // to keep correction accurate
            dev->softirq_count++;
            if ((dev->softirq_count & dev->temp_measure_mask)) {
                if ((ret = dps->startMeasurePressureOnce(dev->prs_osr)) != 0) {
                    log_e("startMeasurePressureOnce: %d",ret);
                }
            } else {
                if ((ret = dps->startMeasureTempOnce(dev->temp_osr)) != 0) {
                    log_e("startMeasureTempOnce: %d",ret);
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

    for (auto i = 0; i < NUM_DPS; i++) {
        dps_sensors_t *dev = &dps_sensors[i];
        Dps3xx *dps = dev->sensor = new Dps3xx();

        dps->begin(*dev->wire, dev->i2caddr);
        if ((ret = dps->standby()) != DPS__SUCCEEDED) {
            log_e("standby failed: %d", ret);
            delete dps;
            continue;
        }

        pinMode(dev->irq_pin, INPUT);
        uint8_t polarity;
        if (dev->i2caddr == 0x77) {
            // on standard address
            attachInterruptArg(digitalPinToInterrupt(dev->irq_pin), irq_handler, (void *)dev, RISING);
            polarity = 1;
        } else {
            // secondary address
            attachInterruptArg(digitalPinToInterrupt(dev->irq_pin), irq_handler, (void *)dev, FALLING);
            polarity = 0;
        }
        log_i("dev=%d addr=0x%x irq pin=%u polarity=%u", i, dev->i2caddr, dev->irq_pin, polarity);

        // identify the device
        log_i("DPS3xx: product 0x%x revision 0x%x",
              dps->getProductId(),
              dps->getRevisionId());

        // measure temperature once for pressure compensation
        float temperature;
        if ((ret = dps->measureTempOnce(temperature, dev->temp_osr)) != 0) {
            log_e("measureTempOnce failed ret=%d", ret);
        } else {
            log_i("dps3xx compensating for %.2f°", temperature);
        }

        // define what causes an interrupt: both temperature and pressure conversion
        // FIXME polarity
        if ((ret = dps->setInterruptSources(DPS3xx_BOTH_INTR, polarity)) != 0) {
            log_i("setInterruptSources: %d", ret);
        }

        // clear interrupt flags by reading the IRQ status register
        if ((ret = dps->getIntStatusPrsReady()) != 0) {
            log_i("getIntStatusPrsReady: %d", ret);
        }

        // start one-shot conversion
        // interrupt will be posted at end of conversion
        if ((ret = dps->startMeasurePressureOnce(dev->prs_osr)) != 0) {
            log_e("startMeasurePressureOnce: %d", ret);
        }
        dev->initialized = true;
    }
}

void loop() {
    delay(1);
}
