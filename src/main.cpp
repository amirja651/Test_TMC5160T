#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include "config.h"

TMC5160Stepper driver = TMC5160Stepper(TMC_CS_PIN, SPI_MOSI, SPI_MISO, SPI_SCK);

// متغیرهای گلوبال برای کنترل حرکت
bool          isMoving     = false;
bool          direction    = true;  // true = forward, false = reverse
const int     stepDelay    = 500;   // میکروثانیه
unsigned long lastStepTime = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);  // زمان بیشتر برای راه‌اندازی

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // تست SPI
    pinMode(TMC_CS_PIN, OUTPUT);
    digitalWrite(TMC_CS_PIN, HIGH);

    // تست ارتباط SPI
    uint8_t test_value = 0x55;
    digitalWrite(TMC_CS_PIN, LOW);
    SPI.transfer(test_value);
    digitalWrite(TMC_CS_PIN, HIGH);

    Serial.println("SPI Test completed");

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, HIGH);  // غیرفعال کردن موتور در ابتدا
    delay(200);

    driver.begin();  // شروع ارتباط با درایور
    delay(200);

    // تنظیمات پایه و ضروری
    driver.GCONF(0x0);  // تنظیمات پایه
    delay(100);

    // افزایش جریان موتور به مقدار مناسب‌تر
    driver.rms_current(1000);  // تنظیم به 1000mA
    delay(100);

    driver.microsteps(16);  // تنظیم میکرواستپ به 16
    delay(100);

    driver.toff(5);  // تنظیم مناسب‌تر
    driver.blank_time(24);
    driver.iholddelay(6);
    driver.TPOWERDOWN(10);
    delay(100);

    // چک کردن ارتباط
    uint32_t gconf = driver.GCONF();
    Serial.print("GCONF: 0x");
    Serial.println(gconf, HEX);

    uint32_t drv_status = driver.DRV_STATUS();
    Serial.print("DRV_STATUS: 0x");
    Serial.println(drv_status, HEX);

    delay(1000);
    digitalWrite(EN_PIN, LOW);  // فعال کردن موتور
    delay(200);
}

void loop() {
    // خواندن ورودی سریال
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'f':  // حرکت به جلو
                direction = true;
                isMoving  = true;
                digitalWrite(DIR_PIN, HIGH);
                Serial.println("Moving Forward");
                break;
            case 'r':  // حرکت به عقب
                direction = false;
                isMoving  = true;
                digitalWrite(DIR_PIN, LOW);
                Serial.println("Moving Reverse");
                break;
            case 's':  // توقف
                isMoving = false;
                Serial.println("Stopped");
                break;
        }
    }

    // کنترل حرکت موتور
    if (isMoving) {
        if (micros() - lastStepTime >= stepDelay) {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(10);
            digitalWrite(STEP_PIN, LOW);
            lastStepTime = micros();

            // چاپ وضعیت هر 1000 استپ
            static int stepCounter = 0;
            if (++stepCounter >= 1000) {
                uint32_t status = driver.DRV_STATUS();
                Serial.print("DRV_STATUS: 0x");
                Serial.println(status, HEX);
                stepCounter = 0;
            }
        }
    }
}