#include <Arduino.h>

#include "CRSF_ESP32.h"
#include <driver\gpio.h>


#define CRSFinterval 5000 //in ms
#define uartCRSFinverted false

CRSF crsf;

#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  crsf.sendRCFrameToFC();
  portEXIT_CRITICAL_ISR(&timerMux);

}

void StartTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, CRSFinterval, true);
  timerAlarmEnable(timer);
}


void setup() {
  // put your setup code here, to run once:
  crsf.Begin();

  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
  
  //pinMode(0, INPUT_PULLUP);

  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
  crsf.sendRCFrameToFC();

  StartTimer();

  //pinMode(4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);

  //digitalWrite(4, HIGH);
  //delay(10);
  //digitalWrite(4, LOW);
  //delay(10);

  if (digitalRead(0)) {
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
  } else {
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MAX;
  }
  crsf.PackedRCdataOut.ch0 = random(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  crsf.PackedRCdataOut.ch1 = random(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = random(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);

}