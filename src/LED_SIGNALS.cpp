#include <LED_SIGNALS.h>


/**
 * @brief Метод инициализации светодиода.
 * 
 * @param LED_PIN - Пин, на котором находится светодиод
 * @param StateLED - состояние светодиода при инициализации
 */
void ICACHE_RAM_ATTR LED_SIGNAL::initLed(const uint8_t &LED_PIN, bool StateLED)
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, StateLED);
    _led = LED_PIN;
    _stateLED = StateLED;
}




void ICACHE_RAM_ATTR LED_SIGNAL::ledInvert()
{
    _stateLED = !_stateLED;
    digitalWrite(_led, _stateLED);
}


void ICACHE_RAM_ATTR LED_SIGNAL::ledON()
{
    digitalWrite(_led, HIGH);
}


void ICACHE_RAM_ATTR LED_SIGNAL::ledOFF()
{
    digitalWrite(_led, LOW);
}