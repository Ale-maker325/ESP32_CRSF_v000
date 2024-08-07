#ifndef _____LED______
#define _____LED______

#include <Arduino.h>

#define GPIO_PIN_LED
#define GPIO_LED_INVERTED 0


class LED_SIGNAL
{
    public:

        void ICACHE_RAM_ATTR initLed(const uint8_t &LED_PIN, bool StateLED);
        void ICACHE_RAM_ATTR ledInvert();
        void ICACHE_RAM_ATTR ledON();
        void ICACHE_RAM_ATTR ledOFF();
        

    private:
        uint8_t _led;
        bool _stateLED;

};





#endif