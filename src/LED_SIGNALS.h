#ifndef _____LED______
#define _____LED______

#include <Arduino.h>


#define PACKED __attribute__((packed))

/**
 * @brief Структура, которая будет хранить состояние кнопки и её
 * свойства во время работы
 * 
 */
struct BUTTON_PROPERTY 
{
    bool isInverted = false;
    bool isON = false;
    bool isOff = true;

} PACKED;



class LED_SIGNAL
{
    public:

        void ICACHE_RAM_ATTR initLed(const uint8_t &LED_PIN, bool StateLED);
        void ICACHE_RAM_ATTR ledInvert();
        void ICACHE_RAM_ATTR ledON();
        void ICACHE_RAM_ATTR ledOFF();

        BUTTON_PROPERTY button_property;                

    private:
        uint8_t _led;
        bool _stateLED;

};





#endif