#include "CRSF_ESP32.h"
#include <Arduino.h>
#include "HardwareSerial.h"

HardwareSerial SerialPort(2);               //Инициализируем сериал интерфейс Serial 2 (в ESP32 это пины 16, 17)
HardwareSerial CRSF::Port = SerialPort;     //Присваиваем встроенному в класс порту инициализируемый порт SerialPort(2)

uint8_t CRSF::CSFR_TXpin_Module = 17;       //определяем пин TX - это будет пин 17
uint8_t CRSF::CSFR_RXpin_Module = 16;       //определяем пин RX - это будет пин 16

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

/**
 * Инициализируем работу класса CRSF путём вызова внутри метода инициализации порта Serial:
 * 
 * Port.begin(CRSF_RX_BAUDRATE, SERIAL_8N1, CSFR_RXpin_Module, CSFR_TXpin_Module, false);
 * 
 * где:
 * 
 *  CRSF_RX_BAUDRATE - задаём скорость работы порта CRSF_RX_BAUDRATE = 420000 в бодах (бит в секунду);
 * 
 *  SERIAL_8N1 - настроить количество бит данных, проверку четности и стоповые биты. По умолчанию,
 *    посылка состоит из 8 бит данных, без проверки четности, с одним стоповым битом;
 * 
 *  CSFR_RXpin_Module - задаём пин пин RX;
 * 
 *  CSFR_TXpin_Module - задаём пин пин TX;
 * 
 *  false/true - задаём инвертирован ли порт или нет. В данном случае установлено значение false;
*/
void CRSF::Begin()
{
  CRSF::Port.begin(CRSF_RX_BAUDRATE, SERIAL_8N1, CSFR_RXpin_Module, CSFR_TXpin_Module, false);
  
}


volatile crsf_channels_s CRSF::PackedRCdataOut; //экземпляр структуры со значениями для каждого канала для отправки
volatile crsf_extended_packet_t CRSF::PackedExtdataOut;//экземпляр структуры со значениями для запроса для отправки


/**
 * Метод, производящий отправку данных на полётный контроллер, поскольку адрес будет CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8.
 * CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16 - Каналы передачи данных (как с трубки на TX, так и с RX на контроллер полета).
 * Данные формируются из структуры, хранящей значения для каналов (PackedRCdataOut) и отправляются в сериал порт в виде байтов.
*/
void ICACHE_RAM_ATTR CRSF::sendFrameToFC()
{   
    //буфер для записи байт в сериал порт.
    uint8_t outBuffer[RCframeLength + 4] = {0};

    outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;      //Первым в буфер идёт адрес назначения - для полётного контроллера
    //length of type (24) + payload + crc Длина следующих байтов, включая тип, полезную нагрузку и CRC (PayloadLength+2).
    //Общая длина пакета составляет PayloadLength+4 (sync, len, type, crc) или LEN+2 (sync, len).
    outBuffer[1] = RCframeLength + 2;                   
    outBuffer[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;   //Тип фрейма - Каналы передачи данных

    /**
     * функция стандартной библиотеки языка программирования Си, копирующая содержимое одной области памяти в другую.
     * Функция определена в заголовочном файле string.h (а также в mem.h), описывается в стандартах ANSI C и POSIX.
     * 
     *      void *memcpy(void *dst, const void *src, size_t n);
     * где:
     * 
     *      dst — адрес буфера назначения
     *      srс — адрес источника
     *      n — количество байт для копирования
     * 
     * Функция копирует n байт из области памяти, на которую указывает src, в область памяти, на которую указывает dst.
     * Функция возвращает адрес назначения dst.
    */
    memcpy(outBuffer + 3, (byte *)&PackedRCdataOut, RCframeLength);

    uint8_t crc = CalcCRC(&outBuffer[2], RCframeLength + 1);

    outBuffer[RCframeLength + 3] = crc;

    CRSF::Port.write(outBuffer, RCframeLength + 4);
}






/**
 * @brief Метод, посылающий запрос в виде расширенного пакета
 * 
 */
void ICACHE_RAM_ATTR CRSF::sendExtendedPacket(uint8_t typePacket)
{
  uint8_t packetBuffer[ExtFrameTypeLength + 4] = {0};

  packetBuffer[0] = CRSF_SYNC_BYTE;            // Байт синхронизации 0XC8
  //Длина следующих байтов, включая тип, полезную нагрузку и CRC (PayloadLength+2).
  //Общая длина пакета составляет PayloadLength+4 (sync, len, type, crc) или LEN+2 (sync, len).
  //Все типы пакетов CRSF_FRAMETYPE 0x28 и выше используют расширенный формат пакетов.
  //В этом случае поле LEN на самом деле представляет собой PayloadLength+4, поскольку первые
  //два байта — это расширенные пункт назначения и источник. Таким образом, максимальная длина
  //полезной нагрузки составляет 58 байт для расширенных пакетов.
  packetBuffer[1] = ExtFrameTypeLength + 4;

  packetBuffer[2] = typePacket;                //Тип отправляемого пакета.CRSF_FRAMETYPE_DEVICE_PING =                0x28,  //Отправитель запрашивает DEVICE_INFO со всех устройств назначения

  memcpy(packetBuffer + 3, (byte *)&PackedExtdataOut, ExtFrameTypeLength);

  uint8_t crc = CalcCRC(& packetBuffer[2], ExtFrameTypeLength + 1);

  packetBuffer[ExtFrameTypeLength + 3] = crc;

  CRSF::Port.write(packetBuffer, ExtFrameTypeLength + 4);

}


#define DEBUG_PRINT



/**
 * @brief Метод, читающий из сериал-порта данные по пакетам
 * 
 */
void ICACHE_RAM_ATTR CRSF::readFromSerial()
{
    //Указатель на входной буффер
    uint8_t *SerialInBuffer = inputBuffer;
    #ifdef DEBUG_PRINT
        Serial.print("Читаем сериал-порт с пина ");
        Serial.print(CSFR_RXpin_Module);
        Serial.println(" ");
    #endif

    //Начинаем читать порт
    while (CRSF::Port.available())
    {
        unsigned char const inChar = CRSF::Port.read();
        bool sinkBiteIsOK = false;
                
        // этап 1 ждать для синхронизации байт - либо получаем байт синхронизации, либо
        // получаем байт того, что пришло обращение к трубке
        if ((inChar == CRSF_ADDRESS_RADIO_TRANSMITTER) || (inChar == CRSF_SYNC_BYTE) || (inChar == CRSF_ADDRESS_CRSF_TRANSMITTER))
        {
            // Мы получили синхронизацию, сбрасываем и записываем у указатель
            positionReadWr = 0;
            readingPacketLen = 0;

            SerialInBuffer[positionReadWr] = inChar;
            positionReadWr++;
            // #ifdef DEBUG_PRINT
            //     Serial.print("Получен байт синхронизации 0x");
            //     Serial.println(inChar, HEX);
            // #endif
            sinkBiteIsOK = true;
        }
        
        else if(sinkBiteIsOK)// Если мы получили на вход другие байты, то это нужно обработать:
        {
            // 1. Если что-то пошло не так, и мы вдруг за пределами размера максимальной длины пакета, то
            // заканчиваем, и выходим из цикла
            if (positionReadWr > CRSF_MAX_PACKET_LEN - 1)
            {
                positionReadWr = 0;
                readingPacketLen = 0;
                #ifdef DEBUG_PRINT
                    Serial.print("ERROR:  Превышена длина пакета ");
                    Serial.println(CRSF_MAX_PACKET_LEN, HEX);
                    Serial.println(" - заканчиваем чтение");
                    Serial.println("RETURN");
                #endif
                return;
            }

            // 2. Если позиция индекса байта = 1, то значит это позиция байта длины отправленного пакета
            //особый случай, когда мы сохраняем ожидаемый pktlen для буфера 
            if (positionReadWr == 1)
            {
                unsigned char const inChar = CRSF::Port.read();
                // #ifdef DEBUG_PRINT
                //     Serial.print("Считали предполагаемую длину пакета: 0x");
                //     Serial.println(inChar, HEX);
                //     Serial.println(" ");
                // #endif
                //Если размер байта вписывается в максимальную длину пакета, значит сохраняем его
                if (inChar <= CRSF_MAX_PACKET_LEN)
                {
                    readingPacketLen = inChar;
                    SerialInBuffer[positionReadWr] = readingPacketLen;
                    positionReadWr++;
                    // #ifdef DEBUG_PRINT
                    //     Serial.print("OK. Длина пакета = 0x");
                    //     Serial.println(readingPacketLen, HEX);
                    //     Serial.println(" ");
                    // #endif
                }
                //Если не вписвается, значит обнуляемся и начинаем заново
                else
                {
                    #ifdef DEBUG_PRINT
                        Serial.print("Длина пришедшего пакета ERROR");
                        Serial.println("RETURN");
                        Serial.println(" ");
                    #endif
                    positionReadWr = 0;
                    readingPacketLen = 0;
                    
                    return;
                }
            }

                      
            //После предыдущего шага имеем общую длину всего пакета который пришёл и который нужно считать с
            //позиции, на которой остановились.

            //Общая длина входного пакета будет состоит из синхронизационного байта, len и PayloadLength (тип пакета, полезная нагрузка и CRC),
            // поэтому мы прибавляем к считанному значению 2 (синхробайт и байт длины) чтобы получить общую длину данных
            uint8_t allLengthPacket = readingPacketLen + 2;

            int toRead = allLengthPacket - positionReadWr;   //количество байт, которые нам осталось считать из буфера

            //Считываем все данные из буфера в соответствии с количеством, определённым выше и считаем сколько было
            //считано штук
            auto count = (int)CRSF::Port.read(& SerialInBuffer[positionReadWr], toRead);
            positionReadWr += count;

            if (positionReadWr >= (readingPacketLen + 2)) // плюс 2, потому что packlen ссылается с начала флага 'тип', то есть есть дополнительные 2 байта.
            {
                uint8_t CalculatedCRC = crsf_crc.calc(SerialInBuffer + 2, positionReadWr - 3);

                if (CalculatedCRC == SerialInBuffer[positionReadWr-1])
                {
                    GoodPktsCount++;
                    #ifdef DEBUG_PRINT
                        Serial.print("OK - CRC в норме");
                        Serial.println(" ");
                    #endif
                }
                else
                {
                    #ifdef DEBUG_PRINT
                        Serial.print("ERROR - Ошибка вычисления CRC");
                        Serial.println(" ");
                    #endif
                    flush_port_input();
                    BadPktsCount++;
                }
                
                positionReadWr = 0;
                readingPacketLen = 0;
            }
        


            #ifdef DEBUG_PRINT

                Serial.print("Получен байт синхронизации 0x");
                Serial.println(SerialInBuffer[0], HEX);
                Serial.print("Считали длину пакета: 0x");
                Serial.println(SerialInBuffer[1], HEX);
                for(int i = 0; i <= CRSF_MAX_PACKET_LEN; i++)
                {
                    Serial.print(" 0x");
                    Serial.print(SerialInBuffer[i], HEX);
                }
                Serial.println(" ");
                
                //return;
            #endif
        }
        else if(!sinkBiteIsOK)
        {
            #ifdef DEBUG_PRINT
                Serial.println("Нет синхронизации!!!");
            #endif
            return;
        }
    }
}




/**
 * @brief Метод для очистки всех возможных данных сериал порта от мусора
 * просто мусор весь считывается и порт освобождается для работы
 */
void CRSF::flush_port_input(void)
{
    // Make sure there is no garbage on the UART at the start
    while (CRSF::Port.available())
    {
        CRSF::Port.read();
    }
}



