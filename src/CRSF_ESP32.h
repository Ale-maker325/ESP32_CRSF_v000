/**
 * @file CRSF_ESP32.h
 * @author Alexander
 * @brief Основано на файле репозитория: https://github.com/CapnBry/CRServoF/blob/main/lib/CrsfSerial/crsf_protocol.h
 * @version 0.1
 * @date 2024-08-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _CRSF____________
#define _CRSF____________

#include <Arduino.h>
#include "HardwareSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include <telemetry_protocol.h>
#include <crc.h>
#include "esp32-hal-uart.h"
#include <stdio.h>
#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"


/**
 * Модификатор выравнивания (один из многих вариантов). В данном случае, этот применяется к простым переменным,
 * или отдельным членам агрегата (к структуре или классу). Как атрибут типа он применяется ко всем членам всех
 * агрегатов, объявленных для этого типа. 
 * Устанавливает максимальное выравнивание выбранной переменной или переменных, к которым оно применяется, на
 * наименьшее возможное значение выравнивания.
*/
#define PACKED __attribute__((packed))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define CRSF_SYNC_BYTE          0XC8    //Байт синхронизации

#define CRSF_RX_BAUDRATE        420000
#define CRSF_NUM_CHANNELS       16      //Максимальное количество каналов

#define CRSF_CHANNEL_VALUE_MIN  172     // 987us - actual CRSF min is 0 with E.Limits on
#define CRSF_CHANNEL_VALUE_1000 191     //??? - это из библиотеки для ЛРС
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_2000 1792    //??? - это из библиотеки для ЛРС
#define CRSF_CHANNEL_VALUE_MAX  1811    // 2012us - actual CRSF max is 1984 with E.Limits on
#define CRSF_ELIMIT_US_MIN      891     // microseconds for CRSF=0 (E.Limits=ON)
#define CRSF_ELIMIT_US_MAX      2119    // microseconds for CRSF=1984
#define CRSF_MAX_PACKET_LEN     64
#define CRSF_MAX_PACKET_SIZE    64 // max declared len is 62+DEST+LEN on top of that = 64
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_PACKET_SIZE - 4) // Max size of payload in [dest] [len] [type] [payload] [crc8]

//#define CRSF_CHANNEL_VALUE_SPAN (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_CRC_POLY 0xd5              //??? - это из библиотеки для ЛРС

#define RCframeLength 22             //Длина передаваемого пакета данных (фрейм) для канала радио: 16 каганалов по 11 бит в каждом.
#define ExtFrameTypeLength 60        //Длина расширенного пакета данных (пинг и запрос конфигурации)

//#define CRSF_PAYLOAD_SIZE_MAX 62
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define CRSF_FRAME_SIZE(payload_size) ((payload_size) + 2) // See crsf_header_t.frame_size
#define CRSF_EXT_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + 2)
#define CRSF_FRAME_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_NOT_COUNTED_BYTES)

#define CRSF_FRAME_CRC_SIZE 1               //??? - это из библиотеки для ЛРС
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC 4    ////??? - это из библиотеки для ЛРС, length of Extended Dest/Origin, TYPE and CRC fields combined

#define CRSF_TELEMETRY_LENGTH_INDEX 1                                       //??? - это из библиотеки для ЛРС
#define CRSF_TELEMETRY_TYPE_INDEX 2                                         //??? - это из библиотеки для ЛРС
#define CRSF_TELEMETRY_FIELD_ID_INDEX 5                                     //??? - это из библиотеки для ЛРС
#define CRSF_TELEMETRY_FIELD_CHUNK_INDEX 6                                  //??? - это из библиотеки для ЛРС
#define CRSF_TELEMETRY_CRC_LENGTH 1                                         //??? - это из библиотеки для ЛРС
#define CRSF_TELEMETRY_TOTAL_SIZE(x) (x + CRSF_FRAME_LENGTH_EXT_TYPE_CRC)   //??? - это из библиотеки для ЛРС

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CRSF_MSP_REQ_PAYLOAD_SIZE 8
#define CRSF_MSP_RESP_PAYLOAD_SIZE 58
#define CRSF_MSP_MAX_PAYLOAD_SIZE (CRSF_MSP_REQ_PAYLOAD_SIZE > CRSF_MSP_RESP_PAYLOAD_SIZE ? CRSF_MSP_REQ_PAYLOAD_SIZE : CRSF_MSP_RESP_PAYLOAD_SIZE)








/**
 * @brief Перечисление типов пакетов (фреймов) CRSF (https://github.com/crsf-wg/crsf/wiki/Packet-Types)
 * 
 */
typedef enum
{
    CRSF_FRAMETYPE_GPS =                        0x02, //GPS-позиция, скорость относительно земли, направление, высота, количество спутников
    CRSF_FRAMETYPE_VARIO =                      0x07, //Вертикальная скорость
    CRSF_FRAMETYPE_BATTERY_SENSOR =             0x08, //Напряжение батареи, ток, мАч, оставшийся процент
    CRSF_FRAMETYPE_BARO_ALTITUDE =              0x09, //Барометрическая высота, вертикальная скорость (опционально)
    
    /**
     * @brief //CRSFv3 - uint16_t Исходный адрес устройства (Big Endian) например, Flight Controller
     * находится в сети, отправляет CRSF_ADDRESS_FLIGHT_CONTROLLER0xC8 / 200
     * Сообщение Heartbeat должно отправляться всегда, даже если телеметрия отключена, что позволяет 
     * другой стороне соединения обнаруживать несоответствия скорости передачи данных.
    */
    CRSF_FRAMETYPE_HEARTBEAT =                  0x0B,                  
    
    CRSF_FRAMETYPE_LINK_STATISTICS =            0x14, //Информация о сигнале. RSSI восходящей/нисходящей линии связи, SNR, качество связи (LQ), режим RF, мощность передачи
    CRSF_FRAMETYPE_OPENTX_SYNC =                0x10, //
    CRSF_FRAMETYPE_RADIO_ID =                   0x3A, //
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED =         0x16, //Каналы передачи данных (как с трубки на TX, так и с RX на контроллер полета)

    /**
     * @brief CRSFv3 - Пакет subset channels позволяет отправлять данные каналов с произвольной точностью
     * и в большем количестве, чем устаревший пакет CRSF_FRAMETYPE_RC_CHANNELS_PACKED . Данные каналов могут
     * быть 10-13 бит, но все представляют значения от 998us до 2012us.
    */
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED =  0x17,  
                                        
    CRSF_FRAMETYPE_LINK_RX_ID =                 0x1C,  //CRSFv3 - Элемент телеметрии, ExpressLRS не генерирует этот кадр. Информация о сигнале нисходящей линии связи (БПЛА-земля)
    CRSF_FRAMETYPE_LINK_TX_ID =                 0x1D,  //CRSFv3 - Элемент телеметрии, ExpressLRS не генерирует этот кадр. Информация о сигнале восходящей линии связи (земля-БПЛА)
    CRSF_FRAMETYPE_ATTITUDE =                   0x1E,  //Положение: тангаж, крен, рыскание
    CRSF_FRAMETYPE_FLIGHT_MODE =                0x21,  //Строка режима полета контроллера полета
    CRSF_FRAMETYPE_DEVICE_PING =                0x28,  //Отправитель запрашивает DEVICE_INFO со всех устройств назначения
    CRSF_FRAMETYPE_DEVICE_INFO =                0x29,  //Имя устройства, версия прошивки, версия оборудования, серийный номер (ответ PING)
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY =   0x2B,  //Фрагмент данных элемента конфигурации
    CRSF_FRAMETYPE_PARAMETER_READ =             0x2C,  //Запрос на чтение элемента конфигурации
    CRSF_FRAMETYPE_PARAMETER_WRITE =            0x2D,  //Запрос на запись элемента конфигурации
    CRSF_FRAMETYPE_ELRS_STATUS =                0x2E,  //!!Нестандартно!! Количество хороших/плохих пакетов ExpressLRS, флаги состояния
    CRSF_FRAMETYPE_COMMAND =                    0x32,  //Выполнение команды CRSF
    CRSF_FRAMETYPE_KISS_REQ  =                  0x78,  //Запрос KISS
    CRSF_FRAMETYPE_KISS_RESP =                  0x79,  //ответ KISS
    CRSF_FRAMETYPE_MSP_REQ =                    0x7A,  //Запрос/команда параметра MSP
    CRSF_FRAMETYPE_MSP_RESP =                   0x7B,  //Фрагмент ответа параметра MSP (reply with 58 byte chunked binary)
    CRSF_FRAMETYPE_MSP_WRITE =                  0x7C,  //Запись параметра MSP (write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit))
    CRSF_FRAMETYPE_DISPLAYPORT_CMD =            0x7D,  //CRSFv3 - Команда MSP DisplayPort
    CRSF_FRAMETYPE_ARDUPILOT_RESP =             0x80,  //Ответ Ardupilot
} crsf_frame_type_e;



/**
 * @brief Адреса CRSF
 * 
 */
typedef enum : uint8_t
{
    CRSF_ADDRESS_BROADCAST =         0x00,   //Широковещательная передача (все устройства обрабатывают пакет)
    CRSF_ADDRESS_USB =               0x10,   //?
    CRSF_ADDRESS_BLUETOOTH =         0x12,   //Bluetooth-модуль
    CRSF_ADDRESS_TBS_CORE_PNP_PRO =  0x80,   //?
    CRSF_ADDRESS_RESERVED1 =         0x8A,   //Зарезервировано
    CRSF_ADDRESS_CURRENT_SENSOR =    0xC0,   //Внешний датчик тока
    CRSF_ADDRESS_GPS =               0xC2,   //Внешний GPS-приемник
    CRSF_ADDRESS_TBS_BLACKBOX =      0xC4,   //Внешнее устройство регистрации Blackbox
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,   //Контроллер полета (Betaflight / iNav)
    CRSF_ADDRESS_RESERVED2 =         0xCA,   //Зарезервировано
    CRSF_ADDRESS_RACE_TAG =          0xCC,   //Гоночный тег?
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,   //Трубка (EdgeTX), не передатчик
    CRSF_ADDRESS_CRSF_RECEIVER =     0xEC,   //Аппаратное обеспечение приемника (TBS Nano RX / RadioMaster RP1)
    CRSF_ADDRESS_CRSF_TRANSMITTER =  0xEE,   //Модуль передатчика, а не трубка
    CRSF_ADDRESS_ELRS_LUA =          0xEF    //!!Нестандартный!! Исходный адрес, используемый ExpressLRS Lua
} crsf_addr_e;



/**
 * Структура хранения пакетов канала Crossfire, каждый канал 11 бит (всего 16 каналов)
 */
typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

typedef struct crsf_extended_packet_t
{
    unsigned extDest : 1;
    unsigned extSrc :  1;
    unsigned header_dest :  1;
    unsigned header_src :  1;
    unsigned payload : 1;
    unsigned payload2 : 1;
    unsigned payload3 : 1;
    unsigned payload4 : 1;
    unsigned crc : 1;
    
} PACKED crsf_packet_t;





/**
 * Определение формы стандартного пакета ЛРС
 */
typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;


// Определение формы расширенного пакета ЛРС (все типы пакетов в диапазоне от 0x28 до 0x96)
typedef struct crsf_ext_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED crsf_ext_header_t;




/* CRC8 implementation with polynom = x​7​+ x​6​+ x​4​+ x​2​+ x​0 ​(0xD5) */
static unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////        ЭТО СТАРАЯ БИБЛИОТЕКА    ///////////////////////////////////////

static inline uint16_t ICACHE_RAM_ATTR UINT_to_CRSF(uint16_t Val);

static inline uint8_t ICACHE_RAM_ATTR CalcCRC(volatile uint8_t *data, int length);
static inline uint8_t ICACHE_RAM_ATTR CalcCRC(uint8_t *data, int length);

uint8_t ICACHE_RAM_ATTR CalcCRC(volatile uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

uint8_t ICACHE_RAM_ATTR CalcCRC(uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////








/**
 * Класс для работы с протоколом CRSF
*/
class CRSF
{

public:
        
    static HardwareSerial Port;                        //Экземпляр класса HardwareSerial для создания своего Serial-порта
    
    static void Begin();
    static void End();
    
    void ICACHE_RAM_ATTR sendFrameToFC();
    void ICACHE_RAM_ATTR sendFrameToTR();
    void ICACHE_RAM_ATTR sendExtendedPacket(uint8_t typePacket);
    void ICACHE_RAM_ATTR sendPacket();
    void ICACHE_RAM_ATTR readFromSerial();

    uint8_t inputBuffer[CRSF_MAX_PACKET_LEN + 1] = {0};       //Входной буффер - максимально 64 байта для пакетов CRSF
    uint8_t outputBuffer[CRSF_MAX_PACKET_LEN + 1] = {0};      //Выходной буффер - максимально 64 байта для пакетов CRSF
    uint8_t readingPacketLen = 0;                             // Считываемая длина пакета CRSF который пришёл в сериал порт
    uint8_t positionReadWr = 0;                               // Индекс, где в данный момент мы читаем-пишем
    uint32_t GoodPktsCount;
    uint32_t BadPktsCount;

    static uint8_t CSFR_TXpin_Module;
    static uint8_t CSFR_RXpin_Module;

    /**
     * Структура, хранящая значения всех каналов перед отправкой в порт
    */
    static volatile crsf_channels_s PackedRCdataOut;

    /**
     * Структура, хранящая значения запроса перед отправкой в порт
    */
    static volatile crsf_extended_packet_t PackedExtdataOut;

    static void SetHeaderAndCrc(uint8_t *frame, crsf_frame_type_e frameType, uint8_t frameSize, crsf_addr_e destAddr);
    static void SetExtendedHeaderAndCrc(uint8_t *frame, crsf_frame_type_e frameType, uint8_t frameSize, crsf_addr_e senderAddr, crsf_addr_e destAddr);
    
private:
    static void flush_port_input(void);
};

extern GENERIC_CRC8 crsf_crc;






//Конец заголовочного файла
#endif