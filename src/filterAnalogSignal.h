/**
 * @file filterAnalogSignal.h
 * @author Alexander
 * @brief Аналоговый фильтр для сигнала со стиков либо другого чего
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _analog_filter____________
#define _analog_filter____________

#include <Arduino.h>
#include <CRSF_ESP32.h>

/**
 * Модификатор выравнивания (один из многих вариантов). В данном случае, этот применяется к простым переменным,
 * или отдельным членам агрегата (к структуре или классу). Как атрибут типа он применяется ко всем членам всех
 * агрегатов, объявленных для этого типа. 
 * Устанавливает максимальное выравнивание выбранной переменной или переменных, к которым оно применяется, на
 * наименьшее возможное значение выравнивания.
*/
#define PACKED __attribute__((packed))

static const uint8_t NUM_READ = 10;  // порядок медианы

/**
 * @brief Структура для хранения буферов аналоговых сигналов для фильтрации
 * Сюда записываются считанные аналоговые выборки с пинов чтобы потом из
 * них вывести среднее значение аналогового сигнала
 * 
 */
struct analog_channels_filter_data
{
   uint32_t buffer_Throttle[NUM_READ];  // статический буфер для Throttle
   uint32_t buffer_Yaw[NUM_READ];       // статический буфер для Yaw
   uint32_t buffer_Pitch[NUM_READ];     // статический буфер для Pitch
   uint32_t buffer_Roll[NUM_READ];      // статический буфер для Roll
} PACKED filter_Data;



/**
 * @brief Функция (подфункция), производящая операции фильтрации
 * 
 */
void filtered_data(uint32_t *data, uint8_t &count, const uint8_t &NUM_READ)
{
  if ((count < NUM_READ - 1) and (data[count] > data[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (data[i] > data[i + 1]) {
        uint32_t buff = data[i];
        data[i] = data[i + 1];
        data[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (data[count - 1] > data[count])) {
      for (int i = count; i > 0; i--) {
        if (data[i] < data[i - 1]) {
          int buff = data[i];
          data[i] = data[i - 1];
          data[i - 1] = buff;
        }
      }
    }
  }
}



/**
 * @brief Функция медианного фильтра
 * 
 * @param newThrottle - входящее аналоговое значение для фильтрации
 * @param newYaw - входящее аналоговое значение для фильтрации
 * @param newPitch - входящее аналоговое значение для фильтрации
 * @param newRoll - входящее аналоговое значение для фильтрации
 * @return unsigned int - значение, отфильтрованное по медиане
 */
void filter_analog_data(uint32_t &newThrottle, uint32_t &newYaw, uint32_t &newPitch, uint32_t &newRoll) {
  
  static byte count_Throttle = 0;
  static byte count_Yaw = 0;
  static byte count_Pitch = 0;
  static byte count_Roll = 0;
  
  filter_Data.buffer_Throttle[count_Throttle] = newThrottle;
  filter_Data.buffer_Yaw[count_Yaw] = newYaw;
  filter_Data.buffer_Pitch[count_Pitch] = newPitch;
  filter_Data.buffer_Roll[count_Roll] = newRoll;

  filtered_data(filter_Data.buffer_Throttle, count_Throttle, NUM_READ);
  filtered_data(filter_Data.buffer_Yaw, count_Yaw, NUM_READ);
  filtered_data(filter_Data.buffer_Pitch, count_Pitch, NUM_READ);
  filtered_data(filter_Data.buffer_Roll, count_Roll, NUM_READ);

  if (++count_Throttle >= NUM_READ) count_Throttle = 0;
  if (++count_Yaw >= NUM_READ) count_Yaw = 0;
  if (++count_Roll >= NUM_READ) count_Roll = 0;
  if (++count_Pitch >= NUM_READ) count_Pitch = 0;
}



static unsigned int Throttle_analog_data = 0;  //"Сырое" аналоговое значение на пине Throttlte
static unsigned int Roll_analog_data = 0;      //"Сырое" аналоговое значение на пине Roll
static unsigned int Yaw_analog_data = 0;       //"Сырое" аналоговое значение на пине Yaw
static unsigned int Pitch_analog_data = 0;     //"Сырое" аналоговое значение на пине Pitch




/**
 * @brief Функция, считывающая текущие аналоговые значения для каналов
 * 
 * @param Throttlte - канал считывания (пин с которого считывается) аналоговое значение
 * @param Roll - канал считывания (пин с которого считывается) аналоговое значение
 * @param Yaw - канал считывания (пин с которого считывается) аналоговое значение
 * @param Pitch - канал считывания (пин с которого считывается) аналоговое значение
 * @param crsf - экземпляр класса CRSF
 */
void getAnalogData(const uint8_t &Throttlte, const uint8_t &Roll, const uint8_t &Yaw, const uint8_t  &Pitch, CRSF crsf)
{
  Throttle_analog_data = map(analogRead(Throttlte), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  Roll_analog_data = map(analogRead(Roll), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  Yaw_analog_data = map(analogRead(Yaw), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  Pitch_analog_data = map(analogRead(Pitch), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);

  filter_analog_data(Throttle_analog_data, Yaw_analog_data, Pitch_analog_data, Roll_analog_data);

  crsf.PackedRCdataOut.ch0 = filter_Data.buffer_Throttle[(uint32_t)NUM_READ / 2];
  crsf.PackedRCdataOut.ch1 = filter_Data.buffer_Pitch[(uint32_t)NUM_READ / 2];
  crsf.PackedRCdataOut.ch2 = filter_Data.buffer_Yaw[(uint32_t)NUM_READ / 2];
  crsf.PackedRCdataOut.ch3 = filter_Data.buffer_Roll[(uint32_t)NUM_READ / 2];
  crsf.PackedRCdataOut.ch4 = 0x00;
  crsf.PackedRCdataOut.ch5 = 0x00;

}




















#endif