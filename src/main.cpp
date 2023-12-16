#include <Arduino.h>

#include "CRSF_ESP32.h"
#include <driver\gpio.h>

//Директива для отладочной печати
#define DEBUG_PRINT


uint64_t CRSFinterval = 5000; //Значение счётчика, при котором будет сгенерировано прерывание в ms
bool uartCRSFinverted = false;

CRSF crsf;

hw_timer_t * timer = NULL;                              //таймер. указатель на переменную типа hw_timer_t 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   //переменная типа portMUX_TYPE, для обеспечения синхронизации между основным циклом и ISR при изменении общей переменной.

//переменная-счетчик будет совместно использоваться основным циклом и ISR,
//ее необходимо объявить с ключевым словом volatile , что позволит избежать ее удаления из-за оптимизации компилятора.
volatile int interruptCounter;


const uint8_t Throttlte = GPIO_NUM_34;    //Пин Throttle
const uint8_t Roll = GPIO_NUM_33;         //Пин Roll
const uint8_t Yaw = GPIO_NUM_32;          //Пин Yaw
const uint8_t Pitch = GPIO_NUM_35;        //Пин Pitch


// const uint8_t Throttlte = GPIO_NUM_34;    //Пин Throttle
// const uint8_t Roll = GPIO_NUM_33;         //Пин Roll
// const uint8_t Yaw = GPIO_NUM_32;          //Пин Yaw
// const uint8_t Pitch = GPIO_NUM_35;        //Пин Pitch


/**
 * Функция обработчик прерывания по таймеру
*/
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  crsf.sendRCFrameToFC();
  portEXIT_CRITICAL_ISR(&timerMux);

}





/**
 * Функция инициализации таймера.
 * 
 * - timer_number - номер таймера (в ESP32 до 4-х таймеров есть, т.е. номер у нас от 0 до 3)
 * 
 * - prescaler - Предделитель - между 0 и 65 535. Поскольку предварительный делитель имеет 16 бит, он может делить частоту
 *               тактового сигнала на коэффициент от 2 до 65536 [2], что дает большую свободу настройки. обычно частота
 *               базового сигнала, используемого счетчиками ESP32, составляет 80 МГц. Это значение равно 80 000 000 Гц, что означает,
 *               что сигнал заставит счетчик таймера увеличиваться 80 000 000 раз в секунду. Хотя мы могли бы произвести
 *               вычисления с этим значением, чтобы установить номер счетчика для генерации прерывания, мы воспользуемся
 *               преимуществом предварительного делителя, чтобы упростить его. Таким образом, если мы разделим это значение
 *               на 80 (используя 80 в качестве значения прескалера), мы получим сигнал с частотой 1 МГц, который будет
 *               увеличивать счетчик таймера на 1 000 000 раз в секунду (счетчик будет изменяться каждую микросекунду)
 * 
 * - flag - флаг, указывающий, должен ли счетчик считать вверх (true) или вниз (false).
*/
void StartTimer(uint8_t timer_number, uint16_t prescaler, bool flag) {
  // инициализируем наш таймер вызовом функции timerBegin  , которая вернет указатель на структуру типа hw_timer_t,
  // которая является одной из глобальных переменных таймера.
  timer = timerBegin(timer_number, prescaler, flag);

  // перед включением таймера нам нужно привязать его к функции-обработчику, которая будет выполняться при генерации прерывания
  // Эта функция получает в качестве входных данных указатель на инициализированный таймер, который мы сохранили в нашей
  // глобальной переменной, адрес функции, которая будет обрабатывать прерывание, и флаг, указывающий, является ли
  // генерируемое прерывание фронтом (истина) или уровнем (ложь)
  timerAttachInterrupt(timer, &onTimer, true);

  // Далее мы воспользуемся функцией timerAlarmWrite , чтобы указать значение счетчика, при котором будет сгенерировано прерывание таймера.
  // Таким образом, эта функция получает в качестве первого входа указатель на таймер, в качестве второго значение счетчика,
  // в котором должно быть сгенерировано прерывание, и в качестве третьего флага, указывающего, должен ли таймер автоматически перезагружаться
  // при генерации прерывания. Итак, в качестве первого аргумента мы снова передаем нашу глобальную переменную таймера, а в качестве
  // третьего аргумента мы передаем true , поэтому счетчик будет перезагружаться и, таким образом, будет периодически генерироваться
  // прерывание. Что касается второго аргумента, помните, что мы устанавливаем прескалер для того, чтобы это означало количество
  // микросекунд, через которое должно произойти прерывание. Итак, для этого примера мы предполагаем, что хотим генерировать прерывание
  // каждую секунду, и поэтому мы передаем значение 1 000 000 микросекунд, что равно 1 секунде. Важно : примите во внимание, что это
  // значение указывается в микросекундах, только если мы указываем значение 80 для предварительного масштабирования. Мы можем
  // использовать разные значения предделителя, и в этом случае нам нужно выполнить вычисления, чтобы узнать, когда счетчик достигнет
  // определенного значения.
  timerAlarmWrite(timer, CRSFinterval, true);

  // Мы заканчиваем нашу функцию настройки, включив таймер вызовом функции timerAlarmEnable , передав в качестве
  // входных данных нашу переменную таймера.
  timerAlarmEnable(timer);
}




















const uint8_t NUM_READ = 10;  // порядок медианы


/**
 * @brief Структура для хранения буферов аналоговых сигналов для фильтрации
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
 * @param newThrottle - аналоговое значение для фильтрации
 * @param newYaw - аналоговое значение для фильтрации
 * @param newPitch - аналоговое значение для фильтрации
 * @param newRoll - аналоговое значение для фильтрации
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

  // if ((count < NUM_READ - 1) and (filter_Data.buffer_Throttle[count] > filter_Data.buffer_Throttle[count + 1])) {
  //   for (int i = count; i < NUM_READ - 1; i++) {
  //     if (filter_Data.buffer_Throttle[i] > filter_Data.buffer_Throttle[i + 1]) {
  //       uint32_t buff = filter_Data.buffer_Throttle[i];
  //       filter_Data.buffer_Throttle[i] = filter_Data.buffer_Throttle[i + 1];
  //       filter_Data.buffer_Throttle[i + 1] = buff;
  //     }
  //   }
  // } else {
  //   if ((count > 0) and (filter_Data.buffer_Throttle[count - 1] > filter_Data.buffer_Throttle[count])) {
  //     for (int i = count; i > 0; i--) {
  //       if (filter_Data.buffer_Throttle[i] < filter_Data.buffer_Throttle[i - 1]) {
  //         int buff = filter_Data.buffer_Throttle[i];
  //         filter_Data.buffer_Throttle[i] = filter_Data.buffer_Throttle[i - 1];
  //         filter_Data.buffer_Throttle[i - 1] = buff;
  //       }
  //     }
  //   }
  // }

  if (++count_Throttle >= NUM_READ) count_Throttle = 0;
  if (++count_Yaw >= NUM_READ) count_Yaw = 0;
  if (++count_Roll >= NUM_READ) count_Roll = 0;
  if (++count_Pitch >= NUM_READ) count_Pitch = 0;
  //filter_Data.buffer_Throttle[(uint32_t)NUM_READ / 2];
}









static unsigned int Throttle_analog_data = 0;  //"Сырое" аналоговое значение на пине Throttlte
static unsigned int Roll_analog_data = 0;      //"Сырое" аналоговое значение на пине Roll
static unsigned int Yaw_analog_data = 0;       //"Сырое" аналоговое значение на пине Yaw
static unsigned int Pitch_analog_data = 0;     //"Сырое" аналоговое значение на пине Pitch


/**
 * @brief Функция, считывающая текущие аналоговые значения для каналов
 * 
 */
void getAnalogData()
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


#ifdef DEBUG_PRINT

uint32_t Throttle_old = 0;
uint32_t Pitch_old = 0;
uint32_t Yaw_old = 0;
uint32_t Roll_old = 0;

/**
 * @brief Функция отладочной печати
 * 
 */
void print_debug()
{
  Serial.println("");
  Serial.println("Значения каналов:");
  Serial.println("");
  if(Throttle_old != crsf.PackedRCdataOut.ch0)
  {
    Serial.print("  - ");
    Serial.print("Throttle = ");
    Serial.println(crsf.PackedRCdataOut.ch0);
  }

  if(Pitch_old != crsf.PackedRCdataOut.ch1)
  {
    Serial.print("  - ");
    Serial.print("Pitch    = ");
    Serial.println(crsf.PackedRCdataOut.ch1);
  }

  if(Yaw_old != crsf.PackedRCdataOut.ch2)
  {
    Serial.print("  - ");
    Serial.print("Yaw      = ");
    Serial.println(crsf.PackedRCdataOut.ch2);
  }

  if(Roll_old != crsf.PackedRCdataOut.ch3)
  {
    Serial.print("  - ");
    Serial.print("Roll     = ");
    Serial.println(crsf.PackedRCdataOut.ch3);
  }
  Serial.println("");
  
  Throttle_old = crsf.PackedRCdataOut.ch0;
  Pitch_old = crsf.PackedRCdataOut.ch1;
  Yaw_old = crsf.PackedRCdataOut.ch2;
  Roll_old = crsf.PackedRCdataOut.ch3;

  delay(300);
}
#endif





void setup() {
  #ifdef DEBUG_PRINT
    Serial.begin(9600);
  #endif
  
  crsf.Begin();   //Инициализируем порт протокола передачи CRSF

  //Определяем пин 0 для кнопки
  //gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  //gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);

  //pinMode(ButtonPin, INPUT_PULLUP);
  //gpio_set_direction(ButtonPin, GPIO_MODE_INPUT); 
  //gpio_set_pull_mode(ButtonPin, GPIO_PULLUP_ONLY);

  pinMode(Throttlte, INPUT);
  pinMode(Roll, INPUT);
  pinMode(Yaw, INPUT);
  pinMode(Pitch, INPUT);
  
  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
  
  #ifdef DEBUG_PRINT
    crsf.PackedRCdataOut.ch4 = 0x00;
    crsf.PackedRCdataOut.ch5 = 0x00;
    crsf.PackedRCdataOut.ch6 = 0x00;
    crsf.PackedRCdataOut.ch7 = 0x00;
    crsf.PackedRCdataOut.ch8 = 0x00;
    crsf.PackedRCdataOut.ch9 = 0x00;
    crsf.PackedRCdataOut.ch10 = 0x00;
    crsf.PackedRCdataOut.ch11 = 0x00;
    crsf.PackedRCdataOut.ch12 = 0x00;
    crsf.PackedRCdataOut.ch13 = 0x00;
    crsf.PackedRCdataOut.ch14 = 0x00;
    crsf.PackedRCdataOut.ch15 = 0x00;
  #else 
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch5 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch6 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch7 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch8 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch9 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch10 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch11 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch12 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch13 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch14 = CRSF_CHANNEL_VALUE_MIN;
    crsf.PackedRCdataOut.ch15 = CRSF_CHANNEL_VALUE_MIN;

  #endif
  

  crsf.sendRCFrameToFC();

  StartTimer(0, 80, true);

  //Отладочная печать
  #ifdef DEBUG_PRINT
    print_debug();
  #endif
  
}







void loop() {
  // put your main code here, to run repeatedly:
  delay(1);

  getAnalogData();
   
  //Отладочная печать
  #ifdef DEBUG_PRINT
    print_debug();
  #endif

  
  
}