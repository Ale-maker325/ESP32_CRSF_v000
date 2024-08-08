#include <Arduino.h>
#include <CRSF_ESP32.h>
#include <driver\gpio.h>
#include <esp_log.h>
#include <filterAnalogSignal.h>
#include <OneButton.h>
#include <LED_SIGNALS.h>

//Директива для отладочной печати - раскомментировать если нужно, но тогда всё будет сильно тормозить
#define DEBUG_PRINT
//Директива (пока что временно!) переключения в режим работы с сериал и кнопкой
#define BUTTON_MODE


static const char *TAG_SYS = "SYS: ";


uint64_t CRSFinterval = 5000; //Значение счётчика, при котором будет сгенерировано прерывание в ms
bool uartCRSFinverted = false;

CRSF crsf;                    //Клас протокола CRSF
OneButton button;             //Класс упраления кнопкой
LED_SIGNAL led_StartCRSF;     //Светодиод, отвечающий за индикацию включения протокола CRSF
LED_SIGNAL led_StopCRSF;      //Светодиод, отвечающий за индикацию выключения протокола CRSF


portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   //переменная типа portMUX_TYPE, для обеспечения синхронизации между основным циклом и ISR при изменении общей переменной.

//переменная-счетчик будет совместно использоваться основным циклом и ISR,
//ее необходимо объявить с ключевым словом volatile , что позволит избежать ее удаления из-за оптимизации компилятора.
volatile int interruptCounter;


const uint8_t Throttlte = GPIO_NUM_34;    //Пин Throttle
const uint8_t Roll =      GPIO_NUM_33;    //Пин Roll
const uint8_t Yaw =       GPIO_NUM_32;    //Пин Yaw
const uint8_t Pitch =     GPIO_NUM_35;    //Пин Pitch

const uint8_t ButtonPin = GPIO_NUM_0;     //Пин кнопки

const uint8_t LED_CRSF_START = GPIO_NUM_25;   //Светодиод сигнализатор работы протокола CRSF 
const uint8_t LED_CRSF_STOP  = GPIO_NUM_26;   //Светодиод сигнализатор остановки работы протокола CRSF





/**
 * Функция обработчик прерывания по таймеру, вызывающий с заданной периодичностью функцию отправки
 * пакетов в сериал-порт
*/
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  crsf.sendFrameToFC();
  portEXIT_CRITICAL_ISR(&timerMux);

}

hw_timer_t * timer = NULL;                              //таймер. указатель на переменную типа hw_timer_t 

/**
 * Функция инициализации таймера для периодичной отправки в порт пакетов
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
  //timerAlarmEnable(timer); //- включается кнопкой в прерывании от кнопки
  timerAlarmDisable(timer); //- временно отключим таймер
}


void firstInitAnalogDataForChannels()
{
  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
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
}






volatile static bool flagButtonOnePressed = false;
volatile static uint8_t typePacket = CRSF_FRAMETYPE_DEVICE_PING;
volatile static bool flagButtonTwoPressed = false;

/**
 * @brief Функция для обработчика прерываний по нажатию
 * кнопки. Включает генерацию протокола CRSF в сериал порт
 * по прерываниям таймера.
 */
void IRAM_ATTR clickButton()
{
  if(flagButtonOnePressed != true)
  {
    flagButtonOnePressed = true;
    flagButtonTwoPressed = false;
    #ifdef DEBUG_PRINT
      log_e("Короткое нажатие кнопки - генерация CRSF");
    #endif
    led_StartCRSF.ledON();
    led_StopCRSF.ledOFF();
    timerAlarmEnable(timer);  //Включаем генерацию протокола по таймеру
  }
}

/**
 * @brief 
 * 
 */
void IRAM_ATTR LongPressButton()
{
  if(flagButtonTwoPressed != true)
  {
    #ifdef DEBUG_PRINT
      log_e("Долгое нажатие - стоп генерации CRSF");
    #endif
    flagButtonOnePressed = false;
    flagButtonTwoPressed = true;
    timerAlarmDisable(timer); //Отключаем генерацию протокола по таймеру
    led_StartCRSF.ledOFF();
    led_StopCRSF.ledON();
  }
}







void setup() {
  Serial.begin(9600);
  
  crsf.Begin();   //Инициализируем порт протокола передачи CRSF
  led_StartCRSF.initLed(LED_CRSF_START, false);
  led_StopCRSF.initLed(LED_CRSF_STOP, true);


  /**
   * @brief Инициируем кнопку с подключением к пину 0. Кнопка срабатывает при подтяжке
   * её к земле. Назначаем одно нажатие - для генерации протокола CRSF. 
   * 
   */
  button.setup(ButtonPin, INPUT_PULLUP, true);
  
  #ifdef DEBUG_PRINT
    log_e("Назначаем кнопку на пин %u ", ButtonPin);
  #endif
  
  button.attachClick(clickButton);
  button.attachLongPressStart(LongPressButton);
  
  pinMode(Throttlte, INPUT);
  pinMode(Roll, INPUT);
  pinMode(Yaw, INPUT);
  pinMode(Pitch, INPUT);
    
  firstInitAnalogDataForChannels();
  crsf.sendFrameToFC();
  StartTimer(0, 80, true);

 
}







void loop()
{
  getAnalogData(Throttlte, Roll, Yaw, Pitch, crsf);
  button.tick();
  
  if(flagButtonOnePressed)
  {
    
  //   crsf.readFromSerial();
  }
  
  if(flagButtonTwoPressed)
  {
    
    // crsf.sendExtendedPacket(typePacket);
      
    
  }


    
}