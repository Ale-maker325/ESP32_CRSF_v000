#include <Arduino.h>

#include "CRSF_ESP32.h"
#include <driver\gpio.h>


uint64_t CRSFinterval = 5000; //Значение счётчика, при котором будет сгенерировано прерывание в ms
bool uartCRSFinverted = false;

CRSF crsf;

hw_timer_t * timer = NULL;                              //таймер. указатель на переменную типа hw_timer_t 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   //переменная типа portMUX_TYPE, для обеспечения синхронизации между основным циклом и ISR при изменении общей переменной.

//переменная-счетчик будет совместно использоваться основным циклом и ISR,
//ее необходимо объявить с ключевым словом volatile , что позволит избежать ее удаления из-за оптимизации компилятора.
volatile int interruptCounter;






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




uint8_t ButtonPin = GPIO_NUM_4;
uint8_t VrxPin = GPIO_NUM_34;
uint8_t VryPin = GPIO_NUM_32;




void setup() {
  Serial.begin(9600);
  
  crsf.Begin();   //Инициализируем порт протокола передачи CRSF

  //Определяем пин 0 для кнопки
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);

  //Определяем пины для аналогового чтения и для кнопки джойстика
  
  // пин кнопки SW
  pinMode(ButtonPin, INPUT_PULLUP);
  //gpio_set_direction(ButtonPin, GPIO_MODE_INPUT); 
  //gpio_set_pull_mode(ButtonPin, GPIO_PULLUP_ONLY);

  // пин аналоговый Vry
  pinMode(VryPin, INPUT);
  //gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT); 
  //gpio_set_pull_mode(GPIO_NUM_32, GPIO_PULLUP_ONLY);

  // пин аналоговый Vrx
  pinMode(VrxPin, INPUT);
  //gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT); 
  //gpio_set_pull_mode(GPIO_NUM_34, GPIO_PULLUP_ONLY);
  
  //pinMode(0, INPUT_PULLUP);

  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
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

  crsf.sendRCFrameToFC();

  StartTimer(0, 80, true);

  
}





#define NUM_READ 10  // порядок медианы

// медиана на N значений со своим буфером, ускоренный вариант
int find_X_analogData(int newVal) {
  static int buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        int buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          int buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}


// медиана на N значений со своим буфером, ускоренный вариант
int find_Y_analogData(int newVal) {
  static int buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        int buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          int buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}





int X_pin = 0;
int Y_pin = 0;
int X_pin_old = 0;
int Y_pin_old = 0;
int Button_state = 0;


void loop() {
  // put your main code here, to run repeatedly:
  delay(1);

  X_pin = map(analogRead(VrxPin), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  X_pin = find_X_analogData(X_pin);

  Y_pin = map(analogRead(VryPin), 0, 4095, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  Y_pin = find_Y_analogData(Y_pin);
  //X_pin = map(analogRead(VrxPin), 0, 4095, 0, 100);
  //Y_pin = map(analogRead(VryPin), 0, 4095, 0, 100);
   
  if(digitalRead(ButtonPin) != 0)
  {
    Button_state = false;
    Serial.print("Кнопка ненажата");
    Serial.println(" ");
  }else{
    Button_state = true;
    Serial.print("Кнопка нажата!!!!");
    Serial.println(" ");
  }

    
  if(X_pin != X_pin_old){
    Serial.print("По оси Х = ");
    Serial.println(X_pin);
    Serial.println(" ");
  }

  if(Y_pin != Y_pin_old){
    Serial.print("По оси Y = ");
    Serial.println(Y_pin);
    Serial.println(" ");
  }
  
  if (Button_state) {
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MAX;
  } else {

    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
  }

  crsf.PackedRCdataOut.ch0 = X_pin;
  crsf.PackedRCdataOut.ch1 = Y_pin;
  // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  // crsf.PackedRCdataOut.ch3 = random(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);


  X_pin_old = X_pin;
  Y_pin_old = Y_pin;

}