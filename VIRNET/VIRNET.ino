// float арифметика на arduino медленная, делать на int
#include "Queue.h"
#include "EEPROM.h"

int sensor_pins[8] = {A7, A6, A5, A4, A3, A2, A1, A0}; // массив, храняший номера портов датчиков линии
int sensors[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // массив значений датчиков линии (от 0 до 7)

int sensor_koef[8] = {-4, -3, -2, -1, 1, 2, 3, 4}; // коэффициенты положений датчиков
int sensor_white_values[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; // максимальное значений датчиков на белом
int sensor_black_values[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //  максимальное значение датчиков на черном

long iteration = 0; // число повторений основного цикла программы

int avg_speed = 120; // средняя скорость моторов
int extra_speed = 0; // дополнительная скорость на прямых участках (не изменять !)
float kP = 0.030; // коэффициент пропорциональной обратной связи
float kD = 0.07;//0.16; // коэффициент дифференциальной обратной связи
float kI = 0.05;//0.08; // коэффициент интегральной обратной связи

int P; // управляющее воздействие пропорциональной части
int D; // управляющее воздействие дифференциальной части
int I; // управляющее воздействие интегральной части

int correction; // управляющее воздействие
int err; // переменная, хранящая текущее значение ошибки

int err_arr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // массив предыдущих ошибок для Д-регулятора
int err_p = -1; // индекс в массиве ошибок для Д-регулятора

int speed = 0; //  конечная скорость робота скорость движения робота (не изменять!)

bool break_state = false; // останавливает двигатели при значении { true }

//Queue<int> errors(10); //пока не используется
int errors[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // массив предыдущих ошибок для коррекции скорости
int delta_errors = 90; // число тактов между считыванием значения ошибки в {errors}


void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(10, INPUT); // пин геркона
  sensorsInit();
  delay(3000);
  //EEPROM.write(0, 210);
}

void loop() {
  getSensorValues();
  //if (iteration % 100 == 0) {
  //  printIteration();
  //  Serial.println(EEPROM.read(0));
  //}
  //delay(300);
  // ************************PID**********************
  err = botPosition();
  err_p = (err_p + 1) % 10; // переставляет индекс массива ошибок на 1 вправо
  err_arr[err_p] = err; // записывает текущее значение ошибки в массив ошибок

  P = err * kP; // расчет пропорционального коэффициент
  D = (err_arr[err_p] - err_arr[(err_p+11) % 10])*kD; // расчет дифференциального коэффициента
  int err_sum = 0;
  for (int i = 0; i < 10; i++) 
    err_sum += err_arr[i];
  I = err_sum/10*kI;

  correction = P + I + D; // суммирование ошибок 3 составляющих регулятора

  speedControl(err);

  setMotorSpeed(false, avg_speed + correction); // добавить speedControl
  setMotorSpeed(true, avg_speed - correction);
  //digitalWrite(7, iteration % 2);
 
  if (break_state == true) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  monotor();
  delayMicroseconds(90);
  iteration++;
  
}

void setMotorSpeed(bool side, int speed) { // функция настройки и управления моторами
  int left_PWR_pin = 5; // пин для установки скорости левого мотора
  int right_PWR_pin = 9; // пин для установки скорсти правого мотора
  int left_turn_pin = 5; // пин для установки вращения левого мотора 
  int right_turn_pin = 8; // пин для установки вращения правого мотора

  if (break_state == false) { // если не включен тормоз
    if (side == false) {
      if (speed >= 0) {
        digitalWrite(left_turn_pin, HIGH);
      } 
      else {
        digitalWrite(left_turn_pin, LOW);
        speed = -speed;
      }
      if (speed > 255) speed = 255;
      analogWrite(left_PWR_pin, speed); 
    }
    else {
      if (speed >= 0) {
        digitalWrite(right_turn_pin, HIGH);
      } 
      else {
        digitalWrite(right_turn_pin, LOW);
        speed = -speed;
      }
      if (speed > 255) speed = 255;
      analogWrite(right_PWR_pin, speed); 
    }
  } else {
    digitalWrite(left_PWR_pin, 0);
    digitalWrite(right_PWR_pin, 0);
  }
  
}

void monotor() { // вывод информации в Serial Port
  if (iteration % 2500 == 0) {
    printIteration();
    Serial.print("kP: ");
    Serial.print(kP);
    Serial.print(" | kD: ");
    Serial.print(kD);
    Serial.print(" | kI: ");
    Serial.println(kI);
    printIteration();
    Serial.println("S1  S2  S3  S4  S5  S6     P  D  I Cor");
    printIteration();
    Serial.println("--------------------------------------");
  }
  if (iteration % 250 == 0) {
    printIteration();
    for(int i = 0; i < 6; i++) {
      if (sensors[i] < 10 && sensors[i] > -10) Serial.print(" ");
      Serial.print(sensors[i] );
      Serial.print("  ");
    }
    if (P < 10) Serial.print(" ");
    Serial.print(P);
    Serial.print(" ");
    Serial.print(D);
    Serial.print(" ");
    Serial.print(I);
    Serial.println();
    Serial.println();
  }
}

void speedControl(int e) { // добавляет скорость роботу на прямых участках
  if (iteration % delta_errors == 0) {
    int errors_sum = 0;
    for (int i = 0; i < 9; i++) {
      errors[i] = errors[i+1];
      errors_sum += errors[i];
    }
    errors[9] = e; 
    errors_sum += e; // обновление массива прошлых ошибок
    extra_speed = 0.03*(500 - errors_sum);
  }
  
  Serial.println(extra_speed);
  extra_speed = abs(extra_speed);
  if (extra_speed < 30) {
    speed = avg_speed + extra_speed;
  } else {
    speed = avg_speed;
  }
}

void getSensorValues() { // получает значения датчиков и нормализует их 
  for (int i = 0; i < 8; i++) {
    sensors[i] = map(analogRead(sensor_pins[i]), sensor_white_values[i], sensor_black_values[i], 0, 100);
  }
}

void sensorsCalib() { // функция калибровки датчиков
  delay(1000);
  setMotorSpeed(false, 100);
  setMotorSpeed(true, -100);
  for (int j = 0; j < 700; j++) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensor_pins[i]) > sensor_black_values[i]) sensor_black_values[i] = analogRead(sensor_pins[i]);
      if (analogRead(sensor_pins[i]) < sensor_white_values[i]) sensor_white_values[i] = analogRead(sensor_pins[i]);
    }
    delay(2);
  }
  setMotorSpeed(false, 0);
  setMotorSpeed(true, 0);
  delay(1000);
}

void PID(int e) { // ПИД-регулятор
  //
}

int botPosition() { // вычисляет текущее положение робота на линиии
  int position = 0; 
  for (int i = 0; i < 8; i++) {
    position += sensor_koef[i]*(100 - sensors[i]);
  }
  return position;
}

void printIteration() { // выводит текущую итерацию основного цикла
  if (iteration < 100) Serial.print(" ");
  if (iteration < 1000) Serial.print(" ");
  if (iteration < 10000) Serial.print(" ");
  Serial.print(iteration);
  Serial.print(" -> ");
}

void sensorsInit() { // инициализирует порты для датчиков
  for (int i = 0; i < 8; i++) {
    pinMode(sensor_pins[i], INPUT);
  }
}