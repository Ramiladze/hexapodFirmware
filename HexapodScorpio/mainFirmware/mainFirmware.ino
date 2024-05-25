#include <PS2X_lib.h>  // документация: http://www.billporter.info/
#include <Servo.h>
#include <math.h>

const int PS2_DAT = 2;  // пины геймпада
const int PS2_ATT = 3;
const int PS2_CMD = 4;
const int PS2_CLK = 5;
const int RUMBLE = true;
const int PRESSURES = false;
const int COXA1_SERVO = 19;  // пины серв
const int FEMUR1_SERVO = 21;
const int TIBIA1_SERVO = 23;
const int COXA2_SERVO = 25;
const int FEMUR2_SERVO = 27;
const int TIBIA2_SERVO = 29;
const int COXA3_SERVO = 31;
const int FEMUR3_SERVO = 33;
const int TIBIA3_SERVO = 35;
const int COXA4_SERVO = 37;
const int FEMUR4_SERVO = 39;
const int TIBIA4_SERVO = 41;
const int COXA5_SERVO = 43;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 47;
const int COXA6_SERVO = 49;
const int FEMUR6_SERVO = 51;
const int TIBIA6_SERVO = 53;
const int COXA_LENGTH = 46;  // длины частей ног в мм. coxa - 1; femur - 2; tibia - 3.
const int FEMUR_LENGTH = 43;
const int TIBIA_LENGTH = 90;
const int TRAVEL = 30;                                           // постоянная предельная величина перемещения при перемещении и повороте
const long A12DEG = 209440;                                      // 12 градусов в радианах x 1,000,000
const long A30DEG = 523599;                                      // 30 градусов в радианах x 1,000,000
const int FRAME_TIME_MS = 20;                                    // фрем тайм (20msec = 50Hz)
const float HOME_X[6] = { 63.0, 0.0, -63.0, -63.0, 0.0, 63.0 };  // координаты наконечника ноги, когда оси вращения второй и третей сервы находятся на одной линии, а наконечник ноги находится точно под осью вращения третей сервы. Отчет координат с оси вращения первой сервы(coxa)
const float HOME_Y[6] = { 63.0, 89.0, 63.0, -63.0, -89.0, -63.0 };
const float HOME_Z[6] = { -70.0, -70.0, -70.0, -70.0, -70.0, -70.0 };
const float BODY_X[6] = { 66.0, 0.0, -66.0, -66.0, 0.0, 66.0 };  // координаты осей вращения первой части ноги(coxa)(точки крепления ног к базе). Отчет координат с центра базы
const float BODY_Y[6] = { 49.5, 61.6, 49.5, -49.5, -61.6, -49.5 };
const float BODY_Z[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const int COXA_CAL[6] = { 7, -5, 0, 6, -5, -10 };  // калибровочные константы. Шлицы на сервах не позволяют точно их выставить, поэтому нужны эти константы.
const int FEMUR_CAL[6] = { 0, 0, 0, 0, 0, 0 };
const int TIBIA_CAL[6] = { -5, -6, 0, 0, 3, 2 };

// Переменные
int gamepad_error;  // переменные геймпада
byte gamepad_type;
byte gamepad_vibrate;
unsigned long currentTime;  // переменные фрейм таймера
unsigned long previousTime;
// переменная режима и управляющие переменные
int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;
float L0, L3;  // переменные для обратной кинематики
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;
int leg_num;  // переменные позиционирования и ходьбы
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];
int tripod_case[6] = { 1, 2, 1, 2, 1, 2 };  // массив для треногой ходьбы
int ripple_case[6] = { 2, 6, 4, 1, 3, 5 };  // for ripple gait

// Объекты
PS2X ps2x;          // PS2 геймпад
Servo coxa1_servo;  // 18 серв
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

void setup() {
  // запуск последовательного порта
  Serial.begin(115200);
  // инит серв
  coxa1_servo.attach(COXA1_SERVO, 610, 2400);
  femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
  tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
  coxa2_servo.attach(COXA2_SERVO, 610, 2400);
  femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
  tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
  coxa3_servo.attach(COXA3_SERVO, 610, 2400);
  femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
  tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);
  coxa4_servo.attach(COXA4_SERVO, 610, 2400);
  femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
  tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
  coxa5_servo.attach(COXA5_SERVO, 610, 2400);
  femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
  tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
  coxa6_servo.attach(COXA6_SERVO, 610, 2400);
  femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
  tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);
  // подключение к геймпаду
  gamepad_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, PRESSURES, RUMBLE);
  if (gamepad_error == 0) Serial.println("Controller attached");
  else if (gamepad_error == 1) Serial.println("No controller found");
  else if (gamepad_error == 2) Serial.println("Controller found but not accepting commands");
  else if (gamepad_error == 3) Serial.println("Controller refusing to enter Pressures mode");
  // проверка типа геймпада
  gamepad_type = ps2x.readType();
  if (gamepad_type == 0) Serial.println("Unknown Controller type found");
  else if (gamepad_type == 1) Serial.println("DualShock Controller found");
  else if (gamepad_type == 2) Serial.println("GuitarHero Controller found");
  else if (gamepad_type == 3) Serial.println("Wireless Sony DualShock Controller found");
  // выкл вибро
  gamepad_vibrate = 0;
  // очистка смещений
  for (leg_num = 0; leg_num < 6; leg_num++) {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.4;
  // инит переменной режима и переменных ходьбы
  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
}

// Главный цикл
void loop() {
  // выход если контроллер не найден
  if ((gamepad_error == 1) || (gamepad_type == 2)) {
    Serial.println("Invalid Controller!");
    return;
  }
  // установка фрейм тайма
  currentTime = millis();
  if ((currentTime - previousTime) > FRAME_TIME_MS) {
    previousTime = currentTime;
    // считывание входных данных контроллера
    ps2x.read_gamepad(false, gamepad_vibrate);
    process_gamepad();
    // reset legs to home position when commanded
    if (reset_position == true) {
      for (leg_num = 0; leg_num < 6; leg_num++) {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }
    // расположите ножки, используя расчеты IK, если только не установлен режим "все на 90 градусов"
    if (mode < 99) {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }
    // режимы (режим 0 по умолчанию - "домашний режим ожидания", режим бездействия)
    if (mode == 1)  // режим ходьбы
    {
      if (gait == 0) tripod_gait();  // ходьба с использованием треногой походки
    }
    if (mode == 99) set_all_90();
  }
}

/** *********************************************************************** **
 ** Process gamepad controller inputs **
 ** *********************************************************************** **/
void process_gamepad() {
  if (ps2x.ButtonPressed(PSB_PAD_DOWN))  // остановка и выбор режима ходьбы 0
  {
    mode = 0;
    gait = 0;
    reset_position = true;
  }
  if (ps2x.ButtonPressed(PSB_TRIANGLE))  // установка режима ходьбы
  {
    mode = 1;
    reset_position = true;
  }
  if (ps2x.Button(PSB_TRIANGLE))  // вибрируем контроллер, если удерживать кнопку ходьбы
    gamepad_vibrate = 64;
  else
    gamepad_vibrate = 0;
  if (ps2x.ButtonPressed(PSB_START))  // изменение режима скорости ходьбы
  {
    if (gait_speed == 0)
      gait_speed = 1;
    else
      gait_speed = 0;
  }
  if (ps2x.ButtonPressed(PSB_SELECT))  // сервоприводы на 90 градусов для калибровки
  {
    mode = 99;
  }
  if ((ps2x.ButtonPressed(PSB_L1)) || (ps2x.ButtonPressed(PSB_R1)) && (mode != 2) && (mode != 3)) {  // захват оффсетов с режимов транслейта и ротейта
    capture_offsets = true;
  }
  if ((ps2x.ButtonPressed(PSB_L2)) || (ps2x.ButtonPressed(PSB_R2))) {  // осчитска оффсетов
    for (leg_num = 0; leg_num < 6; leg_num++) {
      offset_X[leg_num] = 0;
      offset_Y[leg_num] = 0;
      offset_Z[leg_num] = 0;
    }
    step_height_multiplier = 1.4;  // сброс множителя высоты шага
  }
}

/** *********************************************************************** **
 ** Inverse kinematics for leg **
 ** *********************************************************************** **/
void leg_IK(int leg_number, float X, float Y, float Z) {
  // вычисление желаемой длины бедра-ступни (L3)
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));
  // обработка только в случае, если длина достижима (не слишком длинная или слишком короткая!)
  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH))) {
    // вычисление tibia
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);
    // вычисление femur
    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);
    // вычисление coxa
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];
    // управление соответствующей ногой
    switch (leg_number) {
      case 0:
        theta_coxa = theta_coxa + 45.0;  // компенсация за крепление ноги
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa1_servo.write(int(theta_coxa));
        femur1_servo.write(int(theta_femur));
        tibia1_servo.write(int(theta_tibia));
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;  // компенсация за крепление ноги
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa2_servo.write(int(theta_coxa));
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0;  // компенсация за крепление ноги
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa3_servo.write(int(theta_coxa));
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;
      case 3:
        if (theta_coxa < 0)  // компенсация за крепление ноги
          theta_coxa = theta_coxa + 225.0;
        else
          theta_coxa = theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa4_servo.write(int(theta_coxa));
        femur4_servo.write(180 - int(theta_femur));
        tibia4_servo.write(180 - int(theta_tibia));
        break;
      case 4:
        if (theta_coxa < 0)  // компенсация за крепление ноги
          theta_coxa = theta_coxa + 270.0;
        else
          theta_coxa = theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa5_servo.write(int(theta_coxa));
        femur5_servo.write(180 - int(theta_femur));
        tibia5_servo.write(180 - int(theta_tibia));
        break;
      case 5:
        if (theta_coxa < 0)  // компенсация за крепление ноги
          theta_coxa = theta_coxa + 315.0;
        else
          theta_coxa = theta_coxa - 45.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa6_servo.write(int(theta_coxa));
        femur6_servo.write(180 - int(theta_femur));
        tibia6_servo.write(180 - int(theta_tibia));
        break;
    }
  }
}

void tripod_gait() {
  // Чтение команд с геймпада
  commandedX = map(ps2x.Analog(PSS_RY), 0, 255, 127, -127);
  commandedY = map(ps2x.Analog(PSS_RX), 0, 255, -127, 127);
  commandedR = map(ps2x.Analog(PSS_LX), 0, 255, 127, -127);
  // Проверка активности
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0)) {
    // Вычисление параметров движения
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0);  // количество шагов
    // Цикл по ногам
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      if (tripod_case[leg_num] == 1) {
        // Группа 1: Движение вперёд (подъём и опускание)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        // Переход в группу 2 после завершения движения вперёд
        if (tick >= numTicks - 1) {
          tripod_case[leg_num] = 2;
        }
      } else {
        // Группа 2: Движение назад (на земле)
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        // Переход в группу 1 после завершения движения назад
        if (tick >= numTicks - 1) {
          tripod_case[leg_num] = 1;
        }
      }
    }
    // Увеличение счетчика шагов
    tick = (tick < numTicks - 1) ? tick + 1 : 0;
  }
}

// Вычисляем длину шага при ходьбе
void compute_strides() {
  // вычисляем длину шага
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;
  // вычисление триггера вращения
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));
  // установка скорости
  if (gait_speed == 0) duration = 960;  // (960, 840, 720, 600, 480)
  else duration = 3240;
}

// Вычисляем амплитуду ходьбы
void compute_amplitudes() {
  // вычисляем общее расстояние от центра тела до пальцев ног
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
  // вычисляем смещение вращения
  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;
  // вычисляем амплитуду X и Y и ограничиваем, чтобы ноги не врезались друг в друга
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50);
  amplitudeY = constrain(amplitudeY, -50, 50);
  // вычисляем амплитуду Z
  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

// Устанавливаем все сервоприводы на 90 градусов
// Примечание: это полезно для калибровки/выравнивания сервоприводов
// то есть: установить значения COXA_CAL[6], FEMUR_CAL[6] и TIBIA_CAL[6] в
// раздел констант выше, поэтому все углы отображаются как 90 градусов
void set_all_90() {
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);
}
