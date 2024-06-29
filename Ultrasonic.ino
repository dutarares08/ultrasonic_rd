#include <Wire.h>  // Include librăria Wire pentru comunicarea I2C
#include "HC_SR04.h"

// Definirea pinilor pentru LED-uri și buzzer
#define LED_RED         A2
#define LED_YELLOW      A1
#define BUZZER_PIN      A3      

// Pinii pentru senzori ultrasonici
#define TRIG_PIN_1      7   // Pin pentru Trig senzor ultrasonic 1
#define ECHO_PIN_1      8   // Pin pentru Echo senzor ultrasonic 1
#define TRIG_PIN_2      9   // Pin pentru Trig senzor ultrasonic 2
#define ECHO_PIN_2      10  // Pin pentru Echo senzor ultrasonic 2
#define ECHO_INT 0

// Adresa dispozitivului gyro
int gyro_address = 0x68;
int acc_calibration_value = -287; // Valoarea de calibrare a accelerometrului

// Setările pentru PID
float pid_kp = 15;   // Câștigul P-controller
float pid_ki = 0.8;  // Câștigul I-controller
float pid_kd = 5.87; // Câștigul D-controller

// Setările pentru viteză
float turning_speed = 25;     // Viteza de rotire
float max_target_speed = 100; // Viteza maximă țintă

// Variabile globale

int cnt;
byte start;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

unsigned long loop_timer;


HC_SR04 ultrasonic(TRIG_PIN_1, ECHO_PIN_1, ECHO_INT);
int front_sensor_cm;

void setup()
{
    Serial.begin(9600); // Pornește portul serial la 9600 kbps

    Wire.begin();       // Pornește magistrala I2C ca master
    TWBR = 12;          // Setează viteza ceasului I2C la 400kHz

    // Configurarea temporizatorului pentru controlul motorului
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 |= (1 << OCIE2A);
    TCCR2B |= (1 << CS21);
    OCR2A = 39;
    TCCR2A |= (1 << WGM21);

    // Trezirea MPU-6050 și configurarea lui
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    // Configurarea porturilor digitale
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);

    // Calibrarea giroscopului
    for (cnt = 0; cnt < 500; cnt++)
    {
        if (cnt % 15 == 0)
        {
            digitalWrite(LED_RED, !digitalRead(LED_RED));
        }

        Wire.beginTransmission(gyro_address);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(gyro_address, 4);
        gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();
        gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();
        delayMicroseconds(3700);
    }
    gyro_pitch_calibration_value /= 500;
    gyro_yaw_calibration_value /= 500;

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    playTone(BUZZER_PIN, 1000, 2000);
    delay(1000);

    ultrasonic.begin();

    loop_timer = micros() + 4000;
}

void loop()
{
    ultrasonic.start();
    if(ultrasonic.isFinished())
    {
      front_sensor_cm = ultrasonic.getRange();
      if(front_sensor_cm < 10)
        digitalWrite(LED_RED, HIGH);
      else
        digitalWrite(LED_RED, LOW);

    }
    // Calculele pentru unghiuri
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 2);
    accelerometer_data_raw = Wire.read() << 8 | Wire.read();
    accelerometer_data_raw += acc_calibration_value;
    accelerometer_data_raw = constrain(accelerometer_data_raw, -8200, 8200);
    angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;

    if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5)
    {
        angle_gyro = angle_acc;
        start = 1;
    }

    Wire.beginTransmission(gyro_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 4);
    gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();
    gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();
    gyro_pitch_data_raw -= gyro_pitch_calibration_value;
    angle_gyro += gyro_pitch_data_raw * 0.000031;

    // Compensarea offsetului MPU-6050
    gyro_yaw_data_raw -= gyro_yaw_calibration_value;
    angle_gyro -= gyro_yaw_data_raw * -0.0000003;
    angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;

    // Calculele PID
    pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
    if (pid_output > 10 || pid_output < -10)
        pid_error_temp += pid_output * 0.015;

    pid_i_mem += pid_ki * pid_error_temp;
    pid_i_mem = constrain(pid_i_mem, -400, 400);
    pid_output = pid_kp * pid_error_temp + pid_i_mem + pid_kd * (pid_error_temp - pid_last_d_error);
    pid_output = constrain(pid_output, -400, 400);
    pid_last_d_error = pid_error_temp;
    if (abs(pid_output) < 5)
        pid_output = 0;

    if (abs(angle_gyro) > 30 || start == 0)
    {
        pid_output = 0;
        pid_i_mem = 0;
        start = 0;
        self_balance_pid_setpoint = 0;
    }

    // Calculele de control
    pid_output_left = pid_output;
    pid_output_right = pid_output;

    if (pid_setpoint > 0.5)
        pid_setpoint -= 0.05;
    else if (pid_setpoint < -0.5)
        pid_setpoint += 0.05;
    else
        pid_setpoint = 0;

    if (pid_setpoint == 0)
    {
        if (pid_output < 0)
            self_balance_pid_setpoint += 0.0015;
        if (pid_output > 0)
            self_balance_pid_setpoint -= 0.0015;
    }

    // Calculele impulsurilor pentru motoare
    pid_output_left = pid_output_left > 0 ? 405 - (1 / (pid_output_left + 9)) * 5500 : -405 - (1 / (pid_output_left - 9)) * 5500;
    pid_output_right = pid_output_right > 0 ? 405 - (1 / (pid_output_right + 9)) * 5500 : -405 - (1 / (pid_output_right - 9)) * 5500;

    left_motor = pid_output_left > 0 ? 400 - pid_output_left : -400 - pid_output_left;
    right_motor = pid_output_right > 0 ? 400 - pid_output_right : -400 - pid_output_right;

    throttle_left_motor = left_motor;
    throttle_right_motor = right_motor;

    // Timerul pentru bucla de timp
    while (loop_timer > micros());
    loop_timer += 4000;
}

// Rutina de întrerupere TIMER2_COMPA_vect
ISR(TIMER2_COMPA_vect)
{
    // Calculele impulsurilor pentru motorul stâng
    throttle_counter_left_motor++; // Crește variabila throttle_counter_left_motor cu 1 la fiecare execuție a acestei rutine
    if (throttle_counter_left_motor > throttle_left_motor_memory)
    { // Dacă numărul de bucle este mai mare decât variabila throttle_left_motor_memory
        throttle_counter_left_motor = 0; // Resetează variabila throttle_counter_left_motor
        throttle_left_motor_memory = throttle_left_motor; // Încarcă următoarea variabilă throttle_left_motor
        if (throttle_left_motor_memory < 0)
        { // Dacă throttle_left_motor_memory este negativă
            PORTD &= 0b00100000; // Setează ieșirea 5 la low pentru a inversa direcția controlerului stepper
            throttle_left_motor_memory *= -1; // Inversează variabila throttle_left_motor_memory
        }
        else
            PORTD |= 0b11011111; // Setează ieșirea 5 la high pentru o direcție înainte a motorului stepper
    }
    else if (throttle_counter_left_motor == 1)
        PORTD |= 0b01000000; // Setează ieșirea 6 la high pentru a crea un impuls pentru controlerul stepper
    else if (throttle_counter_left_motor == 2)
        PORTD &= 0b10111111; // Setează ieșirea 6 la low deoarece impulsul trebuie să dureze doar 20us

    // Calculele impulsurilor pentru motorul drept
    throttle_counter_right_motor++; // Crește variabila throttle_counter_right_motor cu 1 la fiecare execuție a rutinei
    if (throttle_counter_right_motor > throttle_right_motor_memory)
    { // Dacă numărul de bucle este mai mare decât variabila throttle_right_motor_memory
        throttle_counter_right_motor = 0; // Resetează variabila throttle_counter_right_motor
        throttle_right_motor_memory = throttle_right_motor; // Încarcă următoarea variabilă throttle_right_motor
        if (throttle_right_motor_memory < 0)
        { // Dacă throttle_right_motor_memory este negativă
            PORTD |= 0b11110111; // Setează ieșirea 3 la low pentru a inversa direcția controlerului stepper
            throttle_right_motor_memory *= -1; // Inversează variabila throttle_right_motor_memory
        }
        else
            PORTD &= 0b00001000; // Setează ieșirea 3 la high pentru o direcție înainte a motorului stepper
    }
    else if (throttle_counter_right_motor == 1)
        PORTD |= 0b00010000; // Setează ieșirea 4 la high pentru a crea un impuls pentru controlerul stepper
    else if (throttle_counter_right_motor == 2)
        PORTD &= 0b11101111; // Setează ieșirea 4 la low deoarece impulsul trebuie să dureze doar 20us
}

void playTone(int pin, int freq, int duration) {
    int period = 1000000 / freq;
    int halfPeriod = period / 2;

    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(pin, LOW);
        delayMicroseconds(halfPeriod);
    }
}