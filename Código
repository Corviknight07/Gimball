#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Servo.h>

MPU6050 mpu;

Servo servoRoll;
Servo servoPitch;

int ax, ay, az;
int gx, gy, gz;

unsigned long tiempo_prev;
float dt;

float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

float roll_setpoint = 0;
float pitch_setpoint = 0;

float roll_error, pitch_error;
float roll_prev_error = 0;
float pitch_prev_error = 0;

float roll_p, roll_i = 0, roll_d;
float pitch_p, pitch_i = 0, pitch_d;

float Kp = 4.0;
float Ki = 0.1;
float Kd = 0.1;

float roll_PID, pitch_PID;

void lecturaMPU()
{
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}

void calcularPID() {
  roll_error  = ang_y - roll_setpoint;
  pitch_error = ang_x - pitch_setpoint;

  roll_p  = Kp * roll_error;
  pitch_p = Kp * pitch_error;

  if (roll_error > -3 && roll_error < 3)
    roll_i += Ki * roll_error;

  if (pitch_error > -3 && pitch_error < 3)
    pitch_i += Ki * pitch_error;

  roll_d  = Kd * ((roll_error - roll_prev_error) / dt);
  pitch_d = Kd * ((pitch_error - pitch_prev_error) / dt);

  roll_PID  = roll_p + roll_i + roll_d;
  pitch_PID = pitch_p + pitch_i + pitch_d;

  roll_PID  = constrain(roll_PID,  -90, 90);
  pitch_PID = constrain(pitch_PID, -90, 90);

  roll_prev_error  = roll_error;
  pitch_prev_error = pitch_error;
}

void moverServos() {
  servoRoll.write(90 - roll_PID);
  servoPitch.write(90 + pitch_PID);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  servoRoll.attach(3);
  servoPitch.attach(5);

  tiempo_prev = millis();

  if (mpu.testConnection())
    Serial.println("MPU OK");
  else
    Serial.println("Error MPU");
}

void loop() {
  lecturaMPU();
  calcularPID();
  moverServos();

  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);

}
