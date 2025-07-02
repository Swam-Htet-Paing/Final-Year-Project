#include "Servo.h"
#include "Wire.h"
#include <MPU6050_light.h>


MPU6050 mpu(Wire);
unsigned long timer = 0;
unsigned long last_drive_time = 0;  // Timestamp of last 'w' command
bool drive_active = false;
Servo s1;
Servo s2;
int pwm = 5;
int steer_angle = 90;  // default straight




float angle, prev_angle = 0, Kp = 3.5, Ki = 0, Kd = 0.06, integral = 0, derivative = 0, output = 0, dt = 0.01;
float y_now = 0, y_prev = 0, x_now = 0, x_prev = 0;
float alpha = 0.9, alpha1 = 0.7, prev_output = 0;


void drive(){
 analogWrite(pwm, 105);
}


void stopDrive(){
 analogWrite(pwm, 0);
}


void driveSteer() {
 if (Serial.available() > 0) {
   String msg = Serial.readStringUntil('\n');
   msg.trim();  // remove newline and whitespace


   if (msg == "w") {
     drive_active = true;
     last_drive_time = millis();
   } else if (msg == "a") {
     steer_angle = constrain(steer_angle - 3, 0, 180);
     s2.write(steer_angle);
   } else if (msg == "d") {
     steer_angle = constrain(steer_angle + 3, 0, 180);
     s2.write(steer_angle);
   }
}
}



void setup() {
 pinMode(pwm, OUTPUT);
 Serial.begin(9600);
 Wire.begin();
 s1.attach(9);
 s2.attach(10);  // use pin 10 for steering
 s2.write(steer_angle);
 s1.write(90);
  byte status = mpu.begin();
 Serial.print(F("MPU6050 status: "));
 Serial.println(status);
 while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
 delay(1000);
 // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
 mpu.calcOffsets(); // gyro and accelero
 Serial.println("Done!\n");
}


void loop() {
 mpu.update();
 if ((millis() - timer) > 10) {
   angle = mpu.getAngleX();
   angle = alpha * prev_angle + (1 - alpha) * angle;
   driveSteer();
   // Auto-stop drive if too long since last 'w'
   if (drive_active && millis() - last_drive_time > 50) {
     drive_active = false;
     stopDrive();
   }
   if (drive_active) {
     drive();
   }
   // PID control
   integral += angle * dt;
   derivative = (angle - prev_angle) / dt;
   output = Kp * angle + Kd * derivative;
   output = constrain(output, -180, 180);
   output = map(output, -180, 180, 0, 180);
   output = alpha1 * prev_output + (1 - alpha1) * output;
   s1.write(180 - output);
   Serial.println(output);


   prev_angle = angle;
   prev_output = output;
   timer = millis();
 }
}
