#include <Servo.h>

byte servoPin11 = 11;
byte servoPin10 = 10;
byte servoPin9 = 9;
byte servoPin8 = 8;

Servo servo;
Servo servo10;
Servo servo9;
Servo servo8;

void setup() {
  servo.attach(servoPin11);
  servo10.attach(servoPin10);
  servo9.attach(servoPin9);
  servo8.attach(servoPin8);

  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  servo10.writeMicroseconds(1500); // send "stop" signal to ESC.
  servo9.writeMicroseconds(1500); // send "stop" signal to ESC.
  servo8.writeMicroseconds(1500);
  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  servo.writeMicroseconds(1550); // Send signal to ESC.
  servo10.writeMicroseconds(1550);
  servo9.writeMicroseconds(1550);
  servo8.writeMicroseconds(1550);
  delay(1000);
  
  servo.writeMicroseconds(1500);
  servo10.writeMicroseconds(1500);
  servo9.writeMicroseconds(1500);
  servo8.writeMicroseconds(1500);
  delay(1000);
}
