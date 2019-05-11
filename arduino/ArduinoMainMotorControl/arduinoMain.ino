
#include "robosub-serial.h"

/////////////////////////////////
//Serial setup

Serial_State* serialstate;

void sendCharFn(void* instance, char data, bool terminate){
	Serial.write(data);
}

void delayMsFn(void* instance, int ms){
	delay(ms);
}

void pollReceiveFn(void* instance){
	while(Serial.available()){
		char data = Serial.read();
		Serial_ReceiveChar(serialstate, data);
	}
}

void* serialInstance = 0;

int testidx = '0';

void serial_setup(){
	Serial.begin(115200);
	
	serialstate = Serial_NewState(serialInstance, receiveMessageCallback, sendCharFn, delayMsFn, pollReceiveFn);
	
	delay(100);
	
	//char* smsg = "mtest_";
	//smsg[5] = testidx++;
	//
	//Serial_SendMessage(serialstate, smsg, 6, true);
	//
	//Serial.println("started serial");
	
	//delay(100);
}

/////////////////////////////////
//Motor setup



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//DO NOT WRITE TO PINS 5 OR 6
//EXCEPT THROUGH verticalMotorWrite()
byte verticalMotorPin1 = 5;
byte verticalMotorPin2 = 6;
byte verticalMotorSign = 0;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


#include <Servo.h>

byte servoPin11 = 11;
byte servoPin10 = 10;
byte servoPin9 = 9;
byte servoPin8 = 8;

//arm motors
byte servoPin2 = 2;
byte servoPin3 = 3;
byte servoPin4 = 4;
byte servoPin7 = 7;

Servo servo;
Servo servo10;
Servo servo9;
Servo servo8;

Servo servo2;
Servo servo3;
Servo servo4;
Servo servo7;

int analogVals[256];

void motor_setup() {
	for(int i=0; i<256; i++){ analogVals[i] = 0; }
	
	pinMode(verticalMotorPin1, OUTPUT);
	pinMode(verticalMotorPin2, OUTPUT);
	analogWrite(verticalMotorPin1, 0);
	analogWrite(verticalMotorPin2, 0);
	
	servo.attach(servoPin11);
	servo10.attach(servoPin10);
	servo9.attach(servoPin9);
	servo8.attach(servoPin8);
	servo2.attach(servoPin2);
	servo3.attach(servoPin3);
	servo4.attach(servoPin4);
	servo7.attach(servoPin7);
	
	servo.writeMicroseconds(1500); // send "stop" signal to ESC.
	servo10.writeMicroseconds(1500); // send "stop" signal to ESC.
	servo9.writeMicroseconds(1500); // send "stop" signal to ESC.
	servo8.writeMicroseconds(1500);
	servo2.write(30);
	servo3.write(30);
	servo4.write(30);
	servo7.write(30);
	delay(1000); // delay to allow the ESC to recognize the stopped signal
}

int signum(int n){
	if (n > 0) return 1;
	if (n < 0) return -1;
	return 0;
}

void analogWriteDebug(int pin, int val){
	analogWrite(pin, val);
	analogVals[pin] = val;
}

void verticalMotorWrite(int value){ //[-255,255]
	if( signum(value) == -verticalMotorSign ){
		analogWriteDebug(verticalMotorPin1, 0);
		analogWriteDebug(verticalMotorPin2, 0);
		//delay(1);
	}
	verticalMotorSign = signum(value);
	
	if(value > 0)
		analogWriteDebug(verticalMotorPin1, value);
	else
		analogWriteDebug(verticalMotorPin2, -value);
}

/////////////////////////////////
//Main program

struct MotorValues {
	short bl;
	short br;
	short ul;
	short ur;
	short v ; //-255 to 255
	short a1; //0 to 130 degrees
	short a2; //0 to 180 degrees
	short a3; //0 to 65 degrees
	short a4; //? to ? degrees
};

MotorValues motorvals;

void motor_update(){
	servo  .writeMicroseconds(motorvals.br); //actually br
	servo8 .writeMicroseconds(motorvals.ul); //actually fl
	servo9 .writeMicroseconds(motorvals.bl); //actually bl
	servo10.writeMicroseconds(motorvals.ur); //actually fr
	verticalMotorWrite(motorvals.v);         //vertical motors
	servo2.write(motorvals.a1);              //arm motors
	servo3.write(motorvals.a2);              
	servo4.write(motorvals.a3);              
	servo7.write(motorvals.a4);              
}

void receiveMessageCallback(void* instance, char* message, int length, bool needsresponse, char** response, int* responselength){
	//your code here
	
	memcpy(&motorvals, message, sizeof(motorvals));
	//motor_update();
}

void setup(){
	motorvals.bl = 1500;
	motorvals.br = 1500;
	motorvals.ul = 1500;
	motorvals.ur = 1500;
	motorvals.v  = 0;
	motorvals.a1 = 0;
	motorvals.a2 = 0;
	motorvals.a3 = 0;
	motorvals.a4 = 0;
	
	serial_setup();
	motor_setup();
	
	//motor_update();
}

char msg[256];
int msgl;

void loop(){
	pollReceiveFn(0);
	
	motor_update();
	
	msgl = 0;
	msg[msgl++] = 'm';
	
	for(int i=0; i<256; i++){
		int v = analogVals[i];
		if(v!=0){
			msg[msgl++] = i + '0';
			msg[msgl++] = '=';
			msg[msgl++] = v + '0';
			msg[msgl++] = '\n';
		}
	}
	
	Serial_SendMessage(serialstate, msg, msgl, false);
	
	delay(10);
}
