
#include "robosub-serial.h"

//Sample usage

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

void* instance = 0;

/////////////////////////////////

void receiveMessageCallback(void* instance, char* message, int length, bool needsresponse, char** response, int* responselength){
  //your code here
  
}

void setup(){
  Serial.begin(115200);

  pinMode(A0, INPUT);

  serialstate = Serial_NewState(instance, receiveMessageCallback, sendCharFn, delayMsFn, pollReceiveFn);
}

int ticksToSend = 10;
int curTick = 0;

char senddata[32];
int senddatalength;

void loop(){
  pollReceiveFn(0);

  curTick++;
  if(curTick>=ticksToSend){
    
    senddata[0] = analogRead(A0)/4;
    senddata[1] = 'z';
    senddatalength = 2;
    
    Serial_SendMessage(serialstate, senddata, senddatalength, false);

    curTick = 0;
  }

  delay(1);
}

