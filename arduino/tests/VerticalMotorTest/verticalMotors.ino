
int verticalMotorPin1 = 3;
int verticalMotorPin2 = 4;
int verticalMotorSign = 0;

void setup() {
  pinMode(verticalMotorPin1, OUTPUT);
  pinMode(verticalMotorPin2, OUTPUT);
  analogWrite(verticalMotorPin1, 0);
  analogWrite(verticalMotorPin2, 0);
}

int signum(int n){
  if (n > 0) return 1;
  if (n < 0) return -1;
  return 0;
}

void verticalMotorWrite(int value){ //[-255,255]
  if( signum(value) == -verticalMotorSign ){
    analogWrite(verticalMotorPin1, 0);
    analogWrite(verticalMotorPin2, 0);
    delay(1);
  }
  verticalMotorSign = signum(value);

  if(value > 0)
    analogWrite(verticalMotorPin1, value);
  else
    analogWrite(verticalMotorPin2, -value);
}


void loop() {
  verticalMotorWrite(200);
  delay(1000);
  verticalMotorWrite(-200);
  delay(1000);
}
