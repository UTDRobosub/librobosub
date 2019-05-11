#undef max
#undef min

//!!!!!IMPORTANT!!!!!
//If the IMU is not repsonsive and doesn't show up in the serial port list
//ground the SCL pin during startup to enter bootloader mode

////////////////////////////////////////////////////////////////////////////////////////////
//Encoding and decoding; not part of Serial class.
//Copy and paste this code into an arduino and it will still work.

int strFindFirstMasked(char *buf, int maxlen, char until, char mask){
  for(int i=0; i<maxlen; i++){
    if((buf[i]&mask)==until){
      return i;
    }
  }

  return -1;
}

int strFindLastMasked(char *buf, int maxlen, char until, char mask){
  for(int i=maxlen-1; i>=0; i--){
    if((buf[i]&mask)==until){
      return i;
    }
  }
}

const char Serial_MsgBeginChar = '[';
const char Serial_MsgEndChar = ']';
const char Serial_EscapeChar = '\\';

int Serial_SpecialCharCount = 3;
const char Serial_SpecialChars[] = {
  Serial_MsgBeginChar,
  Serial_MsgEndChar,
  Serial_EscapeChar,
};
const char Serial_SpecialCharEscapes[] = {
  '(',
  ')',
  '/',
};

int SerialDataEncode(char *decoded, int decodedlen, char *encoded){
  int encodedlen = 0;

  encoded[encodedlen++] = Serial_MsgBeginChar;

  for(int ci=0; ci<decodedlen; ci++){
    char c = decoded[ci];

    bool special = false;
    for(int scci=0; scci<Serial_SpecialCharCount; scci++){
      if(c==Serial_SpecialChars[scci]){
        encoded[encodedlen++] = Serial_EscapeChar;
        encoded[encodedlen++] = Serial_SpecialCharEscapes[scci];
        special = true;
      }
    }

    if(!special){
      encoded[encodedlen++] = c;
    }
  }

  encoded[encodedlen++] = Serial_MsgEndChar;

  return encodedlen;
}

int SerialDataDecode(char *encoded, int encodedlen, char *decoded){
  int decodedlen = 0;

  for(int ci=0; ci<encodedlen; ci++){
    char c = encoded[ci];

    if(c==Serial_EscapeChar){ //if the current character is the escape, the next character is an escaped special char
      ci++;
      c = encoded[ci];

      bool special = false;
      for(int scci=0; scci<Serial_SpecialCharCount; scci++){
        if(c==Serial_SpecialCharEscapes[scci]){
          decoded[decodedlen++] = Serial_SpecialChars[scci];
          special = true;
        }
      }

      if(!special){ //the last character was the escape but the current one does not match any special characters
        //cout<<"Error: Escaped non-special char "<<c<<endl;
      }
    }else{ //otherwise it's a regular character
      decoded[decodedlen++] = c;
    }
  }

  return decodedlen;
}

int SerialReadAndRemoveFirstEncodedDataFromBuffer(char *buf, int *buflen, char *decoded, int maxdecodedlen){
  int startloc = strFindFirstMasked(buf, *buflen, Serial_MsgBeginChar, 0xFF); //find the first start marker in the string
  int endloc = strFindFirstMasked(buf+startloc+1, *buflen-startloc-1, Serial_MsgEndChar, 0xFF);

  if(startloc!=-1 && endloc!=-1){
    endloc += startloc+1;

    startloc = strFindLastMasked(buf, endloc, Serial_MsgBeginChar, 0xFF);

    int encodedlen = endloc-startloc-1;
    char *encoded = (char*)malloc(encodedlen+1);

    memcpy(encoded, buf+startloc+1, encodedlen);

    memmove(buf, buf+endloc, *buflen-endloc-1);
    *buflen -= endloc+1;

    return SerialDataDecode(encoded, encodedlen, decoded);
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////


#include <SPI.h>

#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
int ACCEL_SCALE = 2; //Acceleration scale in g's. Valid values: 2 4 8 16
int GYRO_SCALE = 250; //Gyro scale in dps. Valid values: 250 500 1000 2000

/* Error codes
 * 0xFA01 IMU initialization failed
 * 0xFA02 Updating FIFO with DMP data failed
 */
MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

void setup() {
  SerialPort.begin(115200); //Start serial port

  if (imu.begin() != INV_SUCCESS) {
    SerialPort.println("IMU INIT FAILED");
    //Serial.write(0xFA01); //IMU init failed
  } else {
    SerialPort.println("IMU INIT SUCCESS");
  }

  imu.setAccelFSR(ACCEL_SCALE);
  imu.setGyroFSR(GYRO_SCALE);

  imu.dmpBegin( DMP_FEATURE_SEND_RAW_ACCEL | //Send raw accelerometer data
                DMP_FEATURE_SEND_RAW_GYRO | //Send raw gyroscope data
                DMP_FEATURE_6X_LP_QUAT, //Calculate quats with accel/gyro
              100); //Set update rate to 100Hz
}

void sendData(void *data, int size) {
  //Call data send function from lib
}

struct SensorDataPacket {
  unsigned long time;
  int aScale;
  int ax, ay, az;
  int gScale;
  int gx, gy, gz;
  long qx, qy, qz, qw;

  String toString() {
    String s = String(ax) + String(",") + String(ay) + String(",") + String(az)
               + String(",") + String(gx) + String(",") + String(gy) + String(",") + String(gz)
               + String(",") + String(time) + String(",") + String(aScale) + String(",") + String(gScale);
    return s;
  }
};

void setAccelScale(int newScale) {
  if (imu.setAccelFSR(newScale) != 0) {
    //Accel scale set failed
  }
}

void setGyroScale(int newScale) {
  if (imu.setGyroFSR(newScale) != 0) {
    //Gyro scale set failed
  }
}

char encoded[512];

void loop() {
  if (imu.fifoAvailable() > 0) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {

      struct SensorDataPacket data = {imu.time,
        imu.getAccelSens(),
        imu.ax, imu.ay, imu.az,
        imu.getGyroSens(),
        imu.gx, imu.gy, imu.gz,
        imu.qx, imu.qy, imu.qz, imu.qw
      };

      String dataString = data.toString();
      int encodedDataLength = SerialDataEncode((char*)dataString.c_str(), dataString.length()*sizeof(char), encoded);

      for (int i = 0; i < encodedDataLength; i++) {
        SerialPort.write(encoded[i]);
      }
    } else {
      SerialPort.println("IMU FIFO UPDATE FAILED");
      //Serial.write(0xFA02);
    }
  }
}
