
//Universal Serial library, usable on Arduino and in C++

const unsigned char Serial_CRC8Table[256] ={
  0x00, 0x25, 0x4A, 0x6F, 0x94, 0xB1, 0xDE, 0xFB,
  0x0D, 0x28, 0x47, 0x62, 0x99, 0xBC, 0xD3, 0xF6,
  0x1A, 0x3F, 0x50, 0x75, 0x8E, 0xAB, 0xC4, 0xE1,
  0x17, 0x32, 0x5D, 0x78, 0x83, 0xA6, 0xC9, 0xEC,
  0x34, 0x11, 0x7E, 0x5B, 0xA0, 0x85, 0xEA, 0xCF,
  0x39, 0x1C, 0x73, 0x56, 0xAD, 0x88, 0xE7, 0xC2,
  0x2E, 0x0B, 0x64, 0x41, 0xBA, 0x9F, 0xF0, 0xD5,
  0x23, 0x06, 0x69, 0x4C, 0xB7, 0x92, 0xFD, 0xD8,
  0x68, 0x4D, 0x22, 0x07, 0xFC, 0xD9, 0xB6, 0x93,
  0x65, 0x40, 0x2F, 0x0A, 0xF1, 0xD4, 0xBB, 0x9E,
  0x72, 0x57, 0x38, 0x1D, 0xE6, 0xC3, 0xAC, 0x89,
  0x7F, 0x5A, 0x35, 0x10, 0xEB, 0xCE, 0xA1, 0x84,
  0x5C, 0x79, 0x16, 0x33, 0xC8, 0xED, 0x82, 0xA7,
  0x51, 0x74, 0x1B, 0x3E, 0xC5, 0xE0, 0x8F, 0xAA,
  0x46, 0x63, 0x0C, 0x29, 0xD2, 0xF7, 0x98, 0xBD,
  0x4B, 0x6E, 0x01, 0x24, 0xDF, 0xFA, 0x95, 0xB0,
  0xD0, 0xF5, 0x9A, 0xBF, 0x44, 0x61, 0x0E, 0x2B,
  0xDD, 0xF8, 0x97, 0xB2, 0x49, 0x6C, 0x03, 0x26,
  0xCA, 0xEF, 0x80, 0xA5, 0x5E, 0x7B, 0x14, 0x31,
  0xC7, 0xE2, 0x8D, 0xA8, 0x53, 0x76, 0x19, 0x3C,
  0xE4, 0xC1, 0xAE, 0x8B, 0x70, 0x55, 0x3A, 0x1F,
  0xE9, 0xCC, 0xA3, 0x86, 0x7D, 0x58, 0x37, 0x12,
  0xFE, 0xDB, 0xB4, 0x91, 0x6A, 0x4F, 0x20, 0x05,
  0xF3, 0xD6, 0xB9, 0x9C, 0x67, 0x42, 0x2D, 0x08,
  0xB8, 0x9D, 0xF2, 0xD7, 0x2C, 0x09, 0x66, 0x43,
  0xB5, 0x90, 0xFF, 0xDA, 0x21, 0x04, 0x6B, 0x4E,
  0xA2, 0x87, 0xE8, 0xCD, 0x36, 0x13, 0x7C, 0x59,
  0xAF, 0x8A, 0xE5, 0xC0, 0x3B, 0x1E, 0x71, 0x54,
  0x8C, 0xA9, 0xC6, 0xE3, 0x18, 0x3D, 0x52, 0x77,
  0x81, 0xA4, 0xCB, 0xEE, 0x15, 0x30, 0x5F, 0x7A,
  0x96, 0xB3, 0xDC, 0xF9, 0x02, 0x27, 0x48, 0x6D,
  0x9B, 0xBE, 0xD1, 0xF4, 0x0F, 0x2A, 0x45, 0x60
};

const int Serial_TicksToWaitForAck = 1000;
const int Serial_TickLengthMs = 1;

const int Serial_MaxMessageLength = 256;

const char Serial_State_Outside        = 1;
const char Serial_State_InMessageFlags = 2;
const char Serial_State_InMessageSN    = 3;
const char Serial_State_InMessageData  = 4;
const char Serial_State_InMessageCRC   = 5;

const char Serial_Flags_IncludesAck       = 0b00000001;
const char Serial_Flags_NeedsAck          = 0b00000010;
const char Serial_Flags_NoCRCCheck        = 0b00000100;
const char Serial_Flags_NeedsAckData      = 0b00001000;
const char Serial_Flags_IncludesAckData   = 0b00010000;

struct Serial_State{
  void* instance;
  
  void (*onReceiveMessage)(void* instance, char* message, int length, bool needsresponse, char** response, int* responsepength);
  void (*sendChar)(void* instance, char data, bool terminate);
  void (*delay)(void* instance, int milliseconds);
  void (*pollReceive)(void* instance);
  
  char state;
  
  bool inCharacterEscape;
  char currentMessageFlags;
  unsigned char currentMessageSN;
  unsigned char currentMessageCRC;
  unsigned char currentMessageComputedCRC;
  
  bool awaitingAck;
  unsigned char awaitingAckSN;
  
  bool lastMessageHasAckData;
  
  unsigned char lastUsedSN;
  
  int currentMessageDataLength;
  char currentMessageData[Serial_MaxMessageLength];
};

Serial_State* Serial_NewState(
  void* instanceP,
  void (*onReceiveMessageF)(void* instance, char* message, int length, bool needsresponse, char** response, int* responsepength),
  void (*sendCharF)(void* instance, char data, bool terminate),
  void (*delayF)(void* instance, int milliseconds),
  void (*pollReceiveF)(void* instance)
){
  Serial_State* state = new Serial_State();
  
  state->instance = instanceP;
  
  state->onReceiveMessage = onReceiveMessageF;
  state->sendChar = sendCharF;
  state->delay = delayF;
  state->pollReceive = pollReceiveF;
  
  state->state = Serial_State_Outside;
  
  state->inCharacterEscape = false;
  state->currentMessageFlags = 0;
  state->currentMessageSN = 0;
  state->currentMessageCRC = 0;
  state->currentMessageComputedCRC = 0;
  
  state->awaitingAck = false;
  state->awaitingAckSN = 0;
  
  state->lastMessageHasAckData = false;
  
  state->lastUsedSN = 'a';
  
  state->currentMessageDataLength = 0;
  
  return state;
}

void Serial_DeleteState(Serial_State* state){
  free(state->currentMessageData);
  delete(state);
}

void Serial_SendChar(Serial_State *state, char data, bool isfinal){
  state->sendChar(state->instance, data, isfinal);
}

void Serial_SendCharEscaped(Serial_State *state, char data, bool isfinal){
  if     (data=='[' ){ Serial_SendChar(state, '\\', false  ); Serial_SendChar(state, '(', isfinal); }
  else if(data==']' ){ Serial_SendChar(state, '\\', false  ); Serial_SendChar(state, ')', isfinal); }
  else if(data=='\\'){ Serial_SendChar(state, '\\', false  ); Serial_SendChar(state, '/', isfinal); }
  else               { Serial_SendChar(state, data, isfinal);                                }
}

void Serial_SendMessageComplex(Serial_State* state, char* buffer, int length, unsigned char sequenceNumber, bool includesack, bool needsack, bool needsackdata, bool includesackdata, char** response, int* responselength){
  bool retransmit = true;
  
  while(retransmit){
    
    char flags = 0b00100000;
    if(includesack    ){ flags = flags|Serial_Flags_IncludesAck    ; }
    if(needsack       ){ flags = flags|Serial_Flags_NeedsAck       ; }
    if(needsackdata   ){ flags = flags|Serial_Flags_NeedsAckData   ; }
    if(includesackdata){ flags = flags|Serial_Flags_IncludesAckData; }
    
    unsigned char crc = 0;
    
    Serial_SendChar(state, '[', false);
    
    Serial_SendCharEscaped(state, flags, false);
    crc = Serial_CRC8Table[crc ^ (unsigned char)flags];
    
    Serial_SendCharEscaped(state, sequenceNumber, false);
    crc = Serial_CRC8Table[crc ^ (unsigned char)sequenceNumber];
    
    for(int i=0; i<length; i++){
      char data = buffer[i];
      
      Serial_SendCharEscaped(state, data, false);
      
      crc = Serial_CRC8Table[crc ^ (unsigned char)data];
    }
    
    Serial_SendChar(state, ']', false);
    
    Serial_SendCharEscaped(state, crc, true);
    
    if(needsack || needsackdata){
      state->awaitingAck = true;
      state->awaitingAckSN = sequenceNumber;
      
      int ticksWaited = 0;
      
      while(state->awaitingAck && ticksWaited<Serial_TicksToWaitForAck){
        state->pollReceive(state->instance);
        
        state->delay(state->instance, Serial_TickLengthMs);
        
        ticksWaited++;
      }
      
      if(!state->awaitingAck){
        retransmit = false;
        
        if(state->lastMessageHasAckData){
          state->lastMessageHasAckData = false;
          
          response = (char**)&(state->currentMessageData);
          responselength = &(state->currentMessageDataLength);
        }else{
          response = 0;
          responselength = 0;
        }
      }
    }else{
      retransmit = false;
    }
  }
}

void Serial_OnReceiveMessage(Serial_State* state){
  bool needsack        = (state->currentMessageFlags & Serial_Flags_NeedsAck       ) != 0;
  bool needsackdata    = (state->currentMessageFlags & Serial_Flags_NeedsAckData   ) != 0;
  bool includesackdata = (state->currentMessageFlags & Serial_Flags_IncludesAckData) != 0;
  bool includesack     = (state->currentMessageFlags & Serial_Flags_IncludesAck    ) != 0;
  
  if(includesack){
    if(state->currentMessageSN==state->awaitingAckSN){
      state->awaitingAck = false;
    }
  }
  
  if(needsack){
    Serial_SendMessageComplex(state, 0, 0, state->currentMessageSN, true, false, false, false, 0, 0);
  }
  
  char* response;
  int responselength;
  
  if(includesackdata){
    state->lastMessageHasAckData = true;
  }else{
    state->onReceiveMessage(state->instance, state->currentMessageData, state->currentMessageDataLength, needsackdata, &response, &responselength);
  }
  
  if(needsackdata){
    Serial_SendMessageComplex(state, response, responselength, state->currentMessageSN, false, false, false, true, 0, 0);
  }
}

void Serial_ReceiveChar(Serial_State* state, char rawdata){
  char escdata = rawdata;
  
  if(rawdata=='\\'){
    state->inCharacterEscape = true;
  }else{
    if(state->inCharacterEscape && rawdata!='['){
      if     (rawdata=='(') escdata = '[';
      else if(rawdata==')') escdata = ']';
      else if(rawdata=='/') escdata = '\\';
      state->inCharacterEscape = false;
    }
    
    if(rawdata=='['){
      state->state = Serial_State_InMessageFlags;
      state->currentMessageFlags = 0;
      state->currentMessageSN = 0;
      state->currentMessageComputedCRC = 0;
      
      state->currentMessageDataLength = 0;
      
    }else if(state->state==Serial_State_InMessageFlags){
      state->currentMessageFlags = escdata;
      
      state->currentMessageComputedCRC = Serial_CRC8Table[state->currentMessageComputedCRC ^ (unsigned char)escdata];
      
      state->state = Serial_State_InMessageSN;
      
    }else if(state->state==Serial_State_InMessageSN){
      state->currentMessageSN = escdata;
      
      state->currentMessageComputedCRC = Serial_CRC8Table[state->currentMessageComputedCRC ^ (unsigned char)escdata];
      
      state->state = Serial_State_InMessageData;
      
    }else if(state->state==Serial_State_InMessageData){
      if(rawdata!=']'){
        if(state->currentMessageDataLength<Serial_MaxMessageLength){
          state->currentMessageData[state->currentMessageDataLength] = escdata;
          state->currentMessageDataLength++;
          
          state->currentMessageComputedCRC = Serial_CRC8Table[state->currentMessageComputedCRC ^ (unsigned char)escdata];
        }else{
          state->state = Serial_State_Outside;
        }
      }else{
        state->state = Serial_State_InMessageCRC;
      }
      
    }else if(state->state==Serial_State_InMessageCRC){
      state->currentMessageCRC = escdata;
      
      if(state->currentMessageCRC==state->currentMessageComputedCRC || state->currentMessageFlags&Serial_Flags_NoCRCCheck){ //if CRC passes
        Serial_OnReceiveMessage(state);
      }
      
      state->state = Serial_State_Outside;
    }
  }
}

void Serial_SendMessage(Serial_State* state, char* buffer, int length, bool needsack){
  state->lastUsedSN = (state->lastUsedSN+1)%256;
  char sequencenumber = state->lastUsedSN;
  
  Serial_SendMessageComplex(state, buffer, length, sequencenumber, false, needsack, false, false, 0, 0);
}

void Serial_SendMessageNeedsResponse(Serial_State* state, char* buffer, int length, bool needsack, char** response, int* responselength){
  state->lastUsedSN = (state->lastUsedSN+1)%256;
  char sequencenumber = state->lastUsedSN;
  
  Serial_SendMessageComplex(state, buffer, length, sequencenumber, false, false, true, false, response, responselength);
}

