#pragma once

#include "wled.h"
uint8_t CMD_ENABLE_CONFIG[] =           {0xfd,0xfc,0xfb,0xfa,0x04,0x00,0xff,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
uint8_t CMD_ENABLE_ENGINEERING_MODE[] = {0xfd,0xfc,0xfb,0xfa,0x02,0x00,0x62,0x00,0x04,0x03,0x02,0x01};
uint8_t CMD_DISABLE_CONFIG[] =          {0xfd,0xfc,0xfb,0xfa,0x02,0x00,0xfe,0x00,0x04,0x03,0x02,0x01};
uint8_t CMD_SET_BAUD[] =                {0xfd,0xfc,0xfb,0xfa,0x04,0x00,0xA1,0x00,0x07,0x00,0x04,0x03,0x02,0x01}; //index 8 sets baud
uint8_t CMD_RESTART[] =                 {0xfd,0xfc,0xfb,0xfa,0x02,0x00,0xA3,0x00,0x04,0x03,0x02,0x01}; 

enum LD2410CommStates{
  COM_INIT,
  COM_WAITOLD,
  COM_WAITNEW,
  COM_FINDBAUD,
  COM_SETBAUD,
  COM_WAITBAUD,
  COM_WAITRESTARTCMD,
  COM_WAITRESTARTMODULE,
  COM_WAITCONFIG,
  COM_WAITENG,
  COM_WAITENDCONFIG,
  COM_READY,
  COM_ERROR,
  COM_NOTREADY
}; //communication states

enum LD2410FrameStates{
    ST_IDLE,
    ST_CMD1,
    ST_CMD2,
    ST_CMD3,
    ST_GETLENGTH,
    ST_GETRESP, //response to configuration commands
    ST_CMD4,
    ST_CMD5,
    ST_CMD6,
    ST_CMD7,
    ST_ENDCMDFRAME,

    ST_DATA1,
    ST_DATA2,
    ST_DATA3,
    ST_D_LENGTH,
    ST_GETDATA, //get sensor data
    ST_DATA4,
    ST_DATA5,
    ST_DATA6,
    ST_DATA7,
    ST_ENDDATAFRAME,
};

enum LD2410Commands{
    ENABLE_CONFIG = 0xFF,
    END_CONFIG = 0xFE,
    MAX_GATES_AND_MIN_OUTPUT_TIME = 0x60,
    READ_GATES_CONFIG = 0x61,
    ENABLE_ENGINEERING_MODE = 0x62,
    DISABLE_ENGINEERING_MODE = 0x63,
    SET_GATE_SENSITIVITY = 0x64,
    READ_FIRMWARE_VERSION = 0xA0,
    SET_BAUD_RATE = 0xA1,
    RESET_SETTINGS = 0xA2,
    RESTART_MODULE = 0xA3,
    ENABLE_BLUETOOTH = 0xA4, //LD2410B only
    GET_MAC_ADDRESS = 0xA5,
    SET_BLUETOOTH_PASSWORD = 0xA8,
};

enum LD2410DataType{
    ENGINEERING_MODE = 0x01,
    BASIC_MODE = 0x02
};

struct LD2410GateFormat{
  uint8_t gate1;
  uint8_t gate2;
  uint8_t gate3;
  uint8_t gate4;
  uint8_t gate5;
  uint8_t gate6;
  uint8_t gate7;
  uint8_t gate8;
  uint8_t gate9;
};
#define MAX_GATES 9
struct LD2410EngFormat {
  LD2410DataType dataType;
  bool targetMoving;
  bool targetStationary;
  uint16_t movementDistance;
  uint8_t  movementEnergy;
  uint16_t stationaryDistance;
  uint8_t  stationaryEnergy;
  uint16_t detectionDistance; // combined of the 2????
  uint8_t  movementGateCount; // sould be 8
  uint8_t  stationaryGateCount; // sould be 8
  uint8_t movementGateEnergies[MAX_GATES]; //8 bytes each
  uint8_t stationaryGateEnergies[MAX_GATES];
};
/*
 * Usermods allow you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * 
 * This is an example for a v2 usermod.
 * v2 usermods are class inheritance based and can (but don't have to) implement more functions, each of them is shown in this example.
 * Multiple v2 usermods can be added to one compilation easily.
 * 
 * Creating a usermod:
 * This file serves as an example. If you want to create a usermod, it is recommended to use usermod_v2_empty.h from the usermods folder as a template.
 * Please remember to rename the class and file to a descriptive name.
 * You may also use multiple .h and .cpp files.
 * 
 * Using a usermod:
 * 1. Copy the usermod into the sketch folder (same folder as wled00.ino)
 * 2. Register the usermod by adding #include "usermod_filename.h" in the top and registerUsermod(new MyUsermodClass()) in the bottom of usermods_list.cpp
 */

//class name. Use something descriptive and leave the ": public Usermod" part :)
class LD2410 : public Usermod {

  private:
    LD2410FrameStates state = ST_IDLE;
    LD2410CommStates comState = COM_INIT;
    uint16_t dataLength = 0;

    // Private class members. You can declare variables and functions only accessible to your usermod here
    bool enabled = false; //whether to read data from LD2410 and change brightness of lights accordingly
    bool initDone = false;
    unsigned long lastTime = 0;

    // set your config variables to their boot default value (this can also be done in readFromConfig() or a constructor if you prefer)
    /*bool testBool = false;
    unsigned long testULong = 42424242;
    float testFloat = 42.42;
    String testString = "Forty-Two";*/
    int8_t RXpin = -1; //pin number of UART RX pin. Managed by pinManager
    int8_t TXpin = -1; //pin number of UART TX pin. Managed by pinManager
    HardwareSerial _serial = Serial1; //Serial interface of the LD2410
    uint8_t _buf[64] = {0}; //temporary storage for serial messages
    LD2410EngFormat data; // current data from the LD2410
    uint8_t baud = 7; //range 1 to 8. baud rate value of the LD2410
    uint8_t baudold = baud; 
    #define NUM_BAUD_VALUES 8u
    unsigned int baudTable[NUM_BAUD_VALUES] = {9600u,19200u,38400u,57600u,115200u,230400u,256000u,460800u}; //allowable baud rates for ld2410
    bool pinsReady = false; // whether Serial pins are allocated by pin manager

    bool testBool1 = true;
    bool testBool2 = true;
    bool testBool3 = true;
    bool testBool4 = true;
    bool testBool5 = false;
    bool testBool6 = false;

    uint16_t fullonDist = 30; //cm
    uint16_t fulloffDist = 400; //cm
    
    // These config variables have defaults set inside readFromConfig()
    /*int testInt;
    long testLong;
    int8_t testPins[2];*/

    // string that are used multiple time (this will save some flash memory)
    static const char _name[];
    static const char _enabled[];


    // any private methods should go here (non-inline methods should be defined out of class)
    void publishMqtt(const char* state, bool retain = false); // example for publishing MQTT message

    void printarr(uint8_t* a, uint32_t length){
      for(int i=0; i<length; i++){
        Serial.printf("%02x ",a[i]);
      }
      Serial.println();
    }

    void printdata(){
      //Serial.print("header: "); Serial.print("0x"); Serial.print(data.header,HEX); Serial.print(' '); Serial.println(data.header);
      //Serial.print("length: "); Serial.print("0x"); Serial.print(data.length,HEX); Serial.print(' '); Serial.println(data.length);
      Serial.print("isEngineeringMode: "); Serial.print("0x"); Serial.print(data.dataType,HEX); Serial.print(' '); Serial.println(data.dataType);
      //Serial.print("innerHeader: "); Serial.print("0x"); Serial.print(data.innerHeader,HEX); Serial.print(' '); Serial.println(data.innerHeader);
      //Serial.print("targetState: "); Serial.print("0x"); Serial.print(data.targetState,HEX); Serial.print(' '); Serial.println(data.targetState);
      Serial.print("movmentDistance: "); Serial.print("0x"); Serial.print(data.movementDistance,HEX); Serial.print(' '); Serial.println(data.movementDistance);
      Serial.print("movementEnergy: "); Serial.print("0x"); Serial.print(data.movementEnergy,HEX); Serial.print(' '); Serial.println(data.movementEnergy);
      Serial.print("stationaryDistance: "); Serial.print("0x"); Serial.print(data.stationaryDistance,HEX); Serial.print(' '); Serial.println(data.stationaryDistance);
      Serial.print("stationaryEnergy: "); Serial.print("0x"); Serial.print(data.stationaryEnergy,HEX); Serial.print(' '); Serial.println(data.stationaryEnergy);
      Serial.print("detectionDistance: "); Serial.print("0x"); Serial.print(data.detectionDistance,HEX); Serial.print(' '); Serial.println(data.detectionDistance);
      Serial.print("movementGateCount: "); Serial.print("0x"); Serial.print(data.movementGateCount,HEX); Serial.print(' '); Serial.println(data.movementGateCount);
      Serial.print("stationaryGateCount: "); Serial.print("0x"); Serial.print(data.stationaryGateCount,HEX); Serial.print(' '); Serial.println(data.stationaryGateCount);
      
      //Serial.printf("header %08x \n", data->header);
      //Serial.printf("headerdec %u \n", data->header);
      //Serial.printf("headeracc %u \n", 0xF1F2F3F4);
      //Serial.println(data->header == 0xF1F2F3F4);
    }

  public:
    uint8_t getSer(){
      uint8_t dat = _serial.read();
      //DEBUG_PRINTF("Serial Data: %d \r\n", dat);
      return dat;
    }

    uint16_t littleEndianToUINT16(uint8_t a, uint8_t b){
        return b << 8 | a;
    }

    uint32_t littleEndianToUINT32(uint8_t a, uint8_t b, uint8_t c, uint8_t d){
        return d << 24 | c << 16 | b << 8 | a;
    }

    //attempts to read a uint16_t from the serial port in little endian format. if it cannot, it returns false
    bool readuint16_t(uint16_t &data){
        if(_serial.available()<2) return false;
        data = littleEndianToUINT16(getSer(),getSer());
        //DEBUG_PRINTF("length: %d \r\n", data);
        return true;
    }

    bool getDataFrame(uint16_t dataLength){
        if(_serial.available() < dataLength) return false;
        if(_serial.readBytes(_buf, dataLength) != dataLength) return false;
        //DEBUG_PRINTF("data length: %d \r\n", dataLength);
        //printarr(_buf, dataLength);
        uint8_t _bufi = 0; //reset buffer index

        data.dataType = (LD2410DataType)_buf[_bufi++];
        uint8_t dataheader = _buf[_bufi++]; if(dataheader != 0xAA){DEBUG_PRINTLN("Incorrect header for sensor data");}
        
        uint8_t targetState = _buf[_bufi++]; 
        data.targetMoving = targetState & 0b00000001;
        data.targetStationary = targetState & 0b00000010;

        data.movementDistance = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);
        data.movementEnergy = _buf[_bufi++];

        data.stationaryDistance = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);
        data.stationaryEnergy = _buf[_bufi++];

        data.detectionDistance = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);

        if(data.dataType == ENGINEERING_MODE){
            uint8_t numberOfMovingGates = _buf[_bufi++]; if(numberOfMovingGates >= MAX_GATES) {numberOfMovingGates = MAX_GATES-1;}
            uint8_t numberOfStationaryGates = _buf[_bufi++]; if(numberOfMovingGates >= MAX_GATES) {numberOfMovingGates = MAX_GATES-1;}
            for(uint8_t i=0; i <= numberOfMovingGates; i++){
                data.movementGateEnergies[i] = _buf[_bufi++];
            }
            for(uint8_t i=0; i <= numberOfStationaryGates; i++){
                data.stationaryGateEnergies[i] = _buf[_bufi++];
            }

        }
        
        _bufi += 2; //skip 2 reserved bytes

        //DEBUG_PRINTF("bufi: %d \r\n",_bufi);
        uint8_t datafooter = _buf[_bufi++]; if(datafooter != 0x55){DEBUG_PRINTLN("Incorrect footer for sensor data");}
        uint8_t datacheck = _buf[_bufi++]; if(datacheck != 0x00){DEBUG_PRINTLN("Incorrect check for sensor data");}
        if(_bufi != dataLength) {DEBUG_PRINTLN("Incorrect sensor data format");}
        return true;
    }

    bool recievedResponse = false;
    bool baudSet = false;
    bool restartSet = false;
    bool engSet = false;
    bool endConfigSet = false;
    bool getCMDresponse(uint16_t dataLength){
        if(_serial.available() < dataLength) return false;
        if(_serial.readBytes(_buf, dataLength) != dataLength) return false;
        //DEBUG_PRINTF("data length: %d \r\n", dataLength);
        //printarr(_buf, dataLength);

        uint8_t _bufi = 0; //reset buffer index
        LD2410Commands command = (LD2410Commands)_buf[_bufi++]; //command byte
        if(_buf[_bufi++] != 0x01) {DEBUG_PRINTLN("Incorrect command response");}; //command response indicator; should always be 0x01
        uint16_t commandResult = 2;
        uint16_t protocolVersion = 0;
        
        DEBUG_PRINTF("command: %d \r\n", command);
        switch (command){
            case ENABLE_CONFIG: {
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              protocolVersion = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              uint16_t bufferSize = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              recievedResponse = !commandResult;
              commandResult ? Serial.println("Failed to enable config") : DEBUG_PRINTLN("Entered config mode");
              break;
            }
            case END_CONFIG: {
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              endConfigSet = !commandResult;
              commandResult ? Serial.println("Failed to end config") : DEBUG_PRINTLN("Exited config mode"); 
              break;
            }
            case MAX_GATES_AND_MIN_OUTPUT_TIME: {
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              commandResult ? Serial.println("Failed to set number of gates and minimum output time") : DEBUG_PRINTLN("Set number of gates and minimum output time");
              break;
            }
            case READ_GATES_CONFIG: {
                uint8_t bufi=2; //buffer index
                commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
                commandResult ? Serial.println("Failed to read gate config"): DEBUG_PRINTLN("Read gate config");

                uint8_t header = _buf[_bufi++]; if(header != 0xAA){DEBUG_PRINTLN("incorrect header");}
                uint8_t numberOfGates = _buf[_bufi++];
                uint8_t numberOfMotionGates = _buf[_bufi++];
                uint8_t numberOfStaticGates = _buf[_bufi++];
                uint8_t gateMotionSensitivity[MAX_GATES] = {0}; if(numberOfMotionGates >= MAX_GATES){numberOfMotionGates = MAX_GATES-1;}
                uint8_t gateStaticSensitivity[MAX_GATES] = {0}; if(numberOfStaticGates >= MAX_GATES){numberOfStaticGates = MAX_GATES-1;}
                for(uint8_t i = 0; i<= numberOfMotionGates; i++){
                    gateMotionSensitivity[i] = _buf[_bufi++];
                }
                for(uint8_t i = 0; i<= numberOfStaticGates; i++){
                    gateStaticSensitivity[i] = _buf[_bufi++];
                }
                uint16_t minimumDuration = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);
                (_bufi != dataLength) ? DEBUG_PRINTLN("Failed to read Gate sensitivity and minimum duration message format") : DEBUG_PRINTLN("Read gate sensitivities and minimum output duration");
                break;
              }
            case ENABLE_ENGINEERING_MODE: 
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              engSet = !commandResult;
              commandResult ? Serial.println("Failed to enable engineering mode") : DEBUG_PRINTLN("Entered Engineering mode");
              break;
            case DISABLE_ENGINEERING_MODE: 
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              commandResult ? Serial.println("Failed to disable engineering mode") : DEBUG_PRINTLN("Exited Engineering mode");
              break;
            case SET_GATE_SENSITIVITY: 
              commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
              commandResult ? Serial.println("Failed to set gate sensitifity") : DEBUG_PRINTLN("Set gate sensitivities");
              break;
            case READ_FIRMWARE_VERSION: {
                commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); 
                commandResult ? Serial.println("Failed to read firmware version") :  DEBUG_PRINTLN("Read firmware version");
                uint16_t firmwareVirsionType = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);
                uint16_t firmwareVirsionMajor = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]);
                uint64_t firmwareVirsionMinor = littleEndianToUINT32(_buf[_bufi++],_buf[_bufi++],_buf[_bufi++],_buf[_bufi++]);
                if(_bufi != dataLength) {DEBUG_PRINTLN("Failed to read firmware virsion message format");}
                break;
              }
            case SET_BAUD_RATE: commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); baudSet = !commandResult; if(commandResult != 0){Serial.println("Failed to set baud rate");} break;
            case RESET_SETTINGS: commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); if(commandResult != 0){Serial.println("Failed to reset settings");} break;
            case RESTART_MODULE: commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); restartSet = !commandResult; if(commandResult != 0){Serial.println("Failed to restart module");} break;
            case ENABLE_BLUETOOTH: commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); if(commandResult != 0){Serial.println("Failed to set sset bluetooth");} break; //LD2410B only
            case GET_MAC_ADDRESS: {
                commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); if(commandResult != 0){Serial.println("Failed to read MAC address");} break;
                uint8_t MACaddress[6] = {0};
                for(uint8_t i =0; i<6; ++i){
                    MACaddress[i] = _buf[_bufi++];
                }
                if(_bufi != dataLength) {DEBUG_PRINTLN("Failed to read MAC address message format");}
                break;
              }
            //case SET_BLUETOOTH_PASSWORD: commandResult = littleEndianToUINT16(_buf[_bufi++],_buf[_bufi++]); if(commandResult != 0){Serial.println("Failed to set bluetooth password");} break;
            default: break;
        }
        if(_bufi != dataLength) {DEBUG_PRINTLN("Incorrect command response format");}
        return true;
    }

    void updateState(){
        if(_serial.available() > 0){
        if(testBool6){DEBUG_PRINTF("State: %d \r\n",state);}
        switch(state){ //reading sensor configuration states
            case(ST_IDLE): {uint8_t data = getSer(); state = (data == 0xFD) ? ST_CMD1 : (data == 0xF4) ? ST_DATA1 : ST_IDLE; break;}
            case(ST_CMD1): state = (getSer() == 0xFC) ? ST_CMD2      : ST_IDLE; break;
            case(ST_CMD2): state = (getSer() == 0xFB) ? ST_CMD3      : ST_IDLE; break;
            case(ST_CMD3): state = (getSer() == 0xFA) ? ST_GETLENGTH : ST_IDLE; break;
            case(ST_GETLENGTH): state = (readuint16_t(dataLength)) ? ST_GETRESP : ST_GETLENGTH; break;
            case(ST_GETRESP): state = (getCMDresponse(dataLength)) ? ST_CMD4 : ST_GETRESP; break;
            case(ST_CMD4): state = (getSer() == 0x04) ? ST_CMD5   : ST_IDLE; break;
            case(ST_CMD5): state = (getSer() == 0x03) ? ST_CMD6   : ST_IDLE; break;
            case(ST_CMD6): state = (getSer() == 0x02) ? ST_CMD7   : ST_IDLE; break;
            case(ST_CMD7): state = (getSer() == 0x01) ? ST_ENDCMDFRAME  : ST_IDLE; break;
            case(ST_ENDCMDFRAME): state = ST_IDLE; break;
            case(ST_DATA1): state = (getSer() == 0xF3) ? ST_DATA2   : ST_IDLE; break; //reading sensor data states
            case(ST_DATA2): state = (getSer() == 0xF2) ? ST_DATA3   : ST_IDLE; break;
            case(ST_DATA3): state = (getSer() == 0xF1) ? ST_D_LENGTH: ST_IDLE; break;
            case(ST_D_LENGTH): state = (readuint16_t(dataLength)) ? ST_GETDATA : ST_D_LENGTH; break;
            case(ST_GETDATA):  getDataFrame(dataLength); state = ST_DATA4; break;
            case(ST_DATA4): state = (getSer() == 0xF8) ? ST_DATA5   : ST_IDLE; break;
            case(ST_DATA5): state = (getSer() == 0xF7) ? ST_DATA6   : ST_IDLE; break;
            case(ST_DATA6): state = (getSer() == 0xF6) ? ST_DATA7   : ST_IDLE; break;
            case(ST_DATA7): state = (getSer() == 0xF5) ? ST_ENDDATAFRAME  : ST_IDLE; break;
            case(ST_ENDDATAFRAME): 
              state = ST_IDLE; 
              if(testBool2) {DEBUG_PRINTF("sense: %03u, %03u, %03u, %03u \r\n", bri, data.movementDistance,data.stationaryDistance, data.detectionDistance);}
              if(testBool3) {printarr(data.movementGateEnergies,MAX_GATES);}
              if(testBool1){
                uint16_t calcbri = data.movementDistance;
                //uint16_t fullonDist = 30;
                //uint16_t fulloffDist = 300;
                if(calcbri < fullonDist) calcbri = fullonDist;
                if(calcbri > fulloffDist) calcbri = fulloffDist;
                calcbri = 255-(255*(calcbri-fullonDist)/(fulloffDist-fullonDist));
                bri = calcbri;
                stateUpdated(CALL_MODE_DIRECT_CHANGE);
              }
              break;
            default: state = ST_IDLE; break;
        }
        }
    }
    unsigned long comTime = 0;
    unsigned long comTimeoutms = 3000;
    void setBaud(uint8_t _baud){
      _baud = (_baud<1)? 1 : (baud>NUM_BAUD_VALUES) ? 8 : _baud; //1 indexed
      _serial.end(); //clear buffer //watchdog wdt crashes on esp32-cam but not esp32s2 mini
      state = ST_IDLE; //reset data frame state machine
      //delay(2);
      _serial.setRxBufferSize(1024);
      _serial.begin(baudTable[_baud-1],SERIAL_8N1,RXpin,TXpin); 
      _serial.setTimeout(0); //no wait for read
      
      _serial.write(CMD_ENABLE_CONFIG,sizeof(CMD_ENABLE_CONFIG)); //send config mode command
    }

    bool checkComms(){
      if(testBool5){DEBUG_PRINTF("COM state: %d %d \r\n", comState, baudold);}
      bool comReady = false;
      switch (comState){
      case COM_INIT: {
        recievedResponse = false; 
        baudSet = false;
        comTime = millis(); 
        setBaud(baudold);  //check previous baud rate
        comState = COM_WAITOLD; 
        break; //end serial and empty buffer
      }
      case COM_WAITOLD:{
        if(recievedResponse){ comState = COM_SETBAUD;}
        else if(millis() - comTime > comTimeoutms){
          baudold = baud;  //check baud rate being set to
          comTime = millis(); 
          setBaud(baudold);
          comState = COM_WAITNEW;
        }
        break;
      }
      case COM_WAITNEW:{
        if(recievedResponse){ comState = COM_SETBAUD;}
        else if(millis() - comTime > comTimeoutms){
          baudold = 1; //check first baud and increment
          comTime = millis(); 
          setBaud(baudold);
          comState = COM_FINDBAUD;
        }
        break;
      }
      case COM_FINDBAUD:{
        if(recievedResponse){ comState = COM_SETBAUD;}
        else if(millis() - comTime > comTimeoutms){
          baudold += 1;
          if(baudold > 8){comState = COM_ERROR; DEBUG_PRINTLN("No valid baud rate found. Check LD2410 pin assignment and wiring");}
          else{
            comTime = millis(); 
            setBaud(baudold);
          }
        }
        break;
      }
      case COM_SETBAUD:{
        if(baud == baudold){
          comState = COM_WAITCONFIG; //skip to setting engineering mode
        } else {
          baudSet = false;
          CMD_SET_BAUD[8] = baud; //corresponds to baud rate in baudTable
          _serial.write(CMD_SET_BAUD,sizeof(CMD_SET_BAUD)); //set baud
          comTime = millis();
          comState = COM_WAITBAUD;
        }
        break;
      }
      case COM_WAITBAUD:{
        if(baudSet){ 
          comTime = millis();
          restartSet = false;
          _serial.write(CMD_RESTART,sizeof(CMD_RESTART)); //restart module
          comState = COM_WAITRESTARTCMD;
        }else if(millis() - comTime > comTimeoutms){
          DEBUG_PRINTLN("failed to set baud rate");
          comState = COM_ERROR;
        }
        break;
      }
      case COM_WAITRESTARTCMD:{
        if(restartSet){ 
          setBaud(baud); //set to new baud
          comTime = millis();
          comState = COM_WAITRESTARTMODULE;
        }else if(millis() - comTime > comTimeoutms){
          DEBUG_PRINTLN("restart command failed");
          comState = COM_ERROR;
        }
        break;
      }
      case COM_WAITRESTARTMODULE:{
        if(state != ST_IDLE){ 
          comTime = millis();
          recievedResponse = false;
          _serial.write(CMD_ENABLE_CONFIG,sizeof(CMD_ENABLE_CONFIG)); //send config mode command
          comState = COM_WAITCONFIG;
        }else if(millis() - comTime > 3000){
          DEBUG_PRINTLN("failed to restart module");
          comState = COM_ERROR;
        }
        break;
      }
      
      case COM_WAITCONFIG:{
        if(recievedResponse){ 
          comTime = millis();
          engSet = false;
          _serial.write(CMD_ENABLE_ENGINEERING_MODE,sizeof(CMD_ENABLE_ENGINEERING_MODE));
          comState = COM_WAITENG;
        }else if(millis() - comTime > comTimeoutms){
          DEBUG_PRINTLN("failed to set config mode");
          comState = COM_ERROR;
        }
        break;
      }
      case COM_WAITENG:{
        if(engSet){ 
          comTime = millis();
          endConfigSet = false;
          _serial.write(CMD_DISABLE_CONFIG,sizeof(CMD_DISABLE_CONFIG));
          comState = COM_WAITENDCONFIG;
        }else if(millis() - comTime > comTimeoutms){
          DEBUG_PRINTLN("failed to set engineering mode");
          comState = COM_ERROR;
        }
        break;
      }
      case COM_WAITENDCONFIG:{
        if(endConfigSet){ 
          DEBUG_PRINTLN("LD2410 module ready");
          comState = COM_READY;
        }else if(millis() - comTime > comTimeoutms){
          DEBUG_PRINTLN("failed to set config mode");
          comState = COM_ERROR;
        }
        break;
      }
      case COM_READY: comReady = true; break;
      case COM_ERROR: _serial.end(); comState = COM_NOTREADY; break;
      case COM_NOTREADY: comReady = false; break;
      default: break;
      }
      return comReady;
    }

    // non WLED related methods, may be used for data exchange between usermods (non-inline methods should be defined out of class)

    /**
     * Enable/Disable the usermod
     */
    inline void enable(bool enable) { enabled = enable; }  //TODO: may need to start stop serial

    /**
     * Get usermod enabled/disabled state
     */
    inline bool isEnabled() { return enabled; }


    // methods called by WLED (can be inlined as they are called only once but if you call them explicitly define them out of class)

    /*
     * setup() is called once at boot. WiFi is not yet connected at this point.
     * readFromConfig() is called prior to setup()
     * You can use it to initialize variables, sensors or similar.
     */
    void setup() {
      // do your set-up here
      initDone = true;
    }


    /*
     * connected() is called every time the WiFi is (re)connected
     * Use it to initialize network interfaces
     */
    void connected() {
      //Serial.println("Connected to WiFi!");
    }


    /*
     * loop() is called continuously. Here you can check for events, read sensors, etc.
     * 
     * Tips:
     * 1. You can use "if (WLED_CONNECTED)" to check for a successful network connection.
     *    Additionally, "if (WLED_MQTT_CONNECTED)" is available to check for a connection to an MQTT broker.
     * 
     * 2. Try to avoid using the delay() function. NEVER use delays longer than 10 milliseconds.
     *    Instead, use a timer check as shown here.
     */
    void loop() {
      // if usermod is disabled or called during strip updating just exit
      // NOTE: on very long strips strip.isUpdating() may always return true so update accordingly
      if (!enabled || strip.isUpdating()) return;

      //if (millis() - lastTime > 10) { //limit to 10 sensor updates per second
      //  lastTime = millis();
      if(testBool4){
        uint32_t dataNum = _serial.available();

        if(dataNum > 0){
          if(dataNum > 1000){dataNum =1000; DEBUG_PRINTLN("Too much serial data");}
          for(uint32_t i = 0; i < dataNum; ++i){
            updateState(); //read and process serial data
          }
        }
        checkComms(); //must run after update state
      }
      //}
    }


    /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
    void addToJsonInfo(JsonObject& root)
    {
      // if "u" object does not exist yet wee need to create it
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      //this code adds "u":{"ExampleUsermod":[20," lux"]} to the info object
      //int reading = 20;
      //JsonArray lightArr = user.createNestedArray(FPSTR(_name))); //name
      //lightArr.add(reading); //value
      //lightArr.add(F(" lux")); //unit

      // if you are implementing a sensor usermod, you may publish sensor data
      JsonObject sensor = root[F("sensor")];
      if (sensor.isNull()) sensor = root.createNestedObject(F("sensor"));
      JsonArray dist = sensor.createNestedArray(F("distance"));
      dist.add(0); //TODO: place reading here
      dist.add(F("m"));
    }


    /*
     * addToJsonState() can be used to add custom entries to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void addToJsonState(JsonObject& root)
    {
      if (!initDone || !enabled) return;  // prevent crash on boot applyPreset()

      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) usermod = root.createNestedObject(FPSTR(_name));

      usermod[FPSTR(_enabled)] = enabled;
    }


    /*
     * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void readFromJsonState(JsonObject& root)
    {
      if (!initDone) return;  // prevent crash on boot applyPreset()

      JsonObject usermod = root[FPSTR(_name)];
      if (!usermod.isNull()) {
        // expect JSON usermod data in usermod name object: {"LD2410:{"enabled":false}"}
        if (usermod[FPSTR(_enabled)].is<bool>()) {
          enabled = usermod[FPSTR(_enabled)].as<bool>();
        }
      }
    }


    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * If you want to force saving the current state, use serializeConfig() in your loop().
     * 
     * CAUTION: serializeConfig() will initiate a filesystem write operation.
     * It might cause the LEDs to stutter and will cause flash wear if called too often.
     * Use it sparingly and always in the loop, never in network callbacks!
     * 
     * addToConfig() will make your settings editable through the Usermod Settings page automatically.
     *
     * Usermod Settings Overview:
     * - Numeric values are treated as floats in the browser.
     *   - If the numeric value entered into the browser contains a decimal point, it will be parsed as a C float
     *     before being returned to the Usermod.  The float data type has only 6-7 decimal digits of precision, and
     *     doubles are not supported, numbers will be rounded to the nearest float value when being parsed.
     *     The range accepted by the input field is +/- 1.175494351e-38 to +/- 3.402823466e+38.
     *   - If the numeric value entered into the browser doesn't contain a decimal point, it will be parsed as a
     *     C int32_t (range: -2147483648 to 2147483647) before being returned to the usermod.
     *     Overflows or underflows are truncated to the max/min value for an int32_t, and again truncated to the type
     *     used in the Usermod when reading the value from ArduinoJson.
     * - Pin values can be treated differently from an integer value by using the key name "pin"
     *   - "pin" can contain a single or array of integer values
     *   - On the Usermod Settings page there is simple checking for pin conflicts and warnings for special pins
     *     - Red color indicates a conflict.  Yellow color indicates a pin with a warning (e.g. an input-only pin)
     *   - Tip: use int8_t to store the pin value in the Usermod, so a -1 value (pin not set) can be used
     *
     * See usermod_v2_auto_save.h for an example that saves Flash space by reusing ArduinoJson key name strings
     * 
     * If you need a dedicated settings page with custom layout for your Usermod, that takes a lot more work.  
     * You will have to add the setting to the HTML, xml.cpp and set.cpp manually.
     * See the WLED Soundreactive fork (code and wiki) for reference.  https://github.com/atuline/WLED
     * 
     * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      //save these vars persistently whenever settings are saved
      /*top["great"] = userVar0;
      top["testBool"] = testBool;
      top["testInt"] = testInt;
      top["testLong"] = testLong;
      top["testULong"] = testULong;
      top["testFloat"] = testFloat;
      top["testString"] = testString;*/
      
      top["RXpin"] = RXpin;
      top["TXpin"] = TXpin;
      top["baud"] = baud;
      top["test1"] = testBool1;
      top["test2"] = testBool2;
      top["test3"] = testBool3;
      top["test4"] = testBool4;
      top["ComState"] = testBool5;
      top["DataState"] = testBool6;
      top["fullOffDistance"] = fulloffDist;
      top["fullOnDistance"] = fullonDist;
    }


    /*
     * readFromConfig() can be used to read back the custom settings you added with addToConfig().
     * This is called by WLED when settings are loaded (currently this only happens immediately after boot, or after saving on the Usermod Settings page)
     * 
     * readFromConfig() is called BEFORE setup(). This means you can use your persistent values in setup() (e.g. pin assignments, buffer sizes),
     * but also that if you want to write persistent values to a dynamic buffer, you'd need to allocate it here instead of in setup.
     * If you don't know what that is, don't fret. It most likely doesn't affect your use case :)
     * 
     * Return true in case the config values returned from Usermod Settings were complete, or false if you'd like WLED to save your defaults to disk (so any missing values are editable in Usermod Settings)
     * 
     * getJsonValue() returns false if the value is missing, or copies the value into the variable provided and returns true if the value is present
     * The configComplete variable is true only if the "exampleUsermod" object and all values are present.  If any values are missing, WLED will know to call addToConfig() to save them
     * 
     * This function is guaranteed to be called on boot, but could also be called every time settings are updated
     */
    bool firstConfig = true;
    bool readFromConfig(JsonObject& root)
    {
      // store old values before reading new:
      uint8_t RXold = RXpin;
      uint8_t TXold = TXpin;
      bool enabledold = enabled;
      
      // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
      // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)

      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, false);

      // "pin" fields have special handling in settings page (or some_pin as well)
      configComplete &= getJsonValue(top["RXpin"], RXpin, -1);
      configComplete &= getJsonValue(top["TXpin"], TXpin, -1);
      configComplete &= getJsonValue(top["baud"], baud, 7);
      configComplete &= getJsonValue(top["test1"], testBool1);  
      configComplete &= getJsonValue(top["test2"], testBool2);  
      configComplete &= getJsonValue(top["test3"], testBool3);  
      configComplete &= getJsonValue(top["test4"], testBool4);
      configComplete &= getJsonValue(top["ComState"], testBool5);
      configComplete &= getJsonValue(top["DataState"], testBool6);
      configComplete &= getJsonValue(top["fullOffDistance"], fulloffDist, 400);
      configComplete &= getJsonValue(top["fullOnDistance"], fullonDist, 30);

      fullonDist = (fullonDist < 0) ? 0 : (fullonDist > 550) ? 550 : fullonDist; 
      fulloffDist = (fulloffDist <= fullonDist) ? fullonDist+1 : (fulloffDist > 600) ? 600 : fulloffDist; 


      if(firstConfig){
        baudold = baud; //ignore default value on startup
        firstConfig = false;
      }

      if(enabled != enabledold || RXpin != RXold || TXpin != TXold || baud != baudold){ //if a change occurs or after startup
        //sanitize inputs
        if(baud > 8) {baud = 8;}
        else if(baud < 1) {baud = 1;}
        
        DEBUG_PRINTF("Settings changed to: EN:%lu RX: %li, TX: %li, baud: %lu \r\n from: EN:%lu RX: %li, TX: %li, baud: %lu \r\n", enabled, RXpin, TXpin, baud, enabledold, RXold, TXold, baudold);

        //release uart
        _serial.end();
        //release pins
        uint8_t pinsold[2] = {RXold, TXold}; 
        if(!pinManager.deallocateMultiplePins( pinsold, 2, PinOwner::UM_LD2410)){
          DEBUG_PRINTLN("LD2410 failed to release UART pins");
        }
        pinsReady = false;

        //setup uart
        PinManagerPinType pins[2] = {{ RXpin, false },{ TXpin, true }};
        if(enabled){
          //allocate pins
          pinsReady = pinManager.allocateMultiplePins(pins,2, PinOwner::UM_LD2410);
          comState = COM_INIT; //reset communication setup
        }
      }
      return configComplete;
    }


    /*
     * appendConfigData() is called when user enters usermod settings page
     * it may add additional metadata for certain entry fields (adding drop down is possible)
     * be careful not to add too much as oappend() buffer is limited to 3k
     */
    void appendConfigData()
    {
      oappend(SET_F("dd=addDropdown('")); oappend(String(FPSTR(_name)).c_str()); oappend(SET_F("','baud');"));
      oappend(SET_F("addOption(dd,'9600',1);"));
      oappend(SET_F("addOption(dd,'19200',2);"));
      oappend(SET_F("addOption(dd,'38400',3);"));
      oappend(SET_F("addOption(dd,'57600',4);"));
      oappend(SET_F("addOption(dd,'115200',5);"));
      oappend(SET_F("addOption(dd,'230400',6);"));
      oappend(SET_F("addOption(dd,'256000',7);"));
      oappend(SET_F("addOption(dd,'460800',8);"));
    }


    /*
     * handleOverlayDraw() is called just before every show() (LED strip update frame) after effects have set the colors.
     * Use this to blank out some LEDs or set them to a different color regardless of the set effect mode.
     * Commonly used for custom clocks (Cronixie, 7 segment)
     */
    void handleOverlayDraw()
    {
      //strip.setPixelColor(0, RGBW32(0,0,0,0)) // set the first pixel to black
    }


    /**
     * handleButton() can be used to override default button behaviour. Returning true
     * will prevent button working in a default way.
     * Replicating button.cpp
     */
    bool handleButton(uint8_t b) {
      yield();
      // ignore certain button types as they may have other consequences
      if (!enabled
       || buttonType[b] == BTN_TYPE_NONE
       || buttonType[b] == BTN_TYPE_RESERVED
       || buttonType[b] == BTN_TYPE_PIR_SENSOR
       || buttonType[b] == BTN_TYPE_ANALOG
       || buttonType[b] == BTN_TYPE_ANALOG_INVERTED) {
        return false;
      }

      bool handled = false;
      // do your button handling here
      return handled;
    }
  

#ifndef WLED_DISABLE_MQTT
    /**
     * handling of MQTT message
     * topic only contains stripped topic (part after /wled/MAC)
     */
    bool onMqttMessage(char* topic, char* payload) {
      // check if we received a command
      //if (strlen(topic) == 8 && strncmp_P(topic, PSTR("/command"), 8) == 0) {
      //  String action = payload;
      //  if (action == "on") {
      //    enabled = true;
      //    return true;
      //  } else if (action == "off") {
      //    enabled = false;
      //    return true;
      //  } else if (action == "toggle") {
      //    enabled = !enabled;
      //    return true;
      //  }
      //}
      return false;
    }

    /**
     * onMqttConnect() is called when MQTT connection is established
     */
    void onMqttConnect(bool sessionPresent) {
      // do any MQTT related initialisation here
      //publishMqtt("I am alive!");
    }
#endif


    /**
     * onStateChanged() is used to detect WLED state change
     * @mode parameter is CALL_MODE_... parameter used for notifications
     */
    void onStateChange(uint8_t mode) {
      // do something if WLED state changed (color, brightness, effect, preset, etc)
    }


    /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
    uint16_t getId()
    {
      return USERMOD_ID_LD2410;
    }

   //More methods can be added in the future, this example will then be extended.
   //Your usermod will remain compatible as it does not need to implement all methods from the Usermod base class!
};


// add more strings here to reduce flash memory usage
const char LD2410::_name[]    PROGMEM = "LD2410";
const char LD2410::_enabled[] PROGMEM = "enabled";


// implementation of non-inline member methods

void LD2410::publishMqtt(const char* state, bool retain)
{
#ifndef WLED_DISABLE_MQTT
  //Check if MQTT Connected, otherwise it will crash the 8266
  if (WLED_MQTT_CONNECTED) {
    char subuf[64];
    strcpy(subuf, mqttDeviceTopic);
    strcat_P(subuf, PSTR("/example"));
    mqtt->publish(subuf, 0, retain, state);
  }
#endif
}
