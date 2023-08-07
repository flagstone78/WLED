#pragma once
#include <byteswap.h>
#include "wled.h"
uint8_t CMD_ENABLE_CONFIG[] = {0xfd,0xfc,0xfb,0xfa,0x04,0x00,0xff,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
uint8_t CMD_ENABLE_ENGINEERING_MODE[] = {0xfd,0xfc,0xfb,0xfa,0x02,0x00,0x62,0x00,0x04,0x03,0x02,0x01};
uint8_t CMD_DISABLE_CONFIG[] = {0xfd,0xfc,0xfb,0xfa,0x02,0x00,0xfe,0x00,0x04,0x03,0x02,0x01};

struct LD2410GateFormat{
  uint8_t gate1;
  uint8_t gate2;
  uint8_t gate3;
  uint8_t gate4;
  uint8_t gate5;
  uint8_t gate6;
  uint8_t gate7;
  uint8_t gate8;
};

struct LD2410EngFormat {
  uint32_t header;
  uint16_t length;
  uint8_t  isEngineeringMode;
  uint8_t  innerHeader;
  uint8_t  targetState; //0 - no target, 1 movment, 2 stationary, 3 both
  uint16_t movementDistance;
  uint8_t  movementEnergy;
  uint16_t stationaryDistance;
  uint8_t  stationaryEnergy;
  uint16_t detectionDistance; // combined of the 2????
  uint8_t  movementGateCount; // sould be 8
  uint8_t  stationaryGateCount; // sould be 8
  LD2410GateFormat movementGateEnergies; //8 bytes each
  LD2410GateFormat stationaryGateEnergies;
  uint8_t unknown1; //datasheet says reserved or store extra information
  uint8_t unknown2;
  uint8_t unknown3;
  uint8_t unknown4;
  uint8_t innerFooter;
  uint8_t check; //unknown function
  uint32_t footer;
}__attribute__((__packed__));;
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

    // Private class members. You can declare variables and functions only accessible to your usermod here
    bool enabled = false;
    bool initDone = false;
    unsigned long lastTime = 0;

    // set your config variables to their boot default value (this can also be done in readFromConfig() or a constructor if you prefer)
    /*bool testBool = false;
    unsigned long testULong = 42424242;
    float testFloat = 42.42;
    String testString = "Forty-Two";*/
    int8_t RXpin = -1;
    int8_t TXpin = -1;
    HardwareSerial _serial = Serial0;
    uint8_t _buf[64] = {0};
    LD2410EngFormat* data = (LD2410EngFormat*) _buf;
    uint calcbri=0;

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
      Serial.print("header: "); Serial.print("0x"); Serial.print(data->header,HEX); Serial.print(' '); Serial.println(data->header);
      Serial.print("length: "); Serial.print("0x"); Serial.print(data->length,HEX); Serial.print(' '); Serial.println(data->length);
      Serial.print("isEngineeringMode: "); Serial.print("0x"); Serial.print(data->isEngineeringMode,HEX); Serial.print(' '); Serial.println(data->isEngineeringMode);
      Serial.print("innerHeader: "); Serial.print("0x"); Serial.print(data->innerHeader,HEX); Serial.print(' '); Serial.println(data->innerHeader);
      Serial.print("targetState: "); Serial.print("0x"); Serial.print(data->targetState,HEX); Serial.print(' '); Serial.println(data->targetState);
      Serial.print("movmentDistance: "); Serial.print("0x"); Serial.print(data->movementDistance,HEX); Serial.print(' '); Serial.println(data->movementDistance);
      Serial.print("movementEnergy: "); Serial.print("0x"); Serial.print(data->movementEnergy,HEX); Serial.print(' '); Serial.println(data->movementEnergy);
      Serial.print("stationaryDistance: "); Serial.print("0x"); Serial.print(data->stationaryDistance,HEX); Serial.print(' '); Serial.println(data->stationaryDistance);
      Serial.print("stationaryEnergy: "); Serial.print("0x"); Serial.print(data->stationaryEnergy,HEX); Serial.print(' '); Serial.println(data->stationaryEnergy);
      Serial.print("detectionDistance: "); Serial.print("0x"); Serial.print(data->detectionDistance,HEX); Serial.print(' '); Serial.println(data->detectionDistance);
      Serial.print("movementGateCount: "); Serial.print("0x"); Serial.print(data->movementGateCount,HEX); Serial.print(' '); Serial.println(data->movementGateCount);
      Serial.print("stationaryGateCount: "); Serial.print("0x"); Serial.print(data->stationaryGateCount,HEX); Serial.print(' '); Serial.println(data->stationaryGateCount);
      
      //Serial.printf("header %08x \n", data->header);
      //Serial.printf("headerdec %u \n", data->header);
      //Serial.printf("headeracc %u \n", 0xF1F2F3F4);
      //Serial.println(data->header == 0xF1F2F3F4);
    }

  public:

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

      /*if (millis() - lastTime > 1000) {
        //Serial.println("I'm alive!");
        //calcbri+=10;
        lastTime = millis();
      }*/

      // do your magic here
      while(enabled && _serial.available() >= 45){
        _serial.readBytes(_buf,45);

        //printarr(_buf,45);
        if(data->header == 0xF1F2F3F4u){ //if data valid
          //printdata();
          if(!(data->targetState & 0x01)){data->movementDistance = 255;}
          if(!(data->targetState & 0x02)){data->stationaryDistance = 0;}
          
          calcbri = data->movementDistance;
          if(calcbri >= 30){calcbri -= 30;}
          calcbri *= 2;
          if(calcbri >= 255){calcbri = 255;}
          calcbri = 255-calcbri;

        if(bri != calcbri){
          bri = 255;//calcbri;
          stateUpdated(CALL_MODE_DIRECT_CHANGE);
        }
          //Serial.printf("sense: %03u, %03u, %03u, %03u \n", bri, data->movementDistance,data->stationaryDistance, data->detectionDistance);
        } else {_serial.read();}  //shift by one and try on next
      }
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
      /*JsonArray pinArray = top.createNestedArray("pin");
      pinArray.add(serialPins[0]); //RX
      pinArray.add(serialPins[1]); //TX*/
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

      /*configComplete &= getJsonValue(top["great"], userVar0);
      configComplete &= getJsonValue(top["testBool"], testBool);
      configComplete &= getJsonValue(top["testULong"], testULong);
      configComplete &= getJsonValue(top["testFloat"], testFloat);
      configComplete &= getJsonValue(top["testString"], testString);

      // A 3-argument getJsonValue() assigns the 3rd argument as a default value if the Json value is missing
      configComplete &= getJsonValue(top["testInt"], testInt, 42);  
      configComplete &= getJsonValue(top["testLong"], testLong, -42424242);
      */
      // "pin" fields have special handling in settings page (or some_pin as well)
      configComplete &= getJsonValue(top["RXpin"], RXpin, -1);
      configComplete &= getJsonValue(top["TXpin"], TXpin, -1);

      if(enabled != enabledold || RXpin != RXold || TXpin != TXold){ //if a change occurs or after startup
        DEBUG_PRINTF("Settings changed: EN:%lu RX: %lu, TX: %lu \n", enabled, RXpin, TXpin);
        //release uart
        _serial.end();
        //release pins
        uint8_t pinsold[2] = {RXold, TXold}; 
        pinManager.deallocateMultiplePins( pinsold, 2, PinOwner::UM_LD2410);

        //setup uart
        PinManagerPinType pins[2] = {{ RXpin, false },{ TXpin, true }};
        if(enabled){
          //allocate pins
          if (pinManager.allocateMultiplePins(pins,2, PinOwner::UM_LD2410)) {
            //setup serial
            _serial.setTimeout(0); //no wait for read
            _serial.begin(256000,SERIAL_8N1,RXpin,TXpin);
            _serial.write(CMD_ENABLE_CONFIG,sizeof(CMD_ENABLE_CONFIG));
            _serial.readBytes(_buf,18);
            Serial.print("enable config:");
            printarr(_buf, 18);

            _serial.write(CMD_ENABLE_ENGINEERING_MODE,sizeof(CMD_ENABLE_ENGINEERING_MODE));
            _serial.readBytes(_buf,14);
            Serial.print("enable engine:");
            printarr(_buf, 14);


            _serial.write(CMD_DISABLE_CONFIG,sizeof(CMD_DISABLE_CONFIG));
            _serial.readBytes(_buf,14);
            Serial.print("diable config:");
            printarr(_buf, 14);

            //setup ld2410
          }
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
