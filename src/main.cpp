#include <Arduino.h>
#include <OpenTherm.h>

#ifndef DEVICE_BOARD_NAME
#  define DEVICE_BOARD_NAME "OpenthermSlave"
#endif

#define DEVICE_HOSTNAME "OThermGW"

#include "controlWiFi.h"

#include "MQTT_task.h"

#include <esp_task_wdt.h>
#define WDT_TIMEOUT 60


//Port if Built-in LED
#define LED_BUILTIN 2

//Port of relay shield
#define RELAY_PORT D1

//Global variables in Master<>Slave communication
float SetPoint = 0;
bool FlameOn = false;
float MaxModulationLevel = 0;
float RoomSetPoint = 0;
float RoomTemperature = 0;

hw_timer_t *Timer0_Cfg = NULL;


const int inPin = 19;  //for Arduino, 12 for ESP8266 (D6), 19 for ESP32
const int outPin = 23; //for Arduino, 13 for ESP8266 (D7), 23 for ESP32
OpenTherm ot(inPin, outPin, true);

void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

bool publishMQTT = false;
void IRAM_ATTR Timer0_ISR()
{
  publishMQTT = true;
}

void processRequest(unsigned long request, OpenThermResponseStatus status) {
    //Serial.println("T" + String(request, HEX)); //master/thermostat request

    unsigned long response = 0;
    OpenThermMessageID id = ot.getDataID(request);
    uint16_t data = ot.getUInt(request);
    float f = ot.getFloat(request);
    switch(id)
    {
      //message ID 0 Master and slave status flags
      case OpenThermMessageID::Status:
      {
        Serial.println("Master/Slave status flags");
        Serial.println("Received data: " + String(data));
        uint8_t statusRequest = data >> 8;
        Serial.println("statusRequest: " + String(statusRequest));
        uint8_t chEnable = statusRequest & 0x1;
        Serial.println("chEnable: " + String(chEnable));
        data &= 0xFF00;
        if (chEnable) {
          data |= 0x02; //CH active
          data |= 0x08; //flame on
          FlameOn = true;
          digitalWrite(RELAY_PORT, LOW);
          Serial.println("Flame is On");
        } 
        else {
          FlameOn = false;
          digitalWrite(RELAY_PORT, HIGH);
          Serial.println("Flame is Off");
        }
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        Serial.println("Responded data: " + String(data));
        break;
      }

      //message ID 1 Control setpoint ie CH water temp. setpoint
      case OpenThermMessageID::TSet:
      {
        Serial.println("Gotten control setpoint: " + String(f));
        SetPoint = f;
        response = ot.buildResponse(OpenThermMessageType::WRITE_ACK, id, data);
        break;
      }

      //message ID 2 HB: Master configuration
      case OpenThermMessageID::MConfigMMemberIDcode:
      {
        data &= 0xFF00;
        response = ot.buildResponse(OpenThermMessageType::WRITE_ACK, id, data);
        break;
      }

      //message ID 3 Slave configuration flags
      case OpenThermMessageID::SConfigSMemberIDcode:
      {
        Serial.print("Slave configuration flags: ");
        data &= 0xFF00;
        //data |= 0x01; //DHW Present
        data |= 0x02; //Control Type
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        Serial.println(String(data));
        break;
      }

      //message ID 5 HB: Application-specific fault flags
      case OpenThermMessageID::ASFflags:
      {
        data &= 0xFF00;
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 6 HB: Remote-parameter transfer-enable flags
      case OpenThermMessageID::RBPflags:
      {
        data &= 0xFF00;
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }


      //message ID 9 Remote override room setpoint
      case OpenThermMessageID::TrOverride:
      {
        data = ot.temperatureToData(0);
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 12  HB: Size of Fault Buffer
      case OpenThermMessageID::FHBsize:
      {
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, 0);
        break;
      }

      //message ID 14 Maximum relative modulation level setting
      case OpenThermMessageID::MaxRelModLevelSetting:
      {
        MaxModulationLevel = f;
        response = ot.buildResponse(OpenThermMessageType::WRITE_ACK, id, data);
        break;
      }

      //message ID 16  Room Setpoint
      case OpenThermMessageID::TrSet:
      {
        RoomSetPoint = f;
        response = ot.buildResponse(OpenThermMessageType::WRITE_ACK, id, data);
        break;
      }

      //message ID 17 Relative modulation lev
      case OpenThermMessageID::RelModLevel:
      {
        float modulation = 50;
        Serial.println("Requested Current Modulation Level");
        Serial.println("Sent constant " + String(modulation) + " %");
        data = ot.temperatureToData(modulation);
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 18 CH water pressure
      case OpenThermMessageID::CHPressure:
      {
        float pressure = 1;
        Serial.println("Requested  CH water pressure");
        Serial.println("Sent constant " + String(pressure) + " bar");
        data = ot.temperatureToData(pressure);
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 19 DHW flow rate
      case OpenThermMessageID::DHWFlowRate:
      {
        Serial.println("Requested DHW flow rate");
        data = ot.temperatureToData(0);
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        Serial.println("Sent constant " + 0);
        break;
      }

      //message ID 24 Room Temperature
      case OpenThermMessageID::Tr:
      {
        Serial.println("Got room temperature " + String(f));
        RoomTemperature = f;
        response = ot.buildResponse(OpenThermMessageType::WRITE_ACK, id, data);
        break;
      }

      //message ID 25 Boiler temperature
      case OpenThermMessageID::Tboiler:
      {
        data = ot.temperatureToData(45);
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 26 DHW temperature
      case OpenThermMessageID::Tdhw:
      {
        float DHWTemperature = 20;
        data = ot.temperatureToData(DHWTemperature);
        Serial.println("Requested  DHW temperature");
        Serial.println("Sent constant " + String(DHWTemperature) + " C");
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 27 Outside temperature
      case OpenThermMessageID::Toutside:
      {
        data = ot.temperatureToData(OutsideTemperature);
        Serial.println("Requested  outside temperature");
        Serial.println("Sent " + String(OutsideTemperature) + " C");
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      //message ID 28 Return water temperature
      case OpenThermMessageID::Tret:
      {
        float ReturnTemperature = 35;
        data = ot.temperatureToData(ReturnTemperature);
        Serial.println("Requested  return boiler temperature");
        Serial.println("Sent constant " + String(ReturnTemperature) + " C");
        response = ot.buildResponse(OpenThermMessageType::READ_ACK, id, data);
        break;
      }

      default:
      {
        Serial.println("Undefined MessageID: " + String(id));
        //Serial.println("data" + String(data));
        //Serial.println("f" + String(f));
        //build UNKNOWN-DATAID response
        response = ot.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, ot.getDataID(request), 0);   
      }
    }
    //Serial.println("B" + String(response, HEX)); //slave/boiler response

    Serial.println("-------------------------------");
    delay(20); //20..400ms, usually 100ms
    ot.sendResponse(response);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Start");

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RELAY_PORT, OUTPUT);
  digitalWrite(RELAY_PORT, HIGH);

  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:
      Serial.println("Reset due to power-on event");
    break;
  
    case ESP_RST_SW:
      Serial.println("Software reset via esp_restart");
    break;

    case ESP_RST_WDT:
      Serial.println("Rebooted by Watchdog!");
    break;
  }

  //Set witchdog timeout for 32 seconds
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  while (esp_task_wdt_status(NULL) != ESP_OK) {
    // LED blinks indefinitely
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  Serial.print("Start WiFi on ");
  Serial.println(DEVICE_BOARD_NAME);
  
  initializeWiFi(DEVICE_HOSTNAME);
  
  establishWiFi();

  // you're connected now, so print out the data
  printWifiStatus();

  initMQTT();

  //Setup Hardware Timer for MQTT publish
  Timer0_Cfg = timerBegin(0, 300, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 10000000, true);
  timerAlarmEnable(Timer0_Cfg);

  ot.begin(handleInterrupt, processRequest);
}


void loop()
{
  ot.process();
  MQTTLoop();
  if (publishMQTT)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    MQTTMessageCallback(SetPoint, FlameOn, MaxModulationLevel, RoomSetPoint, RoomTemperature);
    digitalWrite(LED_BUILTIN, LOW);
    publishMQTT = false;
    esp_task_wdt_reset();
  }
}