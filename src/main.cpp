#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#define LENGTH(x) (strlen(x) + 1) 
#define DEVICE_NAME "esp-motion-thing"
#define EEPROM_SIZE 200

#define MONITOR_SERIAL Serial

#define RADAR_SERIAL Serial1
#define LD2410_OUT_PIN 16
#define LD2410_RX_PIN 33
#define LD2410_TX_PIN 18
#define REPORT_INTERVAL_MS 100

// webthings
#include <WebThingAdapter.h>
#include <Thing.h>

// Network
#include <WiFi.h>
#include <ESPmDNS.h>

// OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// LD2410
#include <ld2410.h>

WebThingAdapter *adapter;
const char *motionSensorTypes[] = {"MotionSensor", nullptr};
ThingDevice motionSensor("motion", DEVICE_NAME, motionSensorTypes);
ThingProperty motionProp("Motion", "Motion", BOOLEAN, "MotionProperty");
ThingProperty stationaryTargetDistanceProp("StationaryTargetDistance", "StationaryTargetDistance", NUMBER, "CustomProperty");
ThingProperty stationaryTargetEnergyProp("StationaryTargetEnergy", "StationaryTargetEnergy", NUMBER, "CustomProperty");
ThingProperty movingTargetDistanceProp("MovingTargetDistance", "MovingTargetDistance", NUMBER, "CustomProperty");
ThingProperty movingTargetEnergyProp("MovingTargetEnergy", "MovingTargetEnergy", NUMBER, "CustomProperty");


ld2410 radar;
uint32_t lastReading = 0;

void setupRadar() {
  RADAR_SERIAL.begin(256000, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);

  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("OK"));
    MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    MONITOR_SERIAL.print(radar.firmware_major_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.print(radar.firmware_minor_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR_SERIAL.println(F("not connected"));
  }
}

void write_flash(const char* toStore, int startAddr) {
  int i = 0;
  for (; i < LENGTH(toStore); i++) {
    EEPROM.write(startAddr + i, toStore[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}


String read_flash(int startAddr) {
  char in[128]; 
  int i = 0;
  for (; i < 128; i++) {
    in[i] = EEPROM.read(startAddr + i);
  }
  return String(in);
}

void setupWiFi(String deviceName) {
  String ssid;                       
  String password;

  WiFi.mode(WIFI_STA);
  WiFi.hostname(deviceName.c_str());
  WiFi.setAutoReconnect(true);

  bool blink = true;

  ssid = read_flash(0); 
  password = read_flash(40);
  

  WiFi.begin(ssid.c_str(), password.c_str());

  delay(3500);
  
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.beginSmartConfig();
    
    while (!WiFi.smartConfigDone()) {
      delay(1000);
      digitalWrite(LED_BUILTIN, blink ? HIGH : LOW);
      blink = !blink;
    }

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      digitalWrite(LED_BUILTIN, blink ? HIGH : LOW);
      blink = !blink;
    }

    ssid = WiFi.SSID();
    password = WiFi.psk();

    write_flash(ssid.c_str(), 0); 
    write_flash(password.c_str(), 40); 
  }

  WiFi.persistent(true);
  WiFi.stopSmartConfig();

  digitalWrite(LED_BUILTIN, HIGH);
}

void setupWebThing(String deviceName) {
  motionProp.readOnly = true;
  motionProp.title = "Motion";

  stationaryTargetDistanceProp.readOnly = true;
  stationaryTargetDistanceProp.title = "Stat distance cm";

  stationaryTargetEnergyProp.readOnly = true;
  stationaryTargetEnergyProp.title = "Stat energy";

  movingTargetDistanceProp.readOnly = true;
  movingTargetDistanceProp.title = "Moving distance cm";

  movingTargetEnergyProp.readOnly = true;
  movingTargetEnergyProp.title = "Moving energy";
  

  motionSensor.id = deviceName;
  motionSensor.title = deviceName;
  motionSensor.addProperty(&motionProp);
  motionSensor.addProperty(&stationaryTargetDistanceProp);
  motionSensor.addProperty(&stationaryTargetEnergyProp);
  motionSensor.addProperty(&movingTargetDistanceProp);
  motionSensor.addProperty(&movingTargetEnergyProp);

  adapter = new WebThingAdapter(deviceName, WiFi.localIP(), 80, true);
  adapter->addDevice(&motionSensor);
  adapter->begin();
}

void setupOTA(const char* deviceName) {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(deviceName);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");


  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void readRadar() {
  radar.read();
  if(radar.isConnected() && millis() - lastReading > REPORT_INTERVAL_MS)
  {

    uint out = digitalRead(LD2410_OUT_PIN);

    ThingPropertyValue newValue;
    newValue.boolean = out;    
    motionProp.setValue(newValue);

    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDetected())
      {

        uint16_t stationaryDistance = radar.stationaryTargetDistance();

        ThingPropertyValue stationaryDistanceValue;
        stationaryDistanceValue.number = stationaryDistance;    
        stationaryTargetDistanceProp.setValue(stationaryDistanceValue);

        uint8_t statinaryEnergy = radar.stationaryTargetEnergy();

        ThingPropertyValue stationaryEnergyValue;
        stationaryEnergyValue.number = statinaryEnergy;    
        stationaryTargetEnergyProp.setValue(stationaryEnergyValue);

        Serial.print(F("Stationary target: "));
        Serial.print(stationaryDistance);
        Serial.print(F("cm energy:"));
        Serial.print(statinaryEnergy);
        Serial.print(' ');
      }
      if(radar.movingTargetDetected())
      {
        uint16_t movingDistance = radar.movingTargetDistance();

        ThingPropertyValue movingDistanceValue;
        movingDistanceValue.number = movingDistance;    
        movingTargetDistanceProp.setValue(movingDistanceValue);

        uint8_t movingEnergy = radar.movingTargetEnergy();

        ThingPropertyValue movingEnergyValue;
        movingEnergyValue.number = movingEnergy;    
        movingTargetEnergyProp.setValue(movingEnergyValue);

        Serial.print(F("Moving target: "));
        Serial.print(movingDistance);
        Serial.print(F("cm energy:"));
        Serial.print(movingEnergy);
      }
      Serial.println();
    }
    else
    {
      Serial.println(F("No target"));
    }
  }

  adapter->update();
}

void setup() {
  setCpuFrequencyMhz(80);

  uint32_t chipId = 0;
  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}

  String deviceName(DEVICE_NAME);
  deviceName.concat("-");
  deviceName.concat(chipId);
  deviceName.toLowerCase();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LD2410_OUT_PIN, INPUT);

  Serial.begin(115200);

  if (!EEPROM.begin(EEPROM_SIZE)) { 
    Serial.println("Failed to init EEPROM");
    while(1) {
      delay(1000);
    }
  }

  setupWiFi(deviceName);

  setupWebThing(deviceName);

  setupOTA(deviceName.c_str());

  setupRadar();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  ArduinoOTA.handle();
  readRadar();
}

