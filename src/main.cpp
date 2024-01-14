#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#define LENGTH(x) (strlen(x) + 1) 
#define DEVICE_NAME "esp-motion-thing"
#define EEPROM_SIZE 200

#define LD2410_OUT_PIN 16
// #define LD2410_RX_PIN 18
// #define LD2410_TX_PIN 16

// webthings
#include <WebThingAdapter.h>
#include <Thing.h>

// Network
#include <WiFi.h>

// #include <ESPAsyncWebServer.h>

// AsyncWebServer server(80);

WebThingAdapter *adapter;
const char *motionSensorTypes[] = {"MotionSensor", nullptr};
ThingDevice motionSensor("motion", DEVICE_NAME, motionSensorTypes);
ThingProperty motionProp("Motion", "Motion", BOOLEAN, "MotionProperty");

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
  //WiFi.mode(WIFI_AP_STA);
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

  motionSensor.id = deviceName;
  motionSensor.title = deviceName;
  motionSensor.addProperty(&motionProp);

  adapter = new WebThingAdapter(deviceName, WiFi.localIP(), 80, true);
  adapter->addDevice(&motionSensor);
  adapter->begin();
}

void checkProps() {
  // MDNS.update();
  
  uint out = digitalRead(LD2410_OUT_PIN);

  ThingPropertyValue newValue;
  newValue.boolean = out;
  
  motionProp.setValue(newValue);

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

  Serial.begin(9600);

  if (!EEPROM.begin(EEPROM_SIZE)) { 
    Serial.println("Failed to init EEPROM");
    while(1) {
      delay(1000);
    }
  }

  setupWiFi(deviceName);

  setupWebThing(deviceName);

  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  //   request->send(200, "text/plain", "Hello World");
  // });   
  // server.begin();
}

void loop() {
  checkProps();
}
