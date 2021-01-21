#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


//Configuration 
const char* ssid = "WIFI-SSID";
const char* password =  "WIFI-PASS";

const char* mqttServer = "BROKER-IP";
const int mqttPort = 1883;
const char* clientId = "ESP8266-ID";
const char* mqttUser = "-";
const char* mqttPassword = "-";

const char* dataOutTopic = "/solar/data_to_node_red";
const char* dataInTopic = "/solar/data_from_node_red";

boolean newData = false;
const byte dataLength = 22;
char receivedChars[dataLength]; 
WiFiClient EspWiFiClient;
PubSubClient client(EspWiFiClient);

void callback(char* topic, byte* payload, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  } 
}

void receiveData() {
  static byte index = 0;
  char single_char;
  char endMarker = '\n';

  while (Serial.available() > 0 && newData == false) {
  single_char = Serial.read();

    if (single_char != endMarker) {
      receivedChars[index] = single_char;
      index++;
      if (index >= dataLength) {
        index = 0;
      }
    }
    else {
      receivedChars[index] = '\0';
      index = 0;
      newData = true;
    }
  }
}
 
void setup() {
  pinMode(02, OUTPUT);
  digitalWrite(02, HIGH);

  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
 
    if (client.connect(clientId, mqttUser, mqttPassword )) {
      digitalWrite(02, LOW);
    } 
    else {
      digitalWrite(02, HIGH);
      delay(1000);
    }
  }
 
  client.subscribe(dataInTopic);
}
 
void loop() {

  if(Serial.available() > 0){
    receiveData();
    if (newData == true) {
      client.publish(dataOutTopic, receivedChars);
      newData = false;
    }
  }
  client.loop();
}