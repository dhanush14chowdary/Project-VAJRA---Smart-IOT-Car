#include <dummy.h>

#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "MrRobot";
const char* password = "123456789";
const char* mqttServer = "43.204.228.73";
const int mqttPort = 1883;
const char* mqttControlTopic = "car/control"; // Topic to listen for control commands
const char* mqttDataTopic = "car/data";       // Topic to publish sensor data

const char* mqttUser = "admin";
const char* mqttPassword = "nasal@airflow";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);                     // Use default UART for both Arduino and logging

  WiFi.begin(ssid, password);             // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  client.setServer(mqttServer, mqttPort); // Set MQTT server
  client.setCallback(callback);           // Define callback function

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT Broker");
      client.subscribe(mqttControlTopic); // Subscribe to control topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received: ");
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  Serial.println(message);

  // Send the control command to Arduino via UART
  Serial.println(message);               
}

void loop() {
  client.loop();                         // Handle incoming messages

  //Check for sensor data from Arduino
  if (Serial.available()) {
    String sensorData = Serial.readStringUntil('\n');
    sensorData.trim();
    if (sensorData.length() > 0) {
      Serial.println("Received sensor data: " + sensorData);
      client.publish(mqttDataTopic, sensorData.c_str()); // Publish sensor data to MQTT
    }
  }
}