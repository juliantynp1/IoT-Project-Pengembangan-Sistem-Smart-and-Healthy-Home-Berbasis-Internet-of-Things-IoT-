#include <WiFi.h>
#include <PubSubClient.h>  // Library MQTT untuk ThingsBoard
#include <ESP32Servo.h>
#include <Stepper.h>
#include <ArduinoJson.h>    // Tambahkan library ArduinoJson

// Definisikan pin untuk setiap sensor
const int ldrPin = 15;       
const int rainPin = 20;      
const int flamePin = 35;     
const int trigPin = 17;      // Pin untuk sensor Ultrasonic Trig
const int echoPin = 18;      // Pin untuk sensor Ultrasonic Echo
const int flowPin = 21;      
const int soilMoisturePin = 16; 
const int pumpPin = 13;      
const int servoPin = 4;      
const int lampPin = 20;      

// Motor Stepper 28BYJ-48
const int stepsPerRevolution = 2048; // Jumlah langkah untuk satu putaran

// Pin untuk motor stepper
const int stepperPin1 = 10;          
const int stepperPin2 = 11;          
const int stepperPin3 = 12;          
const int stepperPin4 = 13;          

// WiFi Settings
const char* ssid = "BUS.ID";      // Ganti dengan nama WiFi Anda
const char* password = "clean123456"; // Ganti dengan password WiFi Anda

// ThingsBoard Settings
const char* mqtt_server = "demo.thingsboard.io"; // Ganti dengan server ThingsBoard Anda jika perlu
const char* access_token = "9K57Qzhdo9IaPgEnzQSi"; // Ganti dengan Access Token perangkat Anda

WiFiClient espClient;
PubSubClient client(espClient);

Servo myServo;  
Stepper myStepper(stepsPerRevolution, stepperPin1, stepperPin3, stepperPin2, stepperPin4);

volatile int flowCount = 0;  
float flowRate = 0;          
unsigned long lastTime = 0;  

void IRAM_ATTR countFlow() {
  flowCount++;  
}

void setup() {
  Serial.begin(115200);
  pinMode(ldrPin, INPUT);
  pinMode(rainPin, INPUT);
  pinMode(flamePin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(flowPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(lampPin, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(0);  

  myStepper.setSpeed(10); 

  attachInterrupt(digitalPinToInterrupt(flowPin), countFlow, FALLING);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to ThingsBoard MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("ESP32Client", access_token, NULL)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // This function handles incoming messages (if needed)
}

void loop() {
  client.loop();  // Keep MQTT connection alive

  // Membaca status sensor LDR, Raindrop, dan Flame
  int ldrValue = analogRead(ldrPin);
  int calibratedLDR = map(ldrValue, 0, 4095, 0, 1000); 
  int rainState = digitalRead(rainPin);
  int flameState = digitalRead(flamePin);

  // Menggerakkan motor stepper berdasarkan nilai LDR
  if (calibratedLDR > 500) {  
    myStepper.step(stepsPerRevolution / 4); 
  } else {
    myStepper.step(-stepsPerRevolution / 4); 
  }

  // Mengirim data ke ThingsBoard
  StaticJsonDocument<200> doc;
  doc["ldrValue"] = calibratedLDR;
  doc["rainState"] = rainState == HIGH ? "Tidak Hujan" : "Hujan";
  doc["flameState"] = flameState == HIGH ? "Tidak Ada Api" : "Api Terdeteksi";

  // Menghitung jarak menggunakan sensor ultrasonik
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // Menghitung durasi pulsa
  if (duration == 0) {
    distance = -1; // Jika tidak ada respon
  } else {
    distance = duration * 0.034 / 2; // Rumus perhitungan jarak
  }

  if (distance > 0 && distance <= 400) {
    doc["distance"] = distance;  // Menambahkan jarak ke data JSON
  } else {
    doc["distance"] = "Out of range";  // Menandakan jika jarak di luar jangkauan
  }

  // Menghitung kecepatan aliran
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) {
    flowRate = flowCount / 7.5;  
    doc["flowRate"] = flowRate;
    flowCount = 0;
    lastTime = currentTime;
  }

  // Membaca sensor Soil Moisture
  int soilMoistureValue = analogRead(soilMoisturePin);
  int calibratedSoilMoisture = map(soilMoistureValue, 0, 4095, 0, 1000);
  doc["soilMoisture"] = calibratedSoilMoisture;

  // Serialize JSON to char array
  char telemetry[512];
  serializeJson(doc, telemetry);

  // Kirim data ke ThingsBoard
  client.publish("v1/devices/me/telemetry", telemetry);

  delay(500);  
}
