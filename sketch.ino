#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.emqx.io";

const char* topicSubscribeAuto = "IPB/IoT/Lab/AirQuality";           // Tópico automático
const char* topicSubscribeManual = "IPB/IoT/Lab/Actuators/ManualState"; // Tópico manual
const char* topicPublish = "IPB/IoT/Lab/Actuators/State";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 10000;  // 10 segundos

// Estados automáticos e manuais para janela, cortina e AC
int estadoAutomaticoJanela = 1;
int estadoManualJanela = 0;

int estadoAutomaticoCortina = 1;
int estadoManualCortina = 0;

int estadoAutomaticoAC = 0;
int estadoManualAC = 0;

bool manualAtivo = false;

int currentEstadoJanela = 1;

const int buzzerPin = 4;
const int ledPin = 2;
const int ldrPin = 34;
const int servoPin = 27;

int lotacao = 0;
int brilho = 0;

Servo servo;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void updateServoPosition(int estadoJanela) {
  int angle = 0;
  switch (estadoJanela) {
    case 1: angle = 0; break;     // fechada
    case 2: angle = 90; break;    // semi-aberta
    case 3: angle = 180; break;   // aberta
    default: angle = 0; break;
  }
  servo.write(angle);
  Serial.print("Servo (Janela) position set to angle: ");
  Serial.println(angle);
}

void callback(char* topic_received, byte* payload, unsigned int length) {
  String messageTemp;
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  Serial.print("Message received on topic ");
  Serial.print(topic_received);
  Serial.print(": ");
  Serial.println(messageTemp);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, messageTemp);
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.f_str());
    return;
  }

  if (String(topic_received) == topicSubscribeAuto) {
    
    int iaq = doc["iaq"] | 0;
    int co2 = doc["co2_eqv"] | 0;

    int estadoAutoJanelaLocal = 1;
    if (co2 > 2200 || iaq > 400) {
      estadoAutoJanelaLocal = 3;
    } else if (co2 > 1700 || iaq > 200) {
      estadoAutoJanelaLocal = 2;
    } else {
      estadoAutoJanelaLocal = 1;
    }
    estadoAutomaticoJanela = estadoAutoJanelaLocal;


  } else if (String(topic_received) == topicSubscribeManual) {

    estadoManualJanela = doc["janela"] | 0;
    estadoManualCortina = doc["cortina"] | 0;
    estadoManualAC = doc["ac"] | 0;

    manualAtivo = (estadoManualJanela > 0 || estadoManualCortina > 0 || estadoManualAC > 0);
  }

  int estadoJanelaParaUsar = manualAtivo && estadoManualJanela > 0 ? estadoManualJanela : estadoAutomaticoJanela;
  if (estadoJanelaParaUsar != currentEstadoJanela) {
    currentEstadoJanela = estadoJanelaParaUsar;
    updateServoPosition(currentEstadoJanela);
  }

  

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topicSubscribeAuto);
      client.subscribe(topicSubscribeManual);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(buzzerPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(ledPin, OUTPUT);

  servo.attach(servoPin);
  updateServoPosition(currentEstadoJanela);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublishTime > publishInterval) {
    brilho = random(0, 256);
    analogWrite(ledPin, brilho);
    Serial.print("Novo brilho LED: ");
    Serial.println(brilho);

    lotacao = random(0, 6);

    if (lotacao == 5) {
      tone(buzzerPin, 1000);
    } else {
      noTone(buzzerPin);
    }

    int ldrValue = analogRead(ldrPin);
    Serial.print("Leitura LDR: ");
    Serial.println(ldrValue);

    // Publica estados para Node-RED (podes expandir se quiseres)
    StaticJsonDocument<256> pubDoc;
    pubDoc["estadoJanela"] = currentEstadoJanela;
    pubDoc["lotacao"] = lotacao;
    pubDoc["ldr"] = ldrValue;

    char buffer[256];
    serializeJson(pubDoc, buffer);

    Serial.print("Publishing: ");
    Serial.println(buffer);

    client.publish(topicPublish, buffer);
    lastPublishTime = now;
  }
}
