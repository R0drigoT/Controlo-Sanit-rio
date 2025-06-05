#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.emqx.io";

const char* topicSubscribeAuto = "IPB/IoT/Lab/AirQuality";           // Tópico automático (temp, IAQ)
const char* topicSubscribeManual = "IPB/IoT/Lab/Actuators/ManualState"; // Tópico manual
const char* topicPublish = "IPB/IoT/Lab/Actuators/State";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 10000;  // 10 segundos

// Estados manuais
int estadoManualJanela = 0;
int estadoManualCortina = 0;
int estadoManualAC = 0;

// Estados automáticos para AC
int estadoAutomaticoAC = 0;  // 0 = desligado, 1 = aquecer, 2 = arrefecer, 3 = ventilar

bool manualAtivo = false;

int currentEstadoJanela = 1;
int currentEstadoCortina = 1;
int currentEstadoAC = 0;

const int buzzerPin = 4;
const int ledPin = 2;
const int ldrPin = 34;
const int servoPinJanela = 18;
const int servoPinCortina = 5;

int lotacao = 0;
int brilho = 0;

Servo servoJanela;
Servo servoCortina;

int estadoStrToNum(const char* estadoStr, const char* tipo) {
  if (strcmp(estadoStr, "") == 0 || strcmp(estadoStr, "automático") == 0) return 0;

  if (strcmp(tipo, "janela") == 0) {
    if (strcmp(estadoStr, "fechada") == 0) return 1;
    if (strcmp(estadoStr, "semi-aberta") == 0) return 2;
    if (strcmp(estadoStr, "aberta") == 0) return 3;
  }
  if (strcmp(tipo, "cortina") == 0) {
    if (strcmp(estadoStr, "baixa") == 0) return 1;
    if (strcmp(estadoStr, "média") == 0) return 2;
    if (strcmp(estadoStr, "alta") == 0) return 3;
  }
  if (strcmp(tipo, "ac") == 0) {
    if (strcmp(estadoStr, "desligado") == 0) return 0;
    if (strcmp(estadoStr, "aquecer") == 0) return 1;
    if (strcmp(estadoStr, "arrefecer") == 0) return 2;
    if (strcmp(estadoStr, "ventilar") == 0) return 3;
  }
  return 0;
}

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

void updateServoPositionJanela(int estado) {
  int angle = 0;
  switch (estado) {
    case 1: angle = 0; break;     // fechada
    case 2: angle = 90; break;    // semi-aberta
    case 3: angle = 180; break;   // aberta
    default: angle = 0; break;
  }
  servoJanela.write(angle);
  Serial.print("Servo (Janela) position set to angle: ");
  Serial.println(angle);
}

void updateServoPositionCortina(int estado) {
  int angle = 0;
  switch (estado) {
    case 1: angle = 0; break;     // baixa
    case 2: angle = 90; break;    // média
    case 3: angle = 180; break;   // alta
    default: angle = 0; break;
  }
  servoCortina.write(angle);
  Serial.print("Servo (Cortina) position set to angle: ");
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

  if (String(topic_received) == topicSubscribeManual) {
    // Lê estado manual da janela, cortina e AC
    const char* estadoManualJanelaStr = doc["janela"] | "";
    const char* estadoManualCortinaStr = doc["cortina"] | "";
    const char* estadoManualACStr = doc["ac"] | "";

    estadoManualJanela = estadoStrToNum(estadoManualJanelaStr, "janela");
    estadoManualCortina = estadoStrToNum(estadoManualCortinaStr, "cortina");
    estadoManualAC = estadoStrToNum(estadoManualACStr, "ac");

    manualAtivo = doc["manualActive"] | false;

    if (estadoManualJanela != currentEstadoJanela) {
      currentEstadoJanela = estadoManualJanela;
      updateServoPositionJanela(currentEstadoJanela);
    }
    if (estadoManualCortina != currentEstadoCortina) {
      currentEstadoCortina = estadoManualCortina;
      updateServoPositionCortina(currentEstadoCortina);
    }
    if (manualAtivo && estadoManualAC != currentEstadoAC) {
      currentEstadoAC = estadoManualAC;
      Serial.print("Manual AC state: ");
      Serial.println(currentEstadoAC);
      // Aqui podes ligar um atuador real para AC se tiver
    }
  } else if (String(topic_received) == topicSubscribeAuto) {
    // Leitura dos valores automáticos para ajustar janela, cortina e AC
    float temp = doc["temp"] | -100.0;
    float iaq = doc["iaq"] | -1.0;

    int ldrValue = analogRead(ldrPin);  // Leitura LDR

    Serial.print("Automatic mode - temp: ");
    Serial.print(temp);
    Serial.print(" °C, IAQ: ");
    Serial.println(iaq);
    Serial.print("LDR Value: ");
    Serial.println(ldrValue);

    if (!manualAtivo) {
      // Janela: fechada < 18°C, semi-aberta 18-25°C, aberta > 25°C
      if (temp < 18) {
        currentEstadoJanela = 1;
      } else if (temp < 25) {
        currentEstadoJanela = 2;
      } else {
        currentEstadoJanela = 3;
      }
      updateServoPositionJanela(currentEstadoJanela);

      // Cortina: LDR baixo <300 baixa, 300-700 média, >700 alta
      if (ldrValue < 300) {
        currentEstadoCortina = 1;
      } else if (ldrValue < 700) {
        currentEstadoCortina = 2;
      } else {
        currentEstadoCortina = 3;
      }
      updateServoPositionCortina(currentEstadoCortina);

      // AC automático baseado em temperatura e IAQ
      if (temp < 15) {
        estadoAutomaticoAC = 1;  // aquecer
      } else if (temp >= 26) {
        estadoAutomaticoAC = 2;  // arrefecer
      } else if (iaq > 200) {
        estadoAutomaticoAC = 3;  // ventilar
      } else {
        estadoAutomaticoAC = 0;  // desligado
      }

      if (estadoAutomaticoAC != currentEstadoAC) {
        currentEstadoAC = estadoAutomaticoAC;
        Serial.print("Automatic AC state: ");
        Serial.println(currentEstadoAC);
      }
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topicSubscribeManual);
      client.subscribe(topicSubscribeAuto);
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

  servoJanela.attach(servoPinJanela);
  servoCortina.attach(servoPinCortina);

  updateServoPositionJanela(currentEstadoJanela);
  updateServoPositionCortina(currentEstadoCortina);

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

    StaticJsonDocument<256> pubDoc;
    pubDoc["estadoJanela"] = currentEstadoJanela;
    pubDoc["estadoCortina"] = currentEstadoCortina;
    pubDoc["estadoAC"] = currentEstadoAC;
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
