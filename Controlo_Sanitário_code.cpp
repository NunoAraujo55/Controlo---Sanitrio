#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// —— Configurações do DHT22 ——
#define DHTPIN   27
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);



// —— Configurações do servo da janela——
const int   servoPin     = 18;
Servo        windowServo;

//descomentar em caso de usar a temp para abrir as janelas.
/*
const float  TEMP_CLOSED  = 25.0;  // ≤25°C → janela fechada (0°)
const float  TEMP_SEMI    = 28.0;  // ≤28°C → meio-aberta (90°)
// >28°C → totalmente aberta (180°)
*/
int windowAngle  = 0;    
int curtainAngle = 0;    

const int  CO2_CLOSED = 1000;   // ≤1000 ppm → window closed (0°)
const int  CO2_SEMI   = 2000;   // ≤2000 ppm → half-open (90°)

//Servo Para cortinas
const int   curtainServoPin = 21;
Servo       curtainServo;
const int CURTAIN_LOW    =   0;
const int CURTAIN_MEDIUM =  90;
const int CURTAIN_HIGH   = 180;

// —— Sensores ultrassónicos ——
#define TRIG_A      32   // sensor A (lado interior da porta)
#define ECHO_A      33
#define TRIG_B      14   // sensor B (lado exterior da porta)
#define ECHO_B      12
const int     DIST_THRESHOLD = 50;      // cm: corte para “passagem”
bool          flagA = false, flagB = false;
unsigned long timeA = 0, timeB = 0;
const unsigned long FLAG_TIMEOUT = 5000; // ms para cancelar flag, valor alto para testes
int           peopleCount = 0;

// LED Config
const int pinLED = 17;

// LDR Config
const int pinLdrA = 34;
const int pinLdrD = 26;

// —— Credenciais Wi-Fi e MQTT ——
const char* ssid            = "Wokwi-GUEST";
const char* password        = "";
const char* mqtt_server     = "broker.emqx.io";
const uint16_t mqtt_port    = 1883;
const char* topic_readings  = "home/esp32";

WiFiClient   espClient;
PubSubClient client(espClient);

// Função para ler distância (HC-SR04 style)
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30 ms
  return duration > 0 ? duration / 58 : 999;      // cm
}

void connectWiFi() {
  Serial.print("WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  //Serial.println("\nWi-Fi conectado, IP: " + WiFi.localIP().toString());
}

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  String msg = String((char*)payload, len);
  float co2 = msg.toFloat();

  // choose an angle based on CO₂
  int angle = (co2 <= CO2_CLOSED ?   0
               : co2 <= CO2_SEMI   ?  90
                                   : 180);
  windowServo.write(angle);
  windowAngle = angle; 

  Serial.printf("CO₂: %.0f ppm → Servo %d°\n", co2, angle);
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT…");
    if (client.connect("ESP32_AllSensors")) {
      Serial.println(" conectado");
      client.subscribe("sensor/co2");
      Serial.println("Subscribed to sensor/co2");
    } else {
      Serial.print(" falhou rc="); Serial.print(client.state());
      Serial.println(" → tentando em 5s");
      delay(5000);
    }
  }
}


void setup() {
  Serial.begin(9600);
  dht.begin();

  // servo para janela
  windowServo.attach(servoPin, 1000, 2000);

  //servo para cortina
  curtainServo.attach(curtainServoPin, 1000, 2000);
  // ultrassónicos
  pinMode(TRIG_A, OUTPUT);
  pinMode(ECHO_A, INPUT);
  pinMode(TRIG_B, OUTPUT);
  pinMode(ECHO_B, INPUT);
  pinMode(pinLdrD, INPUT);
  pinMode(pinLED, OUTPUT);

  // rede
  connectWiFi();
  client.setCallback(onMqttMessage);
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();
}

void loop() {
  // mantém MQTT
  if (!client.connected()) connectMQTT();
  client.loop();
  
  // 1) Leitura DHT e posicionamento do servo
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  //abrir e fechar janela baseado na luminosidade. 
  /*if (isnan(h) || isnan(t)) {
    Serial.println("Erro DHT");
  } else {
    int angle = (t <= TEMP_CLOSED   ?  0 :
                (t <= TEMP_SEMI     ? 90 : 180));
    windowServo.write(angle);
    //Serial.printf("Temp: %.1f °C  Hum: %.1f%%  -> Servo %d°\n", t, h, angle);
  }*/

  // 2) Leitura dos dois ultrassónicos
  long distA = readDistance(TRIG_A, ECHO_A);
  long distB = readDistance(TRIG_B, ECHO_B);
  //Serial.printf("A: %ld cm  B: %ld cm\n", distA, distB);

  // 3) Lógica de flags para detectar passagem (entry/exit vem primeiro)
if (distB < DIST_THRESHOLD && flagA) {
  // B lê enquanto A já estava ativo → ENTRADA
  peopleCount++;
  Serial.printf("+++ Entrada! Count = %d\n", peopleCount);
  flagA = false;
}
else if (distA < DIST_THRESHOLD && flagB) {
  // A lê enquanto B já estava ativo → SAÍDA
  peopleCount--;
  Serial.printf("--- Saída!   Count = %d\n", peopleCount);
  flagB = false;
}
else if (distA < DIST_THRESHOLD && !flagA && !flagB) {
  // A lê sozinho → set flagA
  flagA = true;
  timeA = millis();
  Serial.println("→ A detectado (início possível entrada)");
}
else if (distB < DIST_THRESHOLD && !flagB && !flagA) {
  // B lê sozinho → set flagB
  flagB = true;
  timeB = millis();
  Serial.println("→ B detectado (início possível saída)");
}

// cancela flags passados FLAG_TIMEOUT  
if (flagA && millis() - timeA > FLAG_TIMEOUT) {
  flagA = false;
  Serial.println("~~ Timeout A ~~");
}
if (flagB && millis() - timeB > FLAG_TIMEOUT) {
  flagB = false;
  Serial.println("~~ Timeout B ~~");
}

   int luxValue  = analogRead(pinLdrA);
  int threshHit = digitalRead(pinLdrD);
  //Serial.printf("Light: %4d   Above threshold? %d\n", luxValue, threshHit);

  
  //—— Decidir posição da cortina conforme LDR ——
  int curtainAngle;
  if (luxValue > 2500) {
    // ambiente muito claro → feche totalmente
    digitalWrite(pinLED, HIGH); 
    curtainAngle = CURTAIN_LOW;
    
  }
  else if (luxValue > 1200) {
    // claro moderado → meia-abertura 
    curtainAngle = CURTAIN_MEDIUM;
    digitalWrite(pinLED, HIGH); 
  }
  else {
    // escuro → abre cortina 
    curtainAngle = CURTAIN_HIGH;
    digitalWrite(pinLED, LOW);
  }

  curtainServo.write(curtainAngle);
  //Serial.printf("  → Cortina: %3d°\n", curtainAngle);

  // 4) Publicar via MQTT
  if (!isnan(h) && !isnan(t)) {
    char payload[150];
    snprintf(payload, sizeof(payload),
      "{\"temperature\":%.1f,"
      "\"humidity\":%.1f,"
      "\"occupancy\":%d,"
      "\"windowAngle\":%d,"
      "\"curtainAngle\":%d}", 
      t, h, peopleCount, windowAngle, curtainAngle
    ); 
     client.publish(topic_readings, payload);
    Serial.printf(
      "Publish MQTT → T:%.1f°C  H:%.1f%%  Occ:%d  Janela:%d°  Cortina:%d°\n",
      t, h, peopleCount, windowAngle, curtainAngle
    );
  }else {
    Serial.println("Erro leitura DHT22, não publicando.");
  }



  delay(2000);
}
