#include <WiFi.h>
#include <WebSocketsServer.h>

// ================= WIFI =================
const char* ssid = "Pixel 7a"; // add your device(phone/laptop) wifi ID here
const char* password = "1029384756"; // add your device(phone/laptop) wifi Password here

WebSocketsServer webSocket(81);

// ================= MOTOR PINS =================
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 32

// ================= ULTRASONIC =================
#define TRIG 19
#define ECHO 18

float distance = 100;

// ================= SPEED =================
int leftSpeed = 0;
int rightSpeed = 0;

// ================= FAILSAFE =================
unsigned long lastReceiveTime = 0;
#define CONNECTION_TIMEOUT 1500

// ================= DISTANCE =================
void readDistance(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 20000);

  if(duration > 0){
    distance = duration * 0.034 / 2;
  }
}

// ================= MOTOR =================
void applyMove(){

  // obstacle safety
  if(distance < 50){
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // connection safety
  if(millis() - lastReceiveTime > CONNECTION_TIMEOUT){
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // LEFT MOTOR direction
  if(leftSpeed > 0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  } else {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
  }

  // RIGHT MOTOR direction
  if(rightSpeed > 0){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  } else {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
  }

  // PWM (v3 API)
  ledcWrite(ENA, leftSpeed);
  ledcWrite(ENB, rightSpeed);
}

// ================= WEBSOCKET =================
void onWS(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_TEXT){

    String data = String((char*)payload);

    int comma = data.indexOf(',');
    if(comma > 0){
      leftSpeed = data.substring(0, comma).toInt();
      rightSpeed = data.substring(comma+1).toInt();
      lastReceiveTime = millis();
    }
  }
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // ✅ NEW PWM API (ESP32 v3.x)
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(onWS);
}

// ================= LOOP =================
void loop(){
  webSocket.loop();
  readDistance();
  applyMove();
}
