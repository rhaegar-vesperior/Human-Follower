#include <WiFi.h>
#include <WiFiUdp.h>

using namespace std; // Using directive as per your preference

// ================= WIFI =================
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const int udpPort = 1234;

WiFiUDP udp;
char packetBuffer[64];

// ================= MOTOR PINS =================
#define IN1 14
#define IN2 15
#define IN3 12
#define IN4 13
#define ENA 2
#define ENB 4

// ================= PWM (LEDC) =================
#define CH_R 0
#define CH_L 1
#define PWM_FREQ 5000
#define PWM_RES 8

struct VisionData {
    int centerX;
    long area;
};

QueueHandle_t commandQueue;
unsigned long lastPacketTime = 0;

// ================= PARAMETERS =================
const int TIMEOUT = 800;
const int FRAME_CENTER = 320;
const int DEAD_BAND = 25;
const int TARGET_AREA = 45000;
const int AREA_DEADZONE = 8000;
const int MAX_PWM = 200;
const int MIN_PWM = 40;   // Helps overcome motor friction

// ================= MOTOR CONTROL =================
void moveMotors(int leftSpeed, int rightSpeed) {

    // Apply minimum threshold to stop motor humming
    if (abs(leftSpeed) < MIN_PWM) leftSpeed = 0;
    if (abs(rightSpeed) < MIN_PWM) rightSpeed = 0;

    // Direction Control (Fixes "Yellow Line" boolean warnings)
    digitalWrite(IN1, (leftSpeed > 0) ? HIGH : LOW);
    digitalWrite(IN2, (leftSpeed < 0) ? HIGH : LOW);

    digitalWrite(IN3, (rightSpeed > 0) ? HIGH : LOW);
    digitalWrite(IN4, (rightSpeed < 0) ? HIGH : LOW);

    // PWM Control (Updated for ESP32 Core 3.x)
    ledcWrite(ENA, constrain(abs(leftSpeed), 0, MAX_PWM));
    ledcWrite(ENB, constrain(abs(rightSpeed), 0, MAX_PWM));
}

// ================= MOTOR TASK =================
void motorTask(void *pvParameters) {
    (void)pvParameters; // Removes unused parameter warning
    VisionData data;
    float smoothL = 0, smoothR = 0;

    for (;;) {
        // FAILSAFE: Stop if connection lost
        if (millis() - lastPacketTime > TIMEOUT) {
            moveMotors(0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (xQueueReceive(commandQueue, &data, pdMS_TO_TICKS(50))) {
            int speed = 0;
            int turn = 0;

            if (data.area > 2000) {
                // Calculation: Turning
                int errorX = data.centerX - FRAME_CENTER;
                if (abs(errorX) > DEAD_BAND)
                    turn = errorX / 2;

                // Calculation: Forward/Backward Speed
                long areaError = (long)TARGET_AREA - data.area;
                if (abs(areaError) > (long)AREA_DEADZONE)
                    speed = (int)(areaError / 300);
            }

            int lSpeed = speed + turn;
            int rSpeed = speed - turn;

            // Apply smoothing for organic movement
            smoothL = 0.8 * smoothL + 0.2 * lSpeed;
            smoothR = 0.8 * smoothR + 0.2 * rSpeed;

            moveMotors((int)smoothL, (int)smoothR);
        }
    }
}

// ================= WIFI TASK =================
void wifiTask(void *pvParameters) {
    (void)pvParameters;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        vTaskDelay(pdMS_TO_TICKS(500));

    udp.begin(udpPort);
    VisionData incoming;

    for (;;) {
        int size = udp.parsePacket();
        if (size) {
            int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
            if (len > 0) {
                packetBuffer[len] = 0;
                if (sscanf(packetBuffer, "%d,%ld", &incoming.centerX, &incoming.area) == 2) {
                    lastPacketTime = millis();
                    xQueueOverwrite(commandQueue, &incoming);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

    // New LEDC Syntax for ESP32 Core 3.x
    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);

    commandQueue = xQueueCreate(1, sizeof(VisionData));

    xTaskCreatePinnedToCore(wifiTask, "WiFi", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motorTask, "Motor", 4096, NULL, 2, NULL, 1);
}

void loop() { 
    vTaskDelete(NULL); 
}