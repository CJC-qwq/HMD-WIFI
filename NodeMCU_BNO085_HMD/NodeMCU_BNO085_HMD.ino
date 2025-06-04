#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// WiFi settings
const char* ssid = "MIX";
const char* password = "cjc20020817qwq";
const int port = 5555;

// LED settings
const int STATUS_LED = LED_BUILTIN;  // NodeMCU的内置LED
const int FAST_BLINK_INTERVAL = 100;  // 快速闪烁间隔(ms)
const int SLOW_BLINK_INTERVAL = 1000; // 慢速闪烁间隔(ms)

WiFiUDP udp;
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

// 校准相关变量
float referenceQuat[4] = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
float inverseReferenceQuat[4] = {1.0, 0.0, 0.0, 0.0}; // w, -x, -y, -z
bool calibrated = false;

// LED控制函数
void blinkLED(int interval) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink >= interval) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        lastBlink = millis();
    }
}

// 四元数求逆（对于单位四元数，逆就是共轭）
void quaternionInverse(float* result, float* q) {
  result[0] = q[0];   // w
  result[1] = -q[1];  // -x
  result[2] = -q[2];  // -y
  result[3] = -q[3];  // -z
}

// 四元数乘法函数
void quaternionMultiply(float* result, float* q1, float* q2) {
  result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; // w
  result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; // x
  result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; // y
  result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; // z
}

void scanI2CDevices() {
    Serial.println("\nScanning I2C devices...");
    byte error, address;
    int nDevices = 0;
    
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found!");
    }
    Serial.println("Scan completed.\n");
}

void setup() {
    // 初始化LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);  // NodeMCU的LED是低电平点亮
    
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\nBNO085 VR Headset Starting...");
    
    // 初始化I2C
    Wire.begin(D2, D1);  // SDA = D2 (GPIO4), SCL = D1 (GPIO5)
    Wire.setClock(400000);  // 设置为400kHz (Fast Mode)
    
    // 连接WiFi
    Serial.println("\nConnecting to WiFi...");
    Serial.printf("SSID: %s\n", ssid);
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        blinkLED(FAST_BLINK_INTERVAL);  // 快速闪烁表示正在连接
        Serial.print(".");
        delay(100);
    }
    
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Signal Strength (RSSI): %d dBm\n", WiFi.RSSI());
    
    // 初始化BNO085
    Serial.println("\nInitializing BNO085...");
    bool sensorInitialized = false;
    
    for (int attempt = 0; attempt < 5; attempt++) {
        Serial.printf("Attempt %d: Trying to connect to BNO085...\n", attempt + 1);
        if (bno.begin_I2C(0x4B)) {
            sensorInitialized = true;
            Serial.println("BNO085 found!");
            break;
        }
        delay(100);
    }
    
    if (!sensorInitialized) {
        Serial.println("Failed to initialize BNO085!");
        while (1) {
            blinkLED(FAST_BLINK_INTERVAL);  // 快速闪烁表示错误
        }
    }
    
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Failed to enable rotation vector!");
        while (1) {
            blinkLED(FAST_BLINK_INTERVAL);  // 快速闪烁表示错误
        }
    }
    
    // 执行初始校准
    Serial.println("\nPerforming initial calibration...");
    Serial.println("Please keep the sensor in neutral position for 3 seconds.");
    
    // 给用户时间调整到中立姿态
    for(int i=3; i>0; i--) {
        Serial.printf("Calibrating in %d seconds...\n", i);
        delay(1000);
    }
    
    // 读取当前姿态作为参考
    bool calibrationSuccess = false;
    for(int attempt = 0; attempt < 10; attempt++) {
        if (bno.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            referenceQuat[0] = sensorValue.un.gameRotationVector.real;
            referenceQuat[1] = sensorValue.un.gameRotationVector.i;
            referenceQuat[2] = sensorValue.un.gameRotationVector.j;
            referenceQuat[3] = sensorValue.un.gameRotationVector.k;
            
            // 计算参考四元数的逆
            quaternionInverse(inverseReferenceQuat, referenceQuat);
            
            calibrationSuccess = true;
            calibrated = true;
            break;
        }
        delay(100);
    }
    
    if (calibrationSuccess) {
        Serial.println("Calibration complete!");
        Serial.printf("Reference quaternion: %.4f, %.4f, %.4f, %.4f\n", 
                    referenceQuat[0], referenceQuat[1], referenceQuat[2], referenceQuat[3]);
    } else {
        Serial.println("Calibration failed! Using default orientation.");
    }
    
    Serial.println("\nSystem ready!");
    Serial.println("UDP Broadcast: 255.255.255.255:5555");
    Serial.println("Quaternion format: W,X,Y,Z (calibrated)");
    Serial.println("\nStarting data stream...\n");
}

void loop() {
    // 检查WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost! Reconnecting...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            blinkLED(FAST_BLINK_INTERVAL);
            delay(10);
        }
        Serial.println("WiFi reconnected!");
        Serial.print("New IP: ");
        Serial.println(WiFi.localIP());
    }

    // 正常工作时慢速闪烁
    blinkLED(SLOW_BLINK_INTERVAL);

    if (bno.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float currentQuat[4] = {
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            };
            
            float calibratedQuat[4] = {1.0, 0.0, 0.0, 0.0};
            
            if (calibrated) {
                // 应用校准：calibratedQuat = inverseReferenceQuat * currentQuat
                quaternionMultiply(calibratedQuat, inverseReferenceQuat, currentQuat);
            } else {
                // 未校准，直接使用原始数据
                memcpy(calibratedQuat, currentQuat, sizeof(float) * 4);
            }
            
            // 交换x和y分量
            float temp = calibratedQuat[1];
            calibratedQuat[1] = calibratedQuat[2];
            calibratedQuat[2] = temp;
            
            // 将y分量（原x分量）取相反数
            calibratedQuat[2] = -calibratedQuat[2];
            
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "%.6f,%.6f,%.6f,%.6f",
                    calibratedQuat[0], calibratedQuat[1], calibratedQuat[2], calibratedQuat[3]);
            
            IPAddress broadcastIP(255, 255, 255, 255);
            udp.beginPacket(broadcastIP, port);
            udp.write(buffer);
            udp.endPacket();
            
            // 每200ms打印一次数据
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 200) {
                Serial.print("Calibrated Quaternion: ");
                Serial.println(buffer);
                lastPrint = millis();
            }
        }
    }
    
    delay(10);
} 