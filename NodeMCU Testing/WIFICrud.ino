// Thư viện của Firebase và ESP8266WIFI
#include "FirebaseESP8266.h"
#include <ESP8266WiFi.h>

// Thư viện của I2C và MPU
#include "I2Cdev.h"
#include "MPU6050.h"

// Thư viện Wire
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Initialize accelgyro
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

// HOST lấy từ Project Settings/Service Accounts/Firebase Admin SDK/databaseURL
#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
// Auth lấy từ Project Settings/Service Accounts/Database Secrets/Secret
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

// WIFI_SSID: Tên WIFI
#define WIFI_SSID "SCTV-CAM07"
// WIFI_PASSWORD: Tên pass của WIFI
#define WIFI_PASSWORD "1234567899"

FirebaseData db;
FirebaseJson json;

unsigned long previousTime=0;
unsigned long previousTime2=0;
unsigned long previousTime3=0;
unsigned long previousTime4=0;
int led1=-1;
byte LED2=0;

void setup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    randomSeed(5);

    // pinMode LED1 - Điều khiển qua Firebase
    pinMode(D7, OUTPUT);
    // pinMode LED2 - Blink theo chu kỳ
    pinMode(D5,OUTPUT);

    // Kết nối với WIFI
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");

    // Kiểm tra kết nối WIFI
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    // Kết nối với Firebase
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void readMPU(){
    // Lấy các giá trị từ MPU6050
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Tạo Json
    json.set("Ax",ax);
    json.set("Ay",ay);
    json.set("Az",az);

    // Thực hiện theo chu kỳ 25ms
    if (millis()-previousTime4>=25){
        // Gửi JsonData đến Firebase
        Firebase.setJSON(db,"/node_1/Axyz",json);
        previousTime4=millis();
    }
}

void loop(){
    // Thực hiện theo chu kỳ 100ms
    if (millis()-previousTime>=100){
        // Lấy integer value từ Firebase
        if (Firebase.getInt(db,"/node_1/LED1")){
            if (db.dataTypeEnum()== fb_esp_rtdb_data_type_integer){
                led1=db.to<int>();
                // Serial.println(led1);
                // if (led1==0){
                //     digitalWrite(D7,LOW);
                // } else {
                //     digitalWrite(D7,HIGH);
                // }
            }
        } else {
            Serial.println(db.errorReason());
        }
        previousTime=millis();  
    }

    if (led1==0){
        digitalWrite(D7,LOW);
    }
    if (led1==1){
        digitalWrite(D7,HIGH);
    }
    
    // Thực hiện theo chu kỳ 2s
    if (millis()-previousTime2>=2000){
        if (LED2==0){
            digitalWrite(D5,HIGH);
            LED2=1;
        } else {
            digitalWrite(D5,LOW);
            LED2=0;
        }
        previousTime2=millis();
    }
    
    // What if delay()?
    // digitalWrite(D5,HIGH);
    // delay(2000);
    // digitalWrite(D5,LOW);
    // delay(2000);

    // Thực hiện theo chu kỳ 5s
    // if (millis()-previousTime3>=5000){
    //     Firebase.setInt(db,"/node_1/randNum",random(300));
    //     previousTime3=millis();
    // }

    readMPU();

}