#include <analogWrite.h>
#include <ESP32PWM.h>
// #include <ESP32Servo.h>
#include <ESP32Tone.h>

// Thư viện đèn
#include <QTRSensors.h>
// Thư viện servo
// #include <Servo.h>
// Thư viện của Firebase và ESP8266WIFI
#include "FirebaseESP32.h"
#include <WiFi.h>

//Cách QTR hoạt động
//SETUP:
//10s cho việc calibrate sensor
//loop sẽ chạy từ 0-5000
String id_car="car_2";
QTRSensors qtr;
QTRSensors qtr2;
float kp=0;
float ki=0;
float kd=0;
int pid_output=0;
int servo_wip=90;
int motor_speed=0;
int error=0;
int previouserror=0;
boolean motor_toggle=false;

uint16_t position=0;
uint16_t position2=0;
float previous_error = 0, previous_I = 0;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t sensorValues2[SensorCount];


unsigned long previousTime=0;
unsigned long previousTime2=0;

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

/////////////////////////////////////////////////////////////////////////////////////////

class newpidConfig{
  public:
    void setConfig(float kpp,float kip,float kdp){
      kp=kpp;
      ki=kip;
      kd=kdp;
    }

    float getKP(){
      return kp;
    }

    float getKD(){
      return kd;
    }

    float getKI(){
      return ki;
    }

    float getError(){
      return error;
    }

    float PIDloop(int error){
      float output=kp*error + kd*(error - previous_error);
      previous_error = error;
      return output;
    }

  private:
    float kp;
    float ki;
    float kd;
    int previous_error=0;
};

float pidConfig(float p,float i,float d){
  kp=p;
  ki=i;
  kd=d;
}

// void ServoDefault(){
//   steering.write(servo_wip);
// }

void SensorCalibrate(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14,15,16,17,18,11,10,9,8,4}, 10);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void SensorCalibrate1(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14,15,16,17,18}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void SensorCalibrate2(){
  // configure the sensors
  qtr2.setTypeRC();
  qtr2.setSensorPins((const uint8_t[]){11,10,9,8,4}, SensorCount);
  qtr2.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr2.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr2.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr2.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void valueChangeDetect(String value,String previous_value){
  if (value!=previous_value){
    Serial.println("LOG: do stuff");
  }
}

void readFromDB(){
  if (millis()-previousTime>=100){
    if (Firebase.getString(db,"IDs/"+id_car+"/P")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_kp=db.to<String>();
        kp=dp_kp.toFloat();
      }
    } else {
      Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/D")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_kd=db.to<String>();
        kd=dp_kd.toFloat();
      }
    } else {
      Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/I")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_ki=db.to<String>();
        ki=dp_ki.toFloat();
      }
    } else {
      Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/Motor")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String db_motor_speed=db.to<String>();
        motor_speed=db_motor_speed.toInt();
      }
    } else {
      Serial.println(db.errorReason());
    }
    
    if (Firebase.getString(db,"/IDs/"+id_car+"/Servo")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String db_servo_wip=db.to<String>();
        servo_wip=db_servo_wip.toInt();
      }
    } else {
      Serial.println(db.errorReason());
    }

    if (Firebase.getBool(db,"/IDs/"+id_car+"/Toggle")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_boolean){
        motor_toggle=db.to<bool>();
      }
    } else {
      Serial.println(db.errorReason());
    }

    previousTime=millis();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////

// Init PID class
newpidConfig newPIDConfig;

// PWM Setup:
double PWM_frequency=1000;
uint8_t PWM_resolution=10;
uint8_t PWM_channel0=13;

// Servo PWM
double Servo_frequency=50;
uint8_t Servo_resolution=8;
uint8_t Servo_channel0=0;

void setup()
{
  ledcSetup(2,50,PWM_resolution);
  ledcAttachPin(25,2);
  
  pinMode(16,OUTPUT);
  pinMode(17,OUTPUT);
  
  // ledcAttachPin(13,PWM_channel0);
  // ledcSetup(PWM_channel0,PWM_frequency,PWM_resolution);
// Servo
  ledcSetup(Servo_channel0,Servo_frequency,Servo_resolution);
  ledcAttachPin(25,Servo_channel0);

  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  // Kiểm tra kết nối WIFI
  while (WiFi.status() != WL_CONNECTED){
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
  // SensorCalibrate1();
  // SensorCalibrate2();
  // pidConfig(0.1,0,0); //0.02 0 0.01
}
void loop()
{
  // Đọc inputs từ Firebase
  readFromDB();

  // Motor
  // Note: 
  // Toggle GPIO: 16; GPIO:17
  // Speed: GPIO: 12
  if (motor_toggle==true){
    digitalWrite(16,1);
    digitalWrite(17,0);
    // ledcWrite(PWM_channel0,motor_speed);
  } else{
    digitalWrite(16,0);
    digitalWrite(17,0);
    // ledcWrite(PWM_channel0,0);

  }
    
  // position = qtr.readLineBlack(sensorValues);
  // position2= 4000-qtr2.readLineBlack(sensorValues2);

  // Print values from db
  Serial.println("P: "+String(kp)+" D: "+String(kd)+" I: "+String(ki)+" Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle));
  
  newPIDConfig.setConfig(kp,ki,kd);

  error=position2-position;
  pid_output = kp*error + kd*(error - previouserror);
  previouserror = error;

  // Send PID outputs to DB
  if (millis()-previousTime2>=50){
    Firebase.setInt(db,"IDs/"+id_car+"/PID_outputs",120);
    previousTime2=millis();
  }

  // if (angleturn>40) pid_output=40;
  // if (angleturn<-30) pid_output=-30;
  // steering.write(90-pid_output);
  // steering.write(servo_wip)
  ;
  // steering.write(90);
  ledcWrite(2,servo_wip);

}