// WIFI LIBRARY
#include <WiFi.h>
#include <QTRSensors.h>
#include <esp_now.h>
#include "esp_task_wdt.h"

// #include "QuickPID.h"
#define FIREBASE_USE_PSRAM

QTRSensors qtr;
float kp=0.02, ki=0, kd=0.016;
float Setpoint=4000, Input, Output;
float kp_motor=0.5, ki_motor=0, kd_motor=0.2;
int pid_output=0;
int servo_wip=75;
int motor_speed=20000;

int sum_err=0;
int error=0;
int count_err=0;
int previouserror=0;
int hall;
byte magnetic=0;
bool hallEn;
uint16_t position=0;
byte traffic_state;

typedef struct struct_message
{
  byte traffic_state;
} struct_message;

struct_message myData;

float previous_error = 0, previous_I = 0;
float previous_error_motor = 0, previous_I_motor = 0;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

boolean motor_toggle = true;

long previousTime=0;

// QuickPID myPID(&Input,&Output,&Setpoint,kp,ki,kd,DIRECT);

// SERVO CONFIG
#define SERVO_CHANNEL_0 0
#define SERVO_TIMER_16_BIT 16
#define SERVO_BASE_FREQ 50
#define SERVO_PIN 21
// MOTOR CONFIG
#define MOTOR_CHANNEL_0 1
#define MOTOR_TIMER_13_BIT 16
#define MOTOR_BASE_FREQ 8000
#define MOTOR_PIN_ENB 13
#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 14
#define HALL_PIN 39

void SensorCalibrate()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 5, 18, 19, 32, 33, 25,  26, 27}, 10);
  qtr.setEmitterPin(4);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

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

void SetServoPos(float pos)
{
  uint32_t duty = (((pos / 180.0) * 2000) / 20000.0 * 65536.0) + 1634;
  // convert 0-180 degrees to 0-65536

  ledcWrite(SERVO_CHANNEL_0, duty);
  // set channel to pos
}

void ServoTesting()
{
  for (int x = 0; x < 100; x++)
  {
    SetServoPos(x);
    delay(10);
  }
  delay(100);
}

void windup(){
  sum_err+=error;
  count_err+=1;
  if (count_err>5) count_err=0;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(2,1);
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Traffic State: ");
  // Serial.println(myData.traffic_state);
  traffic_state=myData.traffic_state;
  // while (myData.traffic_state==1 && hallEn==true){
  //   digitalWrite(MOTOR_PIN_1, 0);
  //   digitalWrite(MOTOR_PIN_2, 0);
  //   ledcWrite(MOTOR_CHANNEL_0, 0);
  // }
  
  
}

void setup()
{
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(HALL_PIN,INPUT);
  ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
  ledcAttachPin(MOTOR_PIN_ENB, MOTOR_CHANNEL_0);

  ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_16_BIT);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);

  // // PID LIBRARY
  // myPID.SetMode(myPID.Control::automatic);
  // myPID.SetTunings(kp,ki,kd);
  // myPID.SetOutputLimits(-180,180);
  // myPID.SetSampleTimeUs(50000);

  // Set servo default
  SetServoPos(90);

  Serial.begin(115200);

  //ESPNOW
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Calibrate sensor for a while
  SensorCalibrate();

  // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void readHallPlus(){
  hall=digitalRead(HALL_PIN);
  if (hall==0){
    magnetic++;
    hallEn=true;
    previousTime=millis();
    while (hall==0){
      hall=digitalRead(HALL_PIN);
      hallEn=true;
      previousTime=millis();
    }
  }
}

void loop(){
  long start = millis();
  digitalWrite(2,0);

  // Drive motor
  digitalWrite(MOTOR_PIN_1, 0);
  digitalWrite(MOTOR_PIN_2, 1);
  ledcWrite(MOTOR_CHANNEL_0, motor_speed);

  if (millis()-previousTime>=500){
    hallEn=false;
    previousTime=millis();
  }

  // Read Hall sensor
  hall=digitalRead(HALL_PIN);
  if (hall==0){
    hallEn=true;
  }
  readHallPlus();

  while (traffic_state==1 && magnetic%2==1 && hallEn==true){
    digitalWrite(MOTOR_PIN_1, 0);
    digitalWrite(MOTOR_PIN_2, 0);
    ledcWrite(MOTOR_CHANNEL_0,0); 
    previousTime=millis();
  }
  
  while (traffic_state==0 && magnetic%2==0 && hallEn==true){
    digitalWrite(MOTOR_PIN_1, 0);
    digitalWrite(MOTOR_PIN_2, 0);
    ledcWrite(MOTOR_CHANNEL_0,0);
    previousTime=millis();
  }

  // while (traffic_state==1 && hallEn==true){
  //   digitalWrite(MOTOR_PIN_1, 0);
  //   digitalWrite(MOTOR_PIN_2, 0);
  //   ledcWrite(MOTOR_CHANNEL_0,0);
  //   if (traffic_state==0){
  //     hallEn=false;
  //     break;
  //   }
  // }

  // RAW PID FUNCTION
  position = qtr.readLineBlack(sensorValues);
  error=4000-position;
  pid_output = kp*error + ki*sum_err + kd*(error - previouserror);
  windup();
  previouserror = error;

  // PID LIBRARY
  // Input = (float) position;
  // myPID.SetTunings(kp,ki,kd);
  // myPID.Compute();
  // pid_output=(int) Output;

  SetServoPos(max(30, min(130, servo_wip - pid_output)));
  Serial.println("Mag: "+String(magnetic));
  Serial.println("            Hall: "+String(hallEn));
  Serial.println("Traffic_State: "+String(traffic_state));
  // ServoTesting();

  // Chỉ uncomment nếu muốn đọc số liệu, nếu ko thì nên disable vì nó tốn thời gian excecute
  // Serial.println("P: "+String(kp)+" D: "+String(kd*10)+" I: "+String(ki));
  // Serial.println("P_motor: "+String(kp_motor)+" D_motor: "+String(kd_motor)+" I_motor: "+String(ki_motor));
  // Serial.println("Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle)+" Position: "+String(position));
  // Serial.println("Input: " + String(Input) + " Output: " + String(pid_output));
  // Serial.println("                    Hall read: "+String(hall));
  Serial.println("TASK1 Speed: " + String(millis() - start));
}
