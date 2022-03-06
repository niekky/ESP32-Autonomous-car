#include <QTRSensors.h>

QTRSensors qtr, qtr1 , qtr2;
float set_point=5000, kp=0.2, ki=0, kd=0.1 , kp_curve = 0.02 , kd_curve = 0.007 , ki_curve = 0, sum_error = 0;
float kp_motor=0, ki_motor=0, kd_motor=0;
int pid_output=0;
int servo_wip=90 , mi , ma;
int motor_speed=12000, motor_speed_run=0 , prevspeed = 0 , magnetic = 0 ,hall,count = 0 , speed = 13000;
int error=0;
int giaidoan=1;
int a[5] , na = 0 ;
int previouserror=0;
int pid_p=0, pid_i=0, pid_d=0;
uint16_t position=0 , position2 = 0;
unsigned int prev = millis(),previoustime=0;

float previous_error = 0, previous_I = 0;
float previous_error_motor=0, previous_I_motor=0;
const uint8_t SensorCount =10;
uint16_t sensorValues[SensorCount] ,sensorValues2[SensorCount];

boolean motor_toggle=false;

// SERVO CONFIG
#define SERVO_CHANNEL_0     0
#define SERVO_TIMER_16_BIT  16
#define SERVO_BASE_FREQ     50
#define SERVO_PIN           21
// MOTOR CONFIG
#define MOTOR_CHANNEL_0     1
#define MOTOR_TIMER_13_BIT  16
#define MOTOR_BASE_FREQ     8000
#define MOTOR_PIN_ENB       13
#define MOTOR_PIN_1         12
#define MOTOR_PIN_2         14

void SensorCalibrate(){
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16,17,5,18,19,32,33,25,26,27}, 10);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); 
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
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
    uint32_t duty = (((pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
    ledcWrite(SERVO_CHANNEL_0,duty);
}

void ServoTesting(){
    for(int x=0;x<100;x++)
   {
      SetServoPos(x);
      delay(10);
   }
   delay(100);
   
}

void setup(){
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(39,INPUT);
    ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
    ledcAttachPin(MOTOR_PIN_ENB, MOTOR_CHANNEL_0);
    
    ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_16_BIT);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);

    Serial.begin(115200);

    SensorCalibrate();
}

void loop(){
  int  hall=digitalRead(39);
  if (hall==0)  motor_toggle=(!motor_toggle) ? true:false;
    
  if (motor_toggle){
    //giaidoan1
    if (giaidoan==1){
      set_point=5000; kp=0.2; ki=0; kd=0.1; motor_speed=20000;
      mi=45; ma=85;
    }
    //giaidoan2
    if (millis()-previoustime>=1000 && giaidoan==1){
      giaidoan=2;
      set_point=5000; kp=0.3; ki=0; kd=0.1; motor_speed=12000;
      mi=40; ma=110;
      previoustime=millis();
    }
    //giaidoan3
    if (millis()-previoustime>=1000 && giaidoan==2){
      giaidoan=3;
      set_point=5000; kp=0.2; ki=0; kd=0.1; motor_speed=20000;
      mi=45; ma=85;
      previoustime=millis();
    }
    //giaidoan4
    if (millis()-previoustime>=1000 && giaidoan==3){
      giaidoan=4;
      set_point=5000; kp=0.2; ki=0; kd=0.1; motor_speed=20000;
      mi=40; ma=110;
      previoustime=millis();
    }
    //giaidoan5
    if (millis()-previoustime>=1000 && giaidoan==4){
      giaidoan=5;
      set_point=5000; kp=0.2; ki=0; kd=0.1; motor_speed=50000;
      mi=45; ma=85;
      previoustime=millis();
    }
  }
  else{
    previoustime=millis();
  }
    

  digitalWrite(MOTOR_PIN_1,1);
  digitalWrite(MOTOR_PIN_2,0);
  ledcWrite(MOTOR_CHANNEL_0,motor_speed);

  position = qtr.readLineBlack(sensorValues);
  error = set_point - position;
  pid_output = kp*error + kd*(error-previouserror);
    
  SetServoPos(max(mi,min(ma,servo_wip+pid_output)));
  previouserror = error;
}