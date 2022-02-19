#include "FirebaseESP32.h"
#include <WiFi.h>

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "Ku min"
#define WIFI_PASSWORD "01658186379"

int r=12;
int y=14;
int g=27;

FirebaseData fbdo;

void setup()
{
  Serial.begin(115200);
  pinMode(r,OUTPUT);
  pinMode(y,OUTPUT);
  pinMode(g,OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}

void count(int max_count){
  for(int i=1;i<=max_count;i++){
    delay(1000);
    Serial.println(i);
    Firebase.RTDB.setInt(&fbdo, "traffic/count", i);
  }
}

void red(){
  Firebase.RTDB.setString(&fbdo, "traffic/light", "red");
  digitalWrite(r,HIGH);
  count(10);
  digitalWrite(r,LOW);
}

void yellow(){
  Firebase.RTDB.setString(&fbdo, "traffic/light", "yellow");
  digitalWrite(y,HIGH);
  count(3);
  digitalWrite(y,LOW);
}

void green(){
  Firebase.RTDB.setString(&fbdo, "traffic/light", "green");
  digitalWrite(g,HIGH);
  count(7);
  digitalWrite(g,LOW);
}

void loop()
{  
  green();
  yellow();
  red();
}


