# Robotraffic Careful Driving Winner 3rd Place
Autonomous car that follows traffic rules. <br />
In this time, we use WIFI as a main way of communication so ESP32 is our best choice. <br />
The car uses PID to track line and can be tuned via website: nodemcu-a4907.web.app <br />
ESPNOW is used as a way to send and get data between a traffic light and the car since it's quicker and it doesn't affect timers unlike database (we use Firebase library and it somehow affects ESP32 timers and messes up Servo and other PWM). <br />
![image](https://user-images.githubusercontent.com/80115619/157892738-300fd5e9-83e6-47d1-a55f-5c3de8896ba5.png)
