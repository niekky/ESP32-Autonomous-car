# ESP32 Autonomous car
Autonomous car that follows traffic rules using PID and ESP32.
*Robotraffic Careful Driving Winner 3rd Place 2022*

<p align="center">
  <img src="https://i.imgur.com/COV8BGi.jpg" width="700">
  <img src="https://i.imgur.com/wqRoDFk.jpg" width="700">
</p>

## Autonomous car:
### I/Microcontroller
ESP32 is a microcontroller manufactured by Epressif that has built-in WiFi, dual-core. In contrast to Arduino, ESP32 has way better performance and it's incredibly faster.

More info about ESP32: https://www.espressif.com/en/products/socs/esp32

<p align="center">
  <img src="https://i.imgur.com/FSb1s9u.jpg#center" width="500">
</p>
<p align="center">
  ESP32
</p>

<p align="center">
  <img src="https://i.imgur.com/J7BXW0T.jpeg" width="500">
</p>
<p align="center">
  ESP32 vs ESP8266 vs Arduino
</p>

### II/Motor driver
To control motor, we must have a motor driver that meets the motor requirement.

Our motor doesn't require much power so a basic typical motor driver like L298n is good enough.

<p align="center">
  <img src="https://i.imgur.com/naafulc.png" width="500">
</p>
<p align="center">
  L298n
</p>

### III/Line follower sensor
Typical line follower sensors in the market, such as TCRT5000 or CNY70, only outputs 0 or 1 instead of continuous value and that is a big disadvantage since PID (PID explanation below) needs a continuous value. Therefore, we used QTR-5RC sensors made by Polulu. QTR-5RC range is from 0 to 5000 (our case is 10000 since we use 2 sensors combined) which makes it perfect for PID.

<p align="center">
  <img src="https://i.imgur.com/Js82a7t.png" width="700">
</p>
<p align="center">
  Our line follower sensors
</p>

More info about QTR sensors: https://www.pololu.com/category/123/pololu-qtr-reflectance-sensors

### IV/Battery
We use 2s lipo battery to power our motor. Also, another lithium-ion battery is used to power ESP32.

<p align="center">
  <img src="https://i.imgur.com/4Qxe6Xr.png" width="700">
</p>
<p align="center">
  Our 2S liPo battery
</p>

<p align="center">
  <img src="https://i.imgur.com/8Gzl2AA.png" width="700">
</p>
<p align="center">
  Our Lithium-ion battery
</p>

We don't think using 2s lipo battery for everything is a good idea because that will make ESP32 reboot itself due to current drop caused by motor and servo.

### V/Steering mechanism
Ackerman steering mechanism is used as a way to make a car as realistic as possible. And another obvious reason is that the contest makes us do that.

<p align="center">
  <img src="https://i.imgur.com/GE1Bgo7.png" width="700">
</p>

### VI/PID
Proportional - Integral - Derivative is one of the most common control loop mechanisms. PID will output feedback from a setpoint and with that value, we will control servo in a way that our car moves automatically on line. In this case, we use line sensor values as inputs. Our setpoint is a line sensor's value when the car is moving straight on line (which is around 4000). Once calculated, the outputs will be fed to servo and control the direction of a car.

<p align="center">
  <img src="https://www.researchgate.net/profile/Sagar-Patel-47/publication/316709017/figure/fig1/AS:742091319681024@1553939766084/PID-Block-Diagram-PID-stands-for-Proportional-Integral-Derivative-control-A-PID.ppm" width="500">
</p>
<p align="center">
  PID controller
</p>

Tuning is required a lot to be stable. We only tune P and D since I is quite hard to implement and control. We had to tune it every time because PID configs will not work properly if motor speed changes. We haven't found the way to make motor speed static yet since it totally depends on battery.

<p align="center">
  <img src="https://i.imgur.com/boVIYXH.gif" width="800">
</p>
<p align="center">
  Bad tuning example
</p>

Tuning is dull and time-consuming. If we want to change a config, we have to upload code again, which takes around 2 minutes, and doing that a lot of time will take forever. Therefore, we create a website especially for tuning. With that, tuning is a lot faster and we don't need to upload code again.

<p align="center">
  <img src="https://i.imgur.com/vuwY05H.png" width="700">
</p>

### VII/Communicate with traffic lights
At first, We all decided that the car and traffic lights would share data with each other through database. Then, we chose Firebase as our database and used Firebase Realtime Database Arduino made by Mobizt. We thought it was a good idea until we discovered that it seems like this library affects ESP32 timer and that somehow causes the servo to weirdly behave. Another drawback is that it takes around 300ms to communicate with each other through Firebase, which is not quick enough for a car to stop at the right time.

To fix this, we used ESPNOW feature supported by ESP development board itself. ESPNOW is Espressif feature that let every ESP communicate with each other. Using this works wonderfully, the communication time is much less (around 5ms) and servo works incredibly well with PID.

More info about ESPNOW: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/


<p align="center">
  <img src="https://i.imgur.com/TVXlMON.jpg" width="700">
</p>

<p align="center">
  <img src="https://i.imgur.com/yeougQm.jpg" width="700">
</p>

*P/s: Lego Mindstorms EV3 in the picture is another section of the contest. It has nothing to do with a traffic light*

## Results: 
<p align="center">
  <img src="https://i.imgur.com/Vwj2qTY.gif" width="1000">
</p>

However, there are still many problems that haven't been fixed yet:
* Motor speed is not always the same, which means we have to tune PID config it every time to adapt to the line.
* The car has high possibility to move out of the line after a 15-minute run.
* We just only use simple PD control, we haven't used I yet. That is why the car is shaky sometimes.

*Thanks for reading*

*Leave a star to support us. If you have any questions, leave it below.*

## Credits:
Libraries used:
* Firebase RTDB Arduino Library for ESP32 by Mobizt
* QTRSensors by Polulu

Everything is made possible by some junior, senior students at Le Hong Phong High school in Ho Chi Minh, Vietnam
