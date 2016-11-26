# Solar Tracker

solar tracker system<br>  
太阳能发电板自动追踪太阳<br>  <br>  

### using
using hardware：MPU9250 , Raspberry pi 3 , Geared motor <br>  

using lib：pysolar,Django,wiringPi,i2c-sensor-hal <br>  
<br>  

###wiring
接线图(Wiring) MPU9250 to Raspberry Pi<br>  
红色RED VCC GPIO pin 01(3.3v DC Power)<br>  
黑色BLACK GND GPIO pin 06(GND)<br>  
紫色Purple SDA GPIO pin 05(SCL1,I2C)<br>  
蓝色BLUE SCL GPIO pin 03(SDA1,I2C)<br>  
<br>  
###something
因为MPU9250的读数在没有校准过的情况下非常不稳定，采用了在旁边放置一块磁铁来强行稳定读数的方法。<br>  
Because MPU9250 readings in very unstable without calibrated, used the force to steady readings in placing a magnet next to approach.<br>  
<br>  
主要工作逻辑：<br>  
1、计算日出日落<br>  
2、判断偏离角度然后校准<br>  
3、等待固定秒数后继续执行第二步<br>  

针对晚上的情况，采取了移动到中间角度并且检测，以防止被大风吹偏碰到限位开关而无法工作。<br>  
<br>  <br>  
Main logic:<br>  
1, calculating Sunrise and sunset<br>  
2, determine the deviation angle and calibration<br>  
3, continues to wait for a fixed number of seconds the second step<br>  

For evening, moved to an intermediate point and testing to prevent being blown touch the limit switch then system shutdown.<br>  

![](https://github.com/abcdlzy/nothing/blob/master/solarTracker1.png)  
![](https://github.com/abcdlzy/nothing/blob/master/solarTracker2.png)  
