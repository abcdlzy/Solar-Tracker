# Solar Tracker

solar tracker system
太阳能发电板自动追踪太阳

using hardware：MPU9250 , Raspberry pi 3 , Geared motor

using lib：pysolar,Django,wiringPi,i2c-sensor-hal



接线图(Wiring) MPU9250 to Raspberry Pi
红色RED VCC GPIO pin 01(3.3v DC Power)
黑色BLACK GND GPIO pin 06(GND)
紫色Purple SDA GPIO pin 05(SCL1,I2C)
蓝色BLUE SCL GPIO pin 03(SDA1,I2C)

因为MPU9250的读数在没有校准过的情况下非常不稳定，采用了在旁边放置一块磁铁来强行稳定读数的方法。
Because MPU9250 readings in very unstable without calibrated, used the force to steady readings in placing a magnet next to approach.

主要工作逻辑：
1、计算日出日落
2、判断偏离角度然后校准
3、等待固定秒数后继续执行第二步

针对晚上的情况，采取了移动到中间角度并且检测，以防止被大风吹偏碰到限位开关而无法工作。

Main logic:
1, calculating Sunrise and sunset
2, determine the deviation angle and calibration
3, continues to wait for a fixed number of seconds the second step

For evening, moved to an intermediate point and testing to prevent being blown touch the limit switch then system shutdown.