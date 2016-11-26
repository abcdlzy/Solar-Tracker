import subprocess
from pysolar.solar import *
from threading import Timer, Thread, Event
import sched, time
import datetime
import RPi.GPIO as GPIO
import os
#import Adafruit_CharLCD as LCD


d = datetime.datetime.now()
GPIO.setmode(GPIO.BOARD)
#37 is machine power enable
#40 is machine turn way False Left , True Right
GPIO.setup(40,GPIO.OUT)
GPIO.setup(37,GPIO.OUT)
GPIO.output(40,False)
GPIO.output(37,False)


solarAzimuth=20
solarAltitude=10
DeviationRange=10

longitude=1
latitude=1


now=datetime.datetime.now()

lastTurnTime=datetime.datetime.now()


dawn=datetime.datetime.now()
sunset=datetime.datetime.now()
spanMinutesCount=0

def getMaxLeftAzimuth():
    return float(open('/home/pi/SolarTrackerWatching/maxLeft').read())

def getMaxRightAzimuth():
    return float(open('/home/pi/SolarTrackerWatching/maxRight').read())

def getCenterAzimuth():
    return (getMaxRightAzimuth()+getMaxLeftAzimuth())/2

def isMaxLeft():
    if getNowAzimuth()<=getMaxLeftAzimuth():
        return True
    else:
        return False

def isMaxRight():
    if getNowAzimuth()>=getMaxRightAzimuth():
        return True
    else:
        return False

def turnLeft():
    if isMaxLeft():
        turnOff()
        print('isMaxLeft')
        return True
    else:
        GPIO.output(37,True)
        GPIO.output(40,True)
        lastTurnTime=datetime.datetime.now()
        return True

def turnRight():
    if isMaxRight():
        turnOff() 
        print('isMaxRight')
        return True
    else:
        GPIO.output(37,True)
        GPIO.output(40,False)
        lastTurnTime=datetime.datetime.now()
        return True

def turnOff():
    GPIO.output(40,False)
    GPIO.output(37,False)
    return True    

def isCooldown():
    if(lastTurnTime+datetime.timedelta(minutes=1)<datetime.datetime.now()):
        return True
    else:
        return False


def getSolarAzimuth(t=now):
    azimuth=0-get_azimuth(latitude,longitude,t)
    if(azimuth>180):
        azimuth-=360
    return azimuth

def getNowAzimuth():
    var = os.popen('./azimuth').read()
    return float(var)

def getFixNowAzimuth():
    if getMaxLeftAzimuth()>getMaxRigthAzimuth():
        return getNowAzimuth()+360
    else:
        return getNowAzimuth()

def getSolarAltitude(t=now):
    return get_altitude(latitude,longitude,t)

def calc():
    global lastTurnTime
    if(isCooldown()):
        if getFixNowAzimuth()<getSolarAzimuth():
            lastTurnTime=datetime.datetime.now()
            turnRight()
        elif getFixNowAzimuth()>getSolarAzimuth():
            lastTurnTime=datetime.datetime.now()
            turnLeft()


def calc_NextDawnAndSunset():
    doCale_DawnAndSunset(datetime.datetime.now()+datetime.timedelta(days=1))

def calc_DawnAndSunset():
    doCale_DawnAndSunset(datetime.datetime.now())
 
def doCale_DawnAndSunset(getTime):
    global spanMinutesCount
    global dawn
    global sunset
    global now
    today = getTime
    midnight = datetime.datetime(today.year, today.month, today.day, 0, 0, 0)-datetime.timedelta(hours=8)
    nextDayMidNight=datetime.datetime(today.year, today.month, today.day+1, 0, 0, 0)-datetime.timedelta(hours=8)
    isFoundDawn=False
    isFoundSunset=False
    print('midNIght')
    print(midnight+datetime.timedelta(hours=8)) 
    print('nextMidNIght')
    print(nextDayMidNight+datetime.timedelta(hours=8))
    while midnight<nextDayMidNight:
        midnight=midnight+datetime.timedelta(minutes=1)
        if not isFoundDawn:
            if getSolarAltitude(midnight)>0:
            # pysolar check get and set
                dawn=midnight
                isFoundDawn=True

        elif not isFoundSunset:
            if getSolarAltitude(midnight)<0:
            # pysolar check get and set
                sunset=midnight
                isFoundSunset=True
        else:
            break

    spanMinutesCount=(sunset-dawn).seconds

def run():
    calc_DawnAndSunset()
    print("dawn")
    print(dawn)
    print("sunset")
    print(sunset)
    print("span")
    print(spanMinutesCount)
    global now
    global lastTurnTime


    isRunnedToday=True
    today=datetime.datetime.now()

    while True:
        now=datetime.datetime.now()
        print('\nRun_now')
        print(datetime.datetime.now()+datetime.timedelta(hours=8))
        if now>sunset:
            calc_NextDawnAndSunset()
            isRunnedToday=False
            print("calc_dawn")
            print(dawn)
            print("calc_sunset")
            print(sunset)
        #time.sleep(600)
        #break
        elif now>dawn and now<sunset:
            print("RUN_dawn")
            print(dawn)
            print("RUN-sunset")
            print(sunset)
            #isRunnedToday=True
            if(getNowAzimuth()>(getCenterAzimuth()+getSolarAzimuth(now)+5)):
                print("debug-run-Left")
                turnLeft()
                time.sleep(0.1)
            elif(getNowAzimuth()<(getCenterAzimuth()+getSolarAzimuth(now))):
                print("debug-run-Right")
                turnRight()
                time.sleep(0.1)
            elif(getNowAzimuth()>(getCenterAzimuth()+getSolarAzimuth(now))):
                print("debug-run-stop")
                turnOff()
                time.sleep(30)
        #print(getSolarAzimuth(now))
        #print(getNowAzimuth())
        #print(getNowAzimuth()-(getCenterAzimuth()+getSolarAzimuth(now)))
        
        #print("z:"+str(getSolarAzimuth(now)))
        #print("l:"+str(getSolarAltitude(now)))
        #time.sleep(0.5)
        else:
            print("wait-dawn")
            print(dawn)
            print("wait-sunset")
            print(sunset)
            if(getNowAzimuth()>(getCenterAzimuth()+5)):
                print("debug-wait-Left")
                turnLeft()
                time.sleep(0.1)
            elif(getNowAzimuth()<(getCenterAzimuth())):
                print("debug-wait-Right")
                turnRight()
                time.sleep(0.1)
            elif(getNowAzimuth()>(getCenterAzimuth())):
                print("debug-wait-stop")
                turnOff()
                time.sleep(30)



run()

