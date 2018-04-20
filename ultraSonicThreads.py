import RPi.GPIO as GPIO
import time


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

#sensor1
GPIO.setup(16,GPIO.OUT) #Trigger1
GPIO.setup(18,GPIO.IN)  #Echo1
#sensor2
GPIO.setup(19,GPIO.OUT) #Trigger2
GPIO.setup(21,GPIO.IN)	#Echo2
#sensor3
GPIO.setup(36,GPIO.OUT) #Trigger3
GPIO.setup(38,GPIO.IN)	#Echo3


triggerList = [16,19,36]
echoList    = [18,21,38]
startTime   = [0,0,0]
endTime     = [0,0,0]
distance    = [0,0,0]

#call back function for sensor1 thread event
def sensor0_callback(channel):
	global startTime
	global endTime
	global distane

	#checking whether the edge is rising or falling
	if GPIO.input(channel)==1:
		startTime[0]= time.time()
	elif GPIO.input(channel)==0:
		endTime[0]=time.time()
		pulseDuration = endTime[0]- startTime[0]
		distance[0] = ((pulseDuration * 34300)/2)

#call back function for sensor2 thread event
def sensor1_callback(channel):
	global startTime
	global endTime
	global distane

	#checking whether the edge is rising or falling
	if GPIO.input(channel)==1:
		startTime[1]= time.time()
	elif GPIO.input(channel)==0:
		endTime[1]=time.time()
		pulseDuration = endTime[1]- startTime[1]
		distance[1] = ((pulseDuration * 34300)/2)


#call back function for sensor3 thread event
def sensor2_callback(channel):
	global startTime
	global endTime
	global distane

	#checking whether the edge is rising or falling
	if GPIO.input(channel)==1:
		startTime[2]= time.time()
	elif GPIO.input(channel)==0:
		endTime[2]=time.time()
		pulseDuration = endTime[2]- startTime[2]
		distance[2] = (pulseDuration * 34300)/2

def distanceMeasurement(GPIO_TRIGGER,GPIO_ECHO,i):
    global startTime
    global endTime

    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER, False)

    return distance[i]


#creating edge detection (rising and falling) events on echos' pin
GPIO.add_event_detect(echoList[0],GPIO.BOTH,callback=sensor0_callback)
GPIO.add_event_detect(echoList[1],GPIO.BOTH,callback=sensor1_callback)
GPIO.add_event_detect(echoList[2],GPIO.BOTH,callback=sensor2_callback)

if __name__ == '__main__':
	try:
		while True:
		   for i in range(3):
			   recoveredDistance=distanceMeasurement(triggerList[i],echoList[i],i)
			   print("Sensor#",(i+1),recoveredDistance,"cm")
			   time.sleep(.4)
	except KeyboardInterrupt:
		print ("Measurement stopped by user")
		GPIO.cleanup()
	GPIO.cleanup()
