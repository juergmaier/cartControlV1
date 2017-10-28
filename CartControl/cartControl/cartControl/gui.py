import arduino
import Tkinter as tk
import time
from thread import start_new_thread
import numpy as np
import cartControl
import collections

CANV_WIDTH = 300
CANV_HEIGHT = 500
''' from fahren.cpp in MotorizedBase
todo: pass these values from cartControl to the Arduino on startup
#define BODEN_NAH_MIN  9
#define BODEN_NAH_MAX  20
#define BODEN_FERN_MIN  17
#define BODEN_FERN_MAX  28
'''
MIN_RANGE_SHORT = 9
MAX_RANGE_SHORT = 20
MIN_RANGE_LONG = 17
MAX_RANGE_LONG = 28
SHORT_ARC_BOX = (MAX_RANGE_SHORT + MIN_RANGE_SHORT) / 2

LINE_SCALE_SHORT = 1
LINE_SCALE_FAR = 1

# use named tupels to make the base positions available
# e.g. reference by: servos[0].id
location = collections.namedtuple('Location', 'id x y x1 y1 x2 y2 arcFrom arcLength')
sensors = []
# front left
X=CANV_WIDTH/4
Y=CANV_HEIGHT/4
sensors.append(location(id=0, x=X, y=Y, x1=X-SHORT_ARC_BOX, y1=Y-SHORT_ARC_BOX, x2=X+SHORT_ARC_BOX,y2=Y+SHORT_ARC_BOX,arcFrom=0,arcLength=180))
Y=CANV_HEIGHT/4 - 40
sensors.append(location(id=0, x=X, y=Y, x1=X-MAX_RANGE_LONG, y1=Y-MAX_RANGE_LONG, x2=X+MAX_RANGE_LONG,y2=Y+MAX_RANGE_LONG,arcFrom=0,arcLength=180))

# front right
X=CANV_WIDTH/4*3
Y=CANV_HEIGHT/4
sensors.append(location(id=1, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=0,arcLength=180))
Y=CANV_HEIGHT/4 - 40
sensors.append(location(id=0, x=X, y=Y, x1=X-MAX_RANGE_LONG, y1=Y-MAX_RANGE_LONG, x2=X+MAX_RANGE_LONG,y2=Y+MAX_RANGE_LONG,arcFrom=0,arcLength=180))

# back left
X=CANV_WIDTH/4
Y=CANV_HEIGHT/4*3
sensors.append(location(id=2, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=-180,arcLength=180))
Y=CANV_HEIGHT/4*3 + 40
sensors.append(location(id=0, x=X, y=Y, x1=X-MAX_RANGE_LONG, y1=Y-MAX_RANGE_LONG, x2=X+MAX_RANGE_LONG,y2=Y+MAX_RANGE_LONG,arcFrom=-180,arcLength=180))

# back right
X=CANV_WIDTH/4*3
Y=CANV_HEIGHT/4*3
sensors.append(location(id=3, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=-180,arcLength=180))
Y=CANV_HEIGHT/4*3 + 40
sensors.append(location(id=0, x=X, y=Y, x1=X-MAX_RANGE_LONG, y1=Y-MAX_RANGE_LONG, x2=X+MAX_RANGE_LONG,y2=Y+MAX_RANGE_LONG,arcFrom=-180,arcLength=180))

# left
X=CANV_WIDTH/4
Y=CANV_HEIGHT/2
sensors.append(location(id=4, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=90,arcLength=180))
sensors.append(location(id=4, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=90,arcLength=180))

# right
X=CANV_WIDTH/4*3
Y=CANV_HEIGHT/2
sensors.append(location(id=5, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=-90,arcLength=180))
sensors.append(location(id=5, x=X, y=Y, x1=X-MAX_RANGE_SHORT, y1=Y-MAX_RANGE_SHORT, x2=X+MAX_RANGE_SHORT,y2=Y+MAX_RANGE_SHORT,arcFrom=-90,arcLength=180))

# global variables
gui = None
controller = None

class manualControl:
    def __init__(self, mainWindow):

        self.w = mainWindow

        self.canvas = tk.Canvas(mainWindow, width=CANV_WIDTH, height=CANV_HEIGHT, background='white')
        self.canvas.grid(row=0, column=1 )
        self.canvas.create_text(sensors[0].x,sensors[0].y + 25,text="Front Left")
        self.canvas.create_text(sensors[2].x,sensors[2].y + 25,text="Front Right")
        self.canvas.create_text(sensors[4].x,sensors[4].y - 25,text="Back Left")
        self.canvas.create_text(sensors[6].x,sensors[6].y - 25,text="Back Right")
        self.canvas.create_text(sensors[8].x+40 ,sensors[8].y,text="Left")
        self.canvas.create_text(sensors[10].x-40,sensors[10].y,text="Right")


        frame = tk.Frame(self.w)
        frame.grid(row=0, column=0, sticky='n')

        self.btnArduino = tk.Button(frame, text="Arduino", command=self.startArduino)
        self.btnArduino.grid(row=0, columnspan=2)
               
        self.lblInfo = tk.Label(frame, text="wait for Arduino button pressed", fg ="white", bg="red")
        self.lblInfo.grid(row=1, columnspan=2)

        self.lblDistanceObstacle = tk.Label(frame, text="distance to obstacle: ")
        self.lblDistanceObstacle.grid(row = 20, column = 0, sticky=tk.E)

        self.lblDistanceObstacleValue = tk.Label(frame, text=" ")
        self.lblDistanceObstacleValue.grid(row = 20, column = 1)

        self.lblDistanceAbyss = tk.Label(frame, text="max distance value: ")
        self.lblDistanceAbyss.grid(row = 25, column = 0, sticky=tk.E)

        self.lblDistanceAbyssValue = tk.Label(frame, text=" ")
        self.lblDistanceAbyssValue.grid(row = 25, column = 1)
    
        self.lblCommand = tk.Label(frame, text="command: ")
        self.lblCommand.grid(row = 30, column = 0, sticky=tk.E)

        self.lblCommandValue = tk.Label(frame, text=" ")
        self.lblCommandValue.grid(row = 30, column = 1)
                
        self.lblRotationTarget = tk.Label(frame, text="target rotation: ")
        self.lblRotationTarget.grid(row = 35, column = 0, sticky=tk.E)

        self.lblRotationTargetValue = tk.Label(frame, text=" ")
        self.lblRotationTargetValue.grid(row = 35, column = 1)

        self.lblRotationCurrent = tk.Label(frame, text="current rotation: ")
        self.lblRotationCurrent.grid(row = 40, column = 0, sticky=tk.E)

        self.lblRotationCurrentValue = tk.Label(frame, text=" ")
        self.lblRotationCurrentValue.grid(row = 40, column = 1)
        
        self.sbRotation = tk.Spinbox(frame, from_=-90, to=90, width=3)
        self.sbRotation.grid(row = 50, column = 0, padx=50, pady=20, sticky=tk.E)

        self.btnRotate = tk.Button(frame, text="Rotate", state="disabled", command=self.rotateCart)
        self.btnRotate.grid(row = 50, column = 1)
        
        self.choices = ["stop","forward","for_diag_right","for_diag_left","left","right","backward","back_diag_right","back_diag_left"]
        defaultDirection = "forward"
        self.direction = self.choices.index(defaultDirection)

        move = tk.StringVar(gui)
        move.set(defaultDirection)
        self.ddMove = tk.OptionMenu(frame, move, *self.choices, command=self.selectedDirection)
        self.ddMove.grid(row = 60, column = 0, sticky=tk.E)

        self.btnMove = tk.Button(frame, text="Move", state="disabled", command = self.moveCart)
        self.btnMove.grid(row = 60, column = 1) 
        
        self.lblXValue = tk.Label(frame, text="X: ")
        self.lblXValue.grid(row = 80, column = 0, pady=10, sticky=tk.E)

        self.sbX = tk.Spinbox(frame, from_=0, to=400, text = "300", width=3)
        self.sbX.grid(row = 80, column = 1, sticky=tk.W)
        self.sbX.insert(0, 30)

        self.lblYValue = tk.Label(frame, text="Y: ")
        self.lblYValue.grid(row = 82, column = 0, sticky=tk.E)

        self.sbY = tk.Spinbox(frame, from_=0, to=400, width=3)
        self.sbY.grid(row = 82, column = 1, sticky=tk.W)
        self.sbY.insert(0, 34)

        self.btnNav = tk.Button(frame, text="navigate", state="normal", command = self.navigateTo)
        self.btnNav.grid(row = 84, column = 1)

        self.servoChoices = ["VL","VR","LM","RM","HL","HR"]
        defaultServo = "VL"
        self.servoID = self.servoChoices.index(defaultServo)

        testServo = tk.StringVar(gui)
        testServo.set(defaultServo)
        self.ddServo = tk.OptionMenu(frame, testServo, *self.servoChoices, command=self.selectedServo)
        self.ddServo.grid(row = 90, column = 0, sticky=tk.E)

        self.btnServoTest = tk.Button(frame, text="Scan Servo", state="normal", command = self.scanServo)
        self.btnServoTest.grid(row = 90, column = 1, columnspan=1, pady=10)

        # sensor test
        self.sensorChoices = ["VL nah","VL fern","VR nah", "VR fern","LM nah","RM nah","HL nah","HL fern","HR nah", "HL fern"]
        defaultSensor = "VL nah"
        self.sensorID = self.sensorChoices.index(defaultSensor)

        testSensor = tk.StringVar(gui)
        testSensor.set(defaultSensor)
        self.ddSensor = tk.OptionMenu(frame, testSensor, *self.sensorChoices, command=self.selectedSensor)
        self.ddSensor.grid(row = 95, column = 0, sticky=tk.E)

        self.btnSensorTest = tk.Button(frame, text="Read Distance", state="normal", command = self.readSensor)
        self.btnSensorTest.grid(row = 95, column = 1, columnspan=1)

        self.lblSensorDistance = tk.Label(frame, text="-", width=25)
        self.lblSensorDistance.grid(row = 96, column = 1)

        # heart beat blinker                
        self.lblHeartBeat = tk.Label(frame, text="Heart Beat", fg="red")
        self.lblHeartBeat.grid(row = 100, column = 0, columnspan = 2, pady=10)

        self.btnStop = tk.Button(frame, text="STOP CART", state="normal", command = self.stopCart, bg = "red", fg = "white")
        self.btnStop.grid(row = 200, column = 0, columnspan=2, pady=30)

        self.startArduino()
        

    def showDistances(self):
        for i in range(cartControl.NUM_DISTANCE_SENSORS):
            s = cartControl.distanceSensors[i]
            d = cartControl.distanceList[i]
            sensor = sensors[i]

            # show valid range of short distance measures
            self.canvas.create_arc(
                sensor.x1, sensor.y1, sensor.x2,sensor.y2,
                start=sensor.arcFrom, extent=sensor.arcLength, style=tk.ARC, width=(MAX_RANGE_SHORT - MIN_RANGE_SHORT) * LINE_SCALE_SHORT, outline="green")

            for k in range(cartControl.NUM_MEASUREMENTS_PER_SCAN):
                xOffset = np.cos(np.radians(k * 15 + 12 + s.get('rotation'))) * d[k]
                yOffset = np.sin(np.radians(k * 15 + 12 + s.get('rotation'))) * d[k]
                age = time.time()- s.get('timestamp')
                col = s.get('color')
                if age > 2 and col=='blue': col='light blue' 
                if age > 2 and col=='red': col='coral1' 
                self.canvas.create_line(sensor.x, sensor.y, sensor.x + xOffset, sensor.y + yOffset, fill=col)

    def selectedDirection(self, value):
        self.direction = self.choices.index(value)
        #println("selected direction: " + value + " index: " + self.direction)

    def selectedServo(self, value):
        self.servoID = self.servoChoices.index(value)

    def selectedSensor(self, value):
        self.sensorID = self.sensorChoices.index(value)

    def startArduino(self):

        arduino.initSerial("COM5")
        self.lblInfo.configure(text = "arduino connected, waiting for cart ready message", bg="white smoke", fg="orange")
        self.btnArduino.configure(state = "disabled")

        # Messages sent by the Arduino should always get processed
        # Start reader in its own thread
        #start_new_thread(arduino.readMessages,())

        self.w.update_idletasks()
        self.w.after(2000, self.checkStatus)


    def checkStatus(self):
    
        if cartControl.arduinoStatus == 1:
            self.lblInfo.configure(text = "cart ready", bg="lawn green", fg="black")
            self.btnRotate.configure(state="normal")
            self.btnMove.configure(state="normal")
            self.btnNav.configure(state="normal")
            self.w.update_idletasks()
            self.w.after(10, self.heartBeat)
        else:
            self.w.after(400, self.checkStatus)
      

    def navigateTo(self):
        start = time.time()
        PathFinder.analyze((int(self.sbX.get()),int(self.sbY.get())))
        print "analyzed in: ", time.time()-start, " seconds."
        self.w.update_idletasks()
    
    def heartBeat(self):

        # toggle heartBeat message color
        if self.lblHeartBeat.cget("fg") == "red":
            self.lblHeartBeat.configure(fg = "green")
        else: 
            self.lblHeartBeat.configure(fg = "red")

        arduino.sendHeartbeat()
        arduino.getCartOrientation()
        self.lblRotationCurrentValue.configure(text=str(cartControl.orientation))
        self.w.update_idletasks()
        self.w.after(500, self.heartBeat)   # heart beat loop
        
#        self.showDistances()


    def stopCart(self):

        arduino.sendStopCommand()
        self.lblCommand.configure(text="Stop")
        self.w.update_idletasks()

    
    def moveCart(self, speed = 50):

        arduino.sendMoveCommand(self.direction, speed)
        self.lblCommandValue.configure(text="Move")
        #self.lblMove.configure(text=str(speed))
        self.w.update_idletasks()

    def scanServo(self):

        arduino.sendScanServoCommand(self.servoID)
        self.lblCommandValue.configure(text="scanServo")
        self.w.update_idletasks()


    def readSensor(self):

        arduino.sendReadSensorCommand(self.sensorID)
        self.lblCommandValue.configure(text="readSensor")
        self.w.update_idletasks()


    def rotateCart(self):

        angle = int(self.sbRotation.get())

        self.lblCommandValue.configure(text="Rotate")
        self.lblRotationTargetValue.configure(text=str(cartControl.orientation + angle))
        arduino.sendRotateCommand(angle)
        self.w.update_idletasks()

    def updateTargetRotation(self, target):
        self.lblRotationTargetValue.configure(text=str(target))
        self.w.update_idletasks()

    def updateDistanceSensorObstacle(self, distance):
        self.lblDistanceObstacleValue.configure(text=str(distance))
        self.w.update_idletasks()

    def updateDistanceSensorAbyss(self, distance):
        self.lblDistanceAbyssValue.configure(text=str(distance))
        self.w.update_idletasks()

    def updateSensorDistance(self, sensorDistance):
        self.lblSensorDistance.configure(text=sensorDistance)
        self.w.update_idletasks()


def startGui():

    global gui
    global controller

    start=time.time()
    gui = tk.Tk()
    gui.geometry('600x600+100+150')     # window size and position

    controller = manualControl(gui)
    #print "gui initialized in: ", time.time()-start, " seconds"
    gui.mainloop()

    pass

