#Robot violet

from sr.robot import *
from math import *
R = Robot()
v = 70




class Nav():
    targetBearing=0
    strongestSignal=0
    targetName=''
    rightM=0.0
    leftM=0.0
    x = 0
    y = 0
    couleur = ""

    def forgetPreviousTarget(self):
        self.targetBearing=0
        self.strongestSignal=0
        self.targetName=''

    def claimTerritory(self):
        print('Claiming a territory')
        R.radio.claim_territory()
        self.forgetPreviousTarget()

    def setMotors(self, r, l):
        self.rightM=r
        self.leftM=l
        R.motors[0].m1.power = r
        R.motors[0].m0.power = l

    def stop(self):
        self.setMotors(0,0)

    def goBackward(self):
        print("going backward")
        while R.ruggeduinos[0].analogue_read(0) < 0.20:
            print(R.ruggeduinos[0].analogue_read(0))
            self.setMotors(-50, -50)
            R.sleep(0.1)
    def rotate(self):
        self.setMotors(-100, 100)
        R.sleep(0.80)
        self.stop()

    def avoidPillar(self):
        print('avoiding a pillar')
        if self.couleur == "violet" :
            if self.x == 1 and self.y == 0 :
                R.sleep(10)
                self.y = self.y + 1
            if self.x > 5:
                self.x = 1
            if self.x == 0 or self.x == 1 or self.x == 5:
                self.setMotors(-50, -50)
                R.sleep(0.30)
                self.setMotors(50, -50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.75)
                self.setMotors(-50, 50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.5)
                self.setMotors(50, -50)
                R.sleep(0.10)
                self.stop()
            if self.x == 2 or self.x == 3 or self.x == 4:
                self.setMotors(-50, -50)
                R.sleep(0.30)
                self.setMotors(-50, 50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.75)
                self.setMotors(50, -50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.5)
                self.setMotors(-50, 50)
                R.sleep(0.10)
                self.stop()
            self.x = self.x + 1
        if self.couleur == "jaune" :
            if self.x == 1 and self.y == 0 :
                R.sleep(10)
                self.y = self.y + 1
            if self.x > 5:
                self.x = 1
            if self.x == 2 or self.x == 3 or self.x == 4:
                self.setMotors(-50, -50)
                R.sleep(0.30)
                self.setMotors(70, -50)
                R.sleep(0.30)
                self.setMotors(55, 50)
                R.sleep(0.75)
                self.setMotors(-50, 45)
                R.sleep(0.30)
                self.setMotors(60, 40)
                R.sleep(0.5)
                self.setMotors(40, 10)
                R.sleep(0.20)
                self.stop()
            if self.x == 0 or self.x == 1 or self.x == 5:
                self.setMotors(-50, -50)
                R.sleep(0.30)
                self.setMotors(-50, 50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.75)
                self.setMotors(50, -50)
                R.sleep(0.30)
                self.setMotors(50, 50)
                R.sleep(0.5)
                self.setMotors(-50, 50)
                R.sleep(0.10)
                self.stop()
            self.x = self.x + 1

    def detect(self):
        transmitters = R.radio.sweep()
        for tx in transmitters:
            if tx.signal_strength > self.strongestSignal and not tx.target_info.owned_by == R.zone:
                self.strongestSignal = tx.signal_strength
                self.targetBearing = tx.bearing
                self.targetName = tx.target_info.station_code
            R.sleep(0.1)
        return(transmitters)

    def updateBearing(self, name=None):
        if not name: name = self.targetName
        transmitters = R.radio.sweep()
        for tx in transmitters:
            if tx.target_info.station_code == name:
                self.targetBearing = tx.bearing
            R.sleep(0.1)
        return(self.targetBearing)

    def updateStrength(self, name=None):
        if not name: name = self.targetName
        transmitters = R.radio.sweep()
        for tx in transmitters:
            if tx.target_info.station_code == name:
                self.strongestSignal = tx.signal_strength
            R.sleep(0.1)
        return(self.strongestSignal)

    def goToClosest(self):
        print('going to the closest pillar')
        self.detect()
        self.setMotors(v, v)
        while not R.ruggeduinos[0].digital_read(2) and self.strongestSignal<10:
            self.targetBearing = self.updateBearing()
            if R.ruggeduinos[0].analogue_read(1) < 0.20:
                self.goBackward()
                return
            if self.targetBearing > 0:
                self.setMotors(37-float(20*abs(self.targetBearing)), 37+float(20*abs(self.targetBearing)))
            else:
                self.setMotors(37+float(20*abs(self.targetBearing)), 37-float(20*abs(self.targetBearing)))
        print(self.strongestSignal, "exit")
        self.stop()

    def spin(self):
        self.setMotors(-100, 100)
    def wander(self):
        self.setMotors(v, v)
        R.sleep(2)
    def Couleur(self):
        if R.zone == 0 :
            print("violet")
            self.couleur = "violet"
        else :
            self.couleur = "jaune"
            print("jaune")

nav = Nav()
nav.Couleur()
while True:
    nav.detect()
    if R.ruggeduinos[0].analogue_read(1) < 0.20:
        nav.goBackward()
    elif R.ruggeduinos[0].digital_read(2):
        print(R.ruggeduinos[0].analogue_read(1), R.ruggeduinos[0].analogue_read(0))
        nav.avoidPillar()
    elif nav.strongestSignal == 0:
        nav.wander()
    else:
        nav.goToClosest()
        nav.claimTerritory()

































#print("I found {} transmitter(s):".format(len(R.radio.sweep())))

# motor board 0, channel 0 to half power forward
#R.motors[0].m0.power = v

# motor board 0, channel 1 to half power forward
#R.motors[0].m1.power = v

# sleep for 1 second
#R.sleep(1)

# motor board 0, channel 0 to stopped
#R.motors[0].m0.power = 0

# motor board 0, channel 1 to stopped
#R.motors[0].m1.power = 0

# sleep for 2 seconds
#R.sleep(2)

# motor board 0, channel 0 to half power backward
#R.motors[0].m0.power = -v

# motor board 0, channel 1 to half power forward
#R.motors[0].m1.power = v

# sleep for 0.75 seconds
#R.sleep(0.75)

# motor board 0, channel 0 to half power forward
#R.motors[0].m0.power = v

# motor board 0, channel 1 to half power forward
#R.motors[0].m1.power = v

# sleep for 1 second
#R.sleep(1)

# motor board 0, channel 0 to stopped
#R.motors[0].m0.power = 0

# motor board 0, channel 1 to stopped
#R.motors[0].m1.power = 0
