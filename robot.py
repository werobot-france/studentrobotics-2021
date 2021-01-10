from sr.robot import *
from math import *
import logging
import numpy as np
logging.basicConfig(level=logging.INFO)



class Monrobot(Robot):
    targetBearing=0
    strongestSignal=0
    pillarName=''
    targetOwner=''
    targetDistance=0
    positionThreshold= 0.1
    rotationThreshold = 12
    upperPillars=['PN', 'EY', 'PO', 'YL']
    midlePillar=['BE']
    lowerPillars=['BG', 'OX', 'TS', 'VB', 'HV', 'BN', 'SW', 'SZ']
    pillars = {
        'PN':(-3,-1.5),
        'EY':(-1.5,-0.75),
        'PO':(1.5,-0.75),
        'YL':(3,-1.5),
        'BE':(0,0),
        'BG':(-4.1,0.1),
        'OX':(-4.1,1.7),
        'TS':(-2.65,0.9),
        'VB':(-1.1,1.7),
        'HV':(4.1,0.1),
        'BN':(4.1,1.7),
        'SW':(2.65,0.9),
        'SZ':(1.1,1.7)
        }
    targets = {
        'PN':(-3.1,-1.1),
        'EY':(-1.6,-0.35),
        'PO':(1.6,-0.35),
        'YL':(3.1,-1.1),
        'BE':(0,-0.4),
        'BG':(-3.9,0.3),
        'OX':(-3.9,1.9),
        'TS':(-2.85,1.1),
        'VB':(-1 ,2),
        'HV':(3.9,0.3),
        'BN':(3.9,1.9),
        'SW':(2.85,1.1),
        'SZ':(1,2)
        }



    def __init__(self):
        Robot.__init__(self)
        self.leftMotor = self.motors[0].m0
        self.rightMotor = self.motors[0].m1
        if self.zone == 0 :
            self.x = -4.5
            self.theta = -pi/4
        else :
            self.x = 4.4
            self.theta = -3*pi/4
        self.y = -2
        self.actu = self.time()
        self.age = self.time()-self.actu
        self.update()

    def toPiPi(self,angle) :
        while angle > pi :
            angle -= 2*pi
        while angle <= -pi :
            angle += 2*pi
        return angle
    
    def update(self) :
        self.dsAVD = self.ruggeduinos[0].analogue_read(1)
        self.dsAVG = self.ruggeduinos[0].analogue_read(0)
        self.dsG = self.ruggeduinos[0].analogue_read(2)
        self.dsD = self.ruggeduinos[0].analogue_read(3)
        self.dsARG = self.ruggeduinos[0].analogue_read(4)
        self.dsARD = self.ruggeduinos[0].analogue_read(5)
        self.tsAV = self.ruggeduinos[0].digital_read(0)
        self.tsAR = self.ruggeduinos[0].digital_read(1)

        transmitters = self.radio.sweep()
        self.transmitters = sorted(transmitters,key=lambda tx: tx.signal_strength, reverse = True)
        if len(self.transmitters)>2:
            cst = 1
            tempList = self.transmitters[:3]
            x= [self.pillars[tx.target_info.station_code][0] for tx in tempList]
            y = [self.pillars[tx.target_info.station_code][1] for tx in tempList]
            d = [sqrt(1/tx.signal_strength) for tx in tempList]

            A = np.array([[2*(x[2]-x[0]),2*(y[2]-y[0])],[2*(x[2]-x[1]),2*(y[2]-y[1])]])
            b = np.array([[d[0]**2-d[2]**2+x[2]**2-x[0]**2+y[2]**2-y[0]**2],[d[1]**2-d[2]**2+x[2]**2-x[1]**2+y[2]**2-y[1]**2]])
            invA = np.linalg.inv(A)
            coord = np.dot(invA,b)
            self.x = coord[0][0]
            self.y= coord[1][0]

            P = self.pillars[tempList[-1].target_info.station_code]
            logging.info(f"Pillier etudie : {tempList[-1].target_info.station_code} de coordonnees {P}")
            logging.info(f"Coordonnees actuelles : ({self.x},{self.y})")
            logging.info(f"Bearing = {tempList[-1].bearing} par rapport à {tempList[-1].target_info.station_code}")
            logging.info(f"vecteur RP : ({P[0]-self.x},{P[1]-self.y})")
            logging.info(f"resultat du atan2 : {atan2(P[1]-self.y,P[0]-self.x)}")

            self.theta = self.toPiPi(tempList[2].bearing + atan2(-(P[1]-self.y),P[0]-self.x))
            self.actu = self.time()
        self.age = self.time()-self.actu

        for tx in transmitters:
            if tx.signal_strength > self.strongestSignal and not tx.target_info.owned_by == self.zone:
                self.strongestSignal = tx.signal_strength
                self.targetBearing = tx.bearing
                self.pillarName = tx.target_info.station_code
                self.targetOwner = tx.target_info.owned_by
            self.sleep(0.1)



    def printState(self):
        print(f"DS AVANT : G {self.dsAVG} et D {self.dsAVD}")
        print(f"DS GAUCHE : {self.dsG} et DROIT : {self.dsD}")
        print(f"DS ARRIERE : G {self.dsARG} et D {self.dsARD}")
        print("------------------------------------------------")
        print(f"x = {self.x} et y = {self.y} et orientation en degre= {180*self.theta/pi}")
        print(f"Temps depuis la derniere actualisation des coordonnees : {self.age}")
        print("* * * * * * * * * * * * * * * * * * * * * * * * *")
        if len(self.transmitters)>0 :
            logging.info(f"Signal le plus fort actuellement : {self.transmitters[0].target_info.station_code} : {self.transmitters[0].signal_strength}")

    def setMotors(self,r,l):
        self.leftMotor.power = l
        self.rightMotor.power = r




    def updateTarget(self, name=None):
        if not name: name = self.pillarName
        transmitters = R.radio.sweep()
        for tx in transmitters:
            if tx.target_info.station_code == name:
                self.strongestSignal = tx.signal_strength
        R.sleep(0.1)
        
        self.targetBearing = -(self.theta - atan2(self.pillars.self.pillarName[1]/self.pillars.self.pillarName[0]))
        self.targetDistance =  sqrt((self.targets[self.pillarName][0]-self.x)**2 - (self.targets[self.pillarName][1]-self.y)**2)

    def arrived(self):
        if self.targetDistance<0.50 :
            if self.x>self.targets[self.pillarName][0]-self.positionThreshold and self.Y>self.targets[self.pillarName][1]-self.threshold and self.x<self.targets[self.pillarName][0]+self.threshold and self.Y<self.targets[self.pillarName][1]+self.threshold:
                return True
        return False  
    def orriented(self):
        if -pi/self.rotationThreshold<self.targetBearing<pi/self.rotationThreshold:
            return True
        return False


    def lookToTarget(self):
        while not self.orriented() :
            if self.targetBearing > 0:
                self.setMotors(-float(20*abs(self.targetBearing)), +float(20*abs(self.targetBearing)))
            else :
                self.setMotors(+float(20*abs(self.targetBearing)), -float(20*abs(self.targetBearing)))
        print('looking at the target')


    def goToTarget(self):
         print('going to the target')
         self.setMotors(speed, speed)
         while not self.arrived() :
            self.updateTarget
            if self.tsAV :
                return
            elif self.targetBearing > 0:
                self.setMotors(80-float(20*abs(self.targetBearing)), 80+float(20*abs(self.targetBearing)))
            elif self.targetBearing >0 :
                self.setMotors(80+float(20*abs(self.targetBearing)), 80-float(20*abs(self.targetBearing)))
            elif sqrt(1/self.strongestSignal) : R.radio.claim_territory()
         print('arrived')
         self.stop()




R = Monrobot()
speed = 70
delay = 0.05
def reachFirstPillar():
    R.setMotors(99, 87)
    R.sleep(1.56)
    R.setMotors(0,0)
    R.radio.claim_territory()
reachFirstPillar()
R.setMotors(speed,speed)
while True :
    R.update()
    R.lookToTarget()
    R.goToTarget()
    R.sleep(delay)





"""

    positionOfTransmittersInRange=[]
    transmittersRelativeDistance=[]
    robotDistanceFromPillars=[]
    robotPosition = np.array(0,0)

    def triangulatePosition(self):
        transmitters = R.radio.sweep()
        for tx in transmitters:
            self.positionOfTransmittersInRange.append(self.pillarsPosition.tx.target_info.station_code)
            self.robotDistanceFromPillars.append(sqrt(1/tx.signal_strength))
        if len(self.positionOfTransmittersInRange)>=3 :
            B = np.array(2*[self.positionOfTransmittersInRange[2][0]*self.positionOfTransmittersInRange[0][0], self.positionOfTransmittersInRange[2][1]*self.positionOfTransmittersInRange[0][1] ],
            2*[self.positionOfTransmittersInRange[2][0]*self.positionOfTransmittersInRange[1][0], self.positionOfTransmittersInRange[2][1]*self.positionOfTransmittersInRange[1][1] ])
            V = np.array(self.robotDistanceFromPillars[0]**2-self.robotDistanceFromPillars[2]**2+self.positionOfTransmittersInRange[2][0]**2-self.positionOfTransmittersInRange[2][1]**2-self.positionOfTransmittersInRange[0][1]**2,
            self.robotDistanceFromPillars[1]**2-self.robotDistanceFromPillars[2]**2+self.positionOfTransmittersInRange[2][0]**2-self.positionOfTransmittersInRange[2][1]**2-self.positionOfTransmittersInRange[1][1]**2)
            self.robotPosition = np.linalg.inv(B)*V
        print(self.robotPosition)

    def reachPillar1(self):
        self.setMotors(99, 87)
        R.sleep(1.56)
        self.stop()
        nav.claimTerritory()
        self.setMotors(89, 100)
        R.sleep(1.3)
        self.stop()
        nav.claimTerritory()



theta - angle U RP'




 class Nav():
     targetBearing=0
     strongestSignal=0
     targetName=''
     targetOwner=''
     rightM=0.0
     leftM=0.0
     x = 0
     couleur = ""
     opponentZone= abs(R.zone-1)

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

     def waitUntilPillarIsClaimed(self):
         cd=0
         if self.targetName=='BE':
             while (not self.targetOwner == self.opponentZone) and cd<10:
                 cd+=1
                 R.sleep(1)


     def detect(self):
         transmitters = R.radio.sweep()
         for tx in transmitters:
             if tx.signal_strength > self.strongestSignal and not tx.target_info.owned_by == R.zone:
                 self.strongestSignal = tx.signal_strength
                 self.targetBearing = tx.bearing
                 self.targetName = tx.target_info.station_code
                 self.targetOwner = tx.target_info.owned_by
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

     def updateOwner(self, name=None):
         if not name: name = self.targetName
         transmitters = R.radio.sweep()
         for tx in transmitters:
             if tx.target_info.station_code == name:
                 self.targetOwner = tx.target_info.owned_by
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
         nav.waitUntilPillarIsClaimed()
         nav.goToClosest()
         nav.claimTerritory()





print("I found {} transmitter(s):".format(len(R.radio.sweep())))

 motor board 0, channel 0 to half power forward
R.motors[0].m0.power = v

 motor board 0, channel 1 to half power forward
R.motors[0].m1.power = v

 sleep for 1 second
R.sleep(1)

 motor board 0, channel 0 to stopped
R.motors[0].m0.power = 0

 motor board 0, channel 1 to stopped
R.motors[0].m1.power = 0

 sleep for 2 seconds
R.sleep(2)

 motor board 0, channel 0 to half power backward
R.motors[0].m0.power = -v

 motor board 0, channel 1 to half power forward
R.motors[0].m1.power = v

 sleep for 0.75 seconds
R.sleep(0.75)

 motor board 0, channel 0 to half power forward
R.motors[0].m0.power = v

 motor board 0, channel 1 to half power forward
R.motors[0].m1.power = v

 sleep for 1 second
R.sleep(1)

 motor board 0, channel 0 to stopped
R.motors[0].m0.power = 0

 motor board 0, channel 1 to stopped
R.motors[0].m1.power = 0
"""