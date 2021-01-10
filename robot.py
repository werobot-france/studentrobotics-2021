# Branch Franck
from sr.robot import *
from math import *
import logging

logging.basicConfig(level=logging.WARNING)


class Monrobot(Robot):
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
    def __init__(self):
        Robot.__init__(self)
        self.leftMotor = self.motors[0].m0
        self.rightMotor = self.motors[0].m1
        if self.zone == 0 :
            self.x = -4.5
        else :
            self.x = 4.4
        self.y = -2
        self.actu = self.time()
        self.age = self.time()-self.actu
        self.update()
    
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
            tempList = self.transmitters[:3]
            sumStrength = sum(list(sqrt(tx.signal_strength) for tx in tempList))
            self.x = 0
            self.y = 0
            for tx in tempList :
                coordPillars = self.pillars[tx.target_info.station_code]
                self.x += coordPillars[0]*sqrt(tx.signal_strength)/sumStrength
                self.y += coordPillars[1]*sqrt(tx.signal_strength)/sumStrength
            self.actu = self.time()
        elif len(self.transmitters) == 2 :
            dist = [sqrt(1/tx.signal_strength) for tx in self.transmitters]
            coord = [self.pillars[tx.target_info.station_code] for tx in self.transmitters]

            a = (coord[1][0]**2 + coord[1][1]**2 - coord[0][0]**2 - coord[0][1]**2 + dist[0]**2 - dist[1]**2)/(2*(coord[1][1]-coord[0][1]))
            d = (coord[1][0]-coord[0][0])/(coord[1][1]-coord[0][1])
            A = d**2+1
            B = -2*coord[0][0]+2*coord[0][1]*d-2*a*d
            C = coord[0][0]**2 + coord[0][1]**2 - 2*coord[0][1]*a + a**2 - dist[0]**2
            Delta = B**2-4*A*C
            x1 = (-B-sqrt(Delta))/(2*a)
            x2 = (-B+sqrt(Delta))/(2*a)
            if (self.x-x1)**2 < (self.x-x2)**2 :
                self.x = x1
            else :
                self.x = x2
            self.y = a-d*self.x
            self.actu = self.time()
        self.age = self.time()-self.actu


    
    def printState(self):
        print(f"DS AVANT : G {self.dsAVG} et D {self.dsAVD}")
        print(f"DS GAUCHE : {self.dsG} et DROIT : {self.dsD}")
        print(f"DS ARRIERE : G {self.dsARG} et D {self.dsARD}")
        print("------------------------------------------------")
        print(f"x = {self.x} et y = {self.y}")
        print(f"Temps depuis la derniere actualisation des coordonnees : {self.age}")
        print("* * * * * * * * * * * * * * * * * * * * * * * * *")

    def setMotors(self,l,r):
        self.leftMotor.power = l
        self.rightMotor.power = r




R = Monrobot()
speed = 0
delay = 0.1
R.setMotors(speed,speed)
while True :
    R.update()
    R.printState()
    R.sleep(delay)






# class Nav():
#     targetBearing=0
#     strongestSignal=0
#     targetName=''
#     targetOwner=''
#     rightM=0.0
#     leftM=0.0
#     x = 0
#     couleur = ""
#     opponentZone= abs(R.zone-1)

#     def forgetPreviousTarget(self):
#         self.targetBearing=0
#         self.strongestSignal=0
#         self.targetName=''

#     def claimTerritory(self):
#         print('Claiming a territory')
#         R.radio.claim_territory()
#         self.forgetPreviousTarget()

#     def setMotors(self, r, l):
#         self.rightM=r
#         self.leftM=l
#         R.motors[0].m1.power = r
#         R.motors[0].m0.power = l

#     def stop(self):
#         self.setMotors(0,0)

#     def goBackward(self):
#         print("going backward")
#         while R.ruggeduinos[0].analogue_read(0) < 0.20:
#             print(R.ruggeduinos[0].analogue_read(0))
#             self.setMotors(-50, -50)
#             R.sleep(0.1)
#     def rotate(self):
#         self.setMotors(-100, 100)
#         R.sleep(0.80)
#         self.stop()

#     def avoidPillar(self):
#         print('avoiding a pillar')
#         if self.couleur == "violet" :
#             if self.x > 5:
#                 self.x = 1
#             if self.x == 0 or self.x == 1 or self.x == 5:
#                 self.setMotors(-50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.75)
#                 self.setMotors(-50, 50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.5)
#                 self.setMotors(50, -50)
#                 R.sleep(0.10)
#                 self.stop()
#             if self.x == 2 or self.x == 3 or self.x == 4:
#                 self.setMotors(-50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(-50, 50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.75)
#                 self.setMotors(50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.5)
#                 self.setMotors(-50, 50)
#                 R.sleep(0.10)
#                 self.stop()
#             self.x = self.x + 1
#         if self.couleur == "jaune" :
#             if self.x > 5:
#                 self.x = 1
#             if self.x == 2 or self.x == 3 or self.x == 4:
#                 self.setMotors(-50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(70, -50)
#                 R.sleep(0.30)
#                 self.setMotors(55, 50)
#                 R.sleep(0.75)
#                 self.setMotors(-50, 45)
#                 R.sleep(0.30)
#                 self.setMotors(60, 40)
#                 R.sleep(0.5)
#                 self.setMotors(40, 10)
#                 R.sleep(0.20)
#                 self.stop()
#             if self.x == 0 or self.x == 1 or self.x == 5:
#                 self.setMotors(-50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(-50, 50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.75)
#                 self.setMotors(50, -50)
#                 R.sleep(0.30)
#                 self.setMotors(50, 50)
#                 R.sleep(0.5)
#                 self.setMotors(-50, 50)
#                 R.sleep(0.10)
#                 self.stop()
#             self.x = self.x + 1

#     def waitUntilPillarIsClaimed(self):
#         cd=0
#         if self.targetName=='BE':
#             while (not self.targetOwner == self.opponentZone) and cd<10:
#                 cd+=1
#                 R.sleep(1)


#     def detect(self):
#         transmitters = R.radio.sweep()
#         for tx in transmitters:
#             if tx.signal_strength > self.strongestSignal and not tx.target_info.owned_by == R.zone:
#                 self.strongestSignal = tx.signal_strength
#                 self.targetBearing = tx.bearing
#                 self.targetName = tx.target_info.station_code
#                 self.targetOwner = tx.target_info.owned_by
#             R.sleep(0.1)
#         return(transmitters)

#     def updateBearing(self, name=None):
#         if not name: name = self.targetName
#         transmitters = R.radio.sweep()
#         for tx in transmitters:
#             if tx.target_info.station_code == name:
#                 self.targetBearing = tx.bearing
#             R.sleep(0.1)
#         return(self.targetBearing)

#     def updateStrength(self, name=None):
#         if not name: name = self.targetName
#         transmitters = R.radio.sweep()
#         for tx in transmitters:
#             if tx.target_info.station_code == name:
#                 self.strongestSignal = tx.signal_strength
#             R.sleep(0.1)
#         return(self.strongestSignal)

#     def updateOwner(self, name=None):
#         if not name: name = self.targetName
#         transmitters = R.radio.sweep()
#         for tx in transmitters:
#             if tx.target_info.station_code == name:
#                 self.targetOwner = tx.target_info.owned_by
#             R.sleep(0.1)
#         return(self.strongestSignal)

#     def goToClosest(self):
#         print('going to the closest pillar')
#         self.detect()
#         self.setMotors(v, v)
#         while not R.ruggeduinos[0].digital_read(2) and self.strongestSignal<10:
#             self.targetBearing = self.updateBearing()
#             if R.ruggeduinos[0].analogue_read(1) < 0.20:
#                 self.goBackward()
#                 return
#             if self.targetBearing > 0:
#                 self.setMotors(37-float(20*abs(self.targetBearing)), 37+float(20*abs(self.targetBearing)))
#             else:
#                 self.setMotors(37+float(20*abs(self.targetBearing)), 37-float(20*abs(self.targetBearing)))
#         print(self.strongestSignal, "exit")
#         self.stop()

#     def spin(self):
#         self.setMotors(-100, 100)
#     def wander(self):
#         self.setMotors(v, v)
#         R.sleep(2)
#     def Couleur(self):
#         if R.zone == 0 :
#             print("violet")
#             self.couleur = "violet"
#         else :
#             self.couleur = "jaune"
#             print("jaune")

# nav = Nav()
# nav.Couleur()
# while True:
#     nav.detect()
#     if R.ruggeduinos[0].analogue_read(1) < 0.20:
#         nav.goBackward()
#     elif R.ruggeduinos[0].digital_read(2):
#         print(R.ruggeduinos[0].analogue_read(1), R.ruggeduinos[0].analogue_read(0))
#         nav.avoidPillar()
#     elif nav.strongestSignal == 0:
#         nav.wander()
#     else:
#         nav.waitUntilPillarIsClaimed()
#         nav.goToClosest()
#         nav.claimTerritory()

































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
