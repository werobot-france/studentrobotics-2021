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
<<<<<<< Updated upstream
    positionThreshold= 0.1
    rotationThreshold = 12
=======
    lastTarget=''
    innerWestWallCoef = (0.528,1.2)
    innerEastWallCoef=(-0.528,1.2)
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

        for tx in transmitters:
            if tx.signal_strength > self.strongestSignal and not tx.target_info.owned_by == self.zone:
                self.strongestSignal = tx.signal_strength
                self.targetBearing = tx.bearing
                self.pillarName = tx.target_info.station_code
                self.targetOwner = tx.target_info.owned_by
=======
        if self.targetPillar == None :
            self.selectTargets()
        else :
            tempPillar = [tx for tx in self.transmitters if tx.target_info.station_code == self.targetPillar.target_info.station_code]
            if len(tempPillar) > 0:
                tempPillar = tempPillar[0]
                if tempPillar.target_info.owned_by == self.zone :
                    logging.debug(f"Ancienne cible {self.targetPillar.target_info.station_code} = {tempPillar.target_info.station_code} deja a moi. Changement.")
                    self.selectTargets()
                else :
                    self.targetPillar = tempPillar
            else :
                self.selectTargets()
        self.wayPointBearing = self.toPiPi(self.theta - (atan2(-(self.targetWayPoint[1]-self.y),self.targetWayPoint[0]-self.x)))
        logging.info(f" Info sur les cibles :")
        logging.info(f"-Target Pillar : {self.pillarName}")
        logging.info(f"-Target Way Point : {self.targetWayPoint}")
        logging.info(f"-Way Point Bearing : {self.wayPointBearing}")


    def selectTargets(self):
        if len(self.transmitters) == 0 : 
            self.targetPillar = None
            self.targetWayPoint = (0,0)
            return
        if self.lastTarget=='EY':
            self.pillarName = 'AUCUN'
            self.targetPillar = None
            self.targetWayPoint = (-4.37,-1.58)
            return
        for tx in self.transmitters:
            logging.debug(f"Evaluation 1ere cond dans selctT : {(not tx.target_info.owned_by == self.zone)} pour {tx.target_info.station_code}")
            if  (not tx.target_info.owned_by == self.zone) and self.isTargetReachable(tx):
                self.targetPillar = tx
                self.pillarName = tx.target_info.station_code
                self.targetWayPoint = self.wayPoints[self.pillarName]
                print(self.targetPillar)
                return
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream



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
            elif sqrt(1/self.strongestSignal)<0.50: R.radio.claim_territory()
            elif self.targetBearing > 0:
                self.setMotors(70-float(25*abs(self.targetBearing)), 70+float(25*abs(self.targetBearing)))
            elif self.targetBearing >0 :
                self.setMotors(70+float(25*abs(self.targetBearing)), 70-float(25*abs(self.targetBearing)))
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
=======
    def orientToTarget (self):
        logging.debug("Entree dans orientToTarget")
        while -pi/4 < self.wayPointBearing > pi/4 :
            R.update()
            if self.wayPointBearing > 0 :
                self.setMotors(21*(self.wayPointBearing),-21*(self.wayPointBearing))
                logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")

            elif self.wayPointBearing < 0 :
                self.setMotors(21*(self.wayPointBearing),-21*(self.wayPointBearing))
                logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")
            self.sleep(0.1)
        return
    def claim(self):
        self.lastTarget=self.targetPillar.target_info.station_code
        self.radio.claim_territory()

    def goToTarget(self):
        logging.debug("Entree dans goToTarget")
        try :
            dist = sqrt(1/self.targetPillar.signal_strength)
            logging.debug(f"Distance au pillar {self.targetPillar.target_info.station_code} : {dist}")
        except :
            logging.debug("Distance au pillier non calculable, distance fixee à 3m")
            dist = 3
        
        if  dist < 0.48 :
            self.setMotors(0,0)
            logging.debug("Arret et claim")
            self.claim()
        elif self.tsAV :
            logging.debug("Coince a l avant : je recule")
            if self.wayPointBearing > 0 :
                logging.debug("Nez vers la droite")
                self.setMotors(-100*cos(self.wayPointBearing),-100)
            else :
                logging.debug("Nez vers la gauche")
                self.setMotors(-100,-100*cos(self.wayPointBearing))
            self.sleep(0.2)
        elif self.wayPointBearing > 0 :
            logging.debug("Bifurque à droite")
            self.setMotors(speed,speed*cos(self.wayPointBearing)**3)
            
        elif self.wayPointBearing < 0 :
            logging.debug("Bifurque à gauche")            
            self.setMotors(speed*cos(self.wayPointBearing)**3,speed)         
        logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")


        
       
R = Monrobot()
speed = 70
delay = 0.1

def reachFirstPillar(R):
    dist = 1
    firstPillar = R.transmitters[0]
    if R.zone==0: R.setMotors(79,89)
    else : R.setMotors(89,79)
    while dist > 0.48 :
        R.sleep(0.1)
        R.update()
        firstPillar = [tx for tx in R.transmitters if tx.target_info.station_code == firstPillar.target_info.station_code][0]
        dist = sqrt(1/firstPillar.signal_strength)
        logging.debug(f"Distance au 1st pillar {firstPillar.target_info.station_code} : {dist}")
    R.setMotors(0,0)
    R.claim()
    logging.debug("first pillar should be claimed")


reachFirstPillar(R)
>>>>>>> Stashed changes
R.setMotors(speed,speed)
while True :
    R.update()
<<<<<<< Updated upstream
    R.lookToTarget()
=======
    R.orientToTarget()
>>>>>>> Stashed changes
    R.goToTarget()
    R.sleep(delay)
