from sr.robot import *
from math import *
import logging
import numpy as np
logging.basicConfig(level=logging.DEBUG)



class Monrobot(Robot):

    timeSinceMouvement = 0
    x = 2
    y = -4.5
    distanceToKeep = 0.10
    targetBearing=0
    strongestSignal=0
    pillarName=''
    targetOwner=''
    lastTarget=''
    targetDistance=0
    positionThreshold= 0.1
    rotationThreshold = 12
    walls = {
        'WM' : {'xA' :  -1.1, 'yA' : 4.3, 'xB' : 0.795, 'yB' : 0.765},
        'WC' : {'xA' : -1.255 , 'yA' : 0.49, 'xB' : 0.595, 'yB' : 0.88},
        'EM' : {'xA' : -1.1 , 'yA' : -4.3, 'xB' : 0.795, 'yB' : -0.765},
        'EC' : {'xA' : -1.255 , 'yA' : -0.49, 'xB' : 0.595, 'yB' : -0.88}
    }
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
    
    wayPoints = {
        'PN':(-3,-1.1),
        'EY':(-1.5,-0.15),
        'PO':(1.5,-0.35),
        'YL':(3,-1.1),
        'BE':(0,0.4),
        'BG':(-4.5,0.1),
        'OX':(-4.1,1.3),
        'TS':(-2.65,1.4),
        'VB':(-1.1 ,1.3),
        'HV':(4.5,0.1),
        'BN':(4.1,1.3),
        'SW':(2.65,1.4),
        'SZ':(1.1,1.3)
    }



    def __init__(self):
        Robot.__init__(self)
        self.startTime = self.time()
        self.leftMotor = self.motors[0].m0
        self.rightMotor = self.motors[0].m1
        if self.zone == 0 :
            self.x = -4.5
            self.theta = self.toPiPi(3.926) #-pi/4
        else :
            self.x = 4.4
            self.theta = self.toPiPi(-2.266) #-3*pi/4
        self.y = -2
        self.actu = self.time()
        self.age = self.time()-self.actu
        self.targetPillar = None
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
        self.theta = self.toPiPi(pi/2 - self.compass.get_heading() )

        transmitters = self.radio.sweep()
        self.transmitters = sorted(transmitters,key=lambda tx: tx.signal_strength, reverse = True)
        temp = []
        for tx in self.transmitters :
            temp.append(tx.target_info.station_code)
        print(temp)
        if len(self.transmitters)>2:
            cst = 1
            tempList = self.transmitters[:3]
            x= [self.pillars[tx.target_info.station_code][0] for tx in tempList]
            y = [self.pillars[tx.target_info.station_code][1] for tx in tempList]
            d = [sqrt(1/tx.signal_strength) for tx in tempList]

            A = np.array([[2*(x[2]-x[0]),2*(y[2]-y[0])],[2*(x[2]-x[1]),2*(y[2]-y[1])]])
            b = np.array([[d[0]**2-d[2]**2+x[2]**2-x[0]**2+y[2]**2-y[0]**2],[d[1]**2-d[2]**2+x[2]**2-x[1]**2+y[2]**2-y[1]**2]])
            try:
                invA = np.linalg.inv(A)
                coord = np.dot(invA,b)
                self.x = coord[0][0]
                self.y= coord[1][0]
                self.actu = self.time()
                logging.debug(f"x = {self.x} et y = {self.y} et orientation en degre= {180*self.theta/pi}")        
            except  :
                logging.warning(f"Exception inversion matrice : ")
                logging.warning(f"A = {A}")
                logging.warning(f"x = {x}")
                logging.warning(f"y = {y}")
                logging.warning(f"d = {d}")
                pass
        elif len(self.transmitters)==2: 
            tempList = self.transmitters[:2]
            x= [self.pillars[tx.target_info.station_code][0] for tx in tempList]
            y = [self.pillars[tx.target_info.station_code][1] for tx in tempList]
            d = [sqrt(1/tx.signal_strength) for tx in tempList]
            R = (x[1]-x[0])**2+(y[1]-y[0])**2
            K = (d[0]**2-d[1]**2)/(2*R)
            if (d[0]**2+d[1]**2)/(2*R)-K**2-1/4 >0: 
                problem=(d[0]**2+d[1]**2)/(2*R)-K**2-1/4
            else : 
                problem = 0
            print( x[0], x[1], y[0], y[1], d[0], d[1], R, K, (d[0]**2+d[1]**2)/(2*R)-K**2-1/4)
            ix1 = (x[0]+x[1])/2+(x[1]-x[0])*(K+sqrt(problem))
            ix2 = (x[0]+x[1])/2+(x[1]-x[0])*(K-sqrt(problem))
            iy1 = (y[0]+y[1])/2+(y[1]-y[0])*(K+sqrt(problem))
            iy2 = (y[0]+y[1])/2+(y[1]-y[0])*(K-sqrt(problem))

            if sqrt( (ix1- self.x )**2 + (iy1- self.y)**2 ) < sqrt( (ix2- self.x )**2 + (iy2- self.y)**2 ) :
                self.x = ix1
                self.y = iy1
            else : 
                self.x = ix2
                self.y = iy2
            self.actu = self.time()
            logging.debug(f"x = {self.x} et y = {self.y} et orientation en degre= {180*self.theta/pi}")             
        else :
            logging.debug("Pas d'actualisation des coordonnees")            
        self.age = self.time()-self.actu
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
        logging.info(f"Target Pillar : {self.pillarName}")
        logging.info(f"Target Way Point : {self.targetWayPoint}")
        logging.info(f"Way Point Bearing : {self.wayPointBearing}")

    def claim(self):
        self.lastTarget=self.targetPillar.target_info.station_code
        self.radio.claim_territory()

    def goBackToStart(self):
        diff=0
        if self.zone == 0 : 
            while not self.ruggeduinos[0].digital_read(1):
                if self.patinage() : self.setMotors(100, 70); self.sleep(0.2) 
                if self.ruggeduinos[0].analogue_read(3)-self.distanceToKeep<0 : diff = 10
                else : diff = -10
                self.setMotors(-100+diff, -100-diff)
                self.sleep(0.1)
            self.setMotors(100,0)
            self.sleep(0.45)
            self.setMotors(100,100)
            self.sleep(0.2)
            return
        while not self.ruggeduinos[0].digital_read(1):
            if self.patinage() : self.setMotors(70, 100); self.sleep(0.2)
            if self.ruggeduinos[0].analogue_read(2)-self.distanceToKeep<0 : diff = 9
            else : diff = -9
            self.setMotors(-100-diff, -100+diff)
            self.sleep(0.1)
        self.setMotors(0,100)
        self.sleep(0.45)
        self.setMotors(100,100)
        self.sleep(0.2)
        return

    def selectTargets(self):
        if len(self.transmitters) == 0 : 
            self.targetPillar = None
            self.targetWayPoint = (0,0)
            return
        if self.lastTarget=='EY' or self.lastTarget=='PO':
            print('je suis coince')
            self.lastTarget=''
            self.goBackToStart()
        if self.lastTarget=='VB'  or self.lastTarget=='SZ':
            for tx in self.transmitters:
                if  (not tx.target_info.owned_by == self.zone) and self.isTargetReachable(tx) and tx.target_info.station_code == 'BE':
                    self.targetPillar = tx
                    self.pillarName = tx.target_info.station_code
                    self.targetWayPoint = self.wayPoints[self.pillarName]
                    return
                self.sleep(0.1)
        if self.lastTarget=='BE':
            for tx in self.transmitters:
                if  (not tx.target_info.owned_by == self.zone) and self.isTargetReachable(tx) and tx.target_info.station_code == 'SZ' or tx.target_info.station_code == 'VB':
                    self.targetPillar = tx
                    self.pillarName = tx.target_info.station_code
                    self.targetWayPoint = self.wayPoints[self.pillarName]
                    return
                self.sleep(0.1)
        for tx in self.transmitters:
            logging.debug(f"Evaluation 1ere cond dans selctT : {(not tx.target_info.owned_by == self.zone)} pour {tx.target_info.station_code}")
            if  (not tx.target_info.owned_by == self.zone) and self.isTargetReachable(tx):
                self.targetPillar = tx
                self.pillarName = tx.target_info.station_code
                self.targetWayPoint = self.wayPoints[self.pillarName]
                return
            self.sleep(0.1)
        self.pillarName = 'AUCUN'
        self.targetPillar = None
        self.targetWayPoint = (0,1.2)
        print("toutes les cibles à portee sont à nous, direction le centre de la map")

    def withinReach(self,xA,xB,yA,yB,xp,yp):
        xR = self.x
        yR = self.y 

        xAB = xB-xA
        yAB = yB-yA

        xRP = xp-xR
        yRP = yp-yR

        if (xAB*yRP-xRP*yAB) == 0 :
            return True
        else :
            t = (yRP * (xR - xA) + xRP * (yA - yR))/(xAB*yRP-xRP*yAB)
            f = (xAB * (yA - yR) + yAB * (xR - xA))/(xAB*yRP-xRP*yAB)
            if t <= 1 and t >= 0 and f <= 1 and f >= 0 :
                return False
            else :
                return True
    
    def isTargetReachable(self,x,y):
        for n,w in self.walls.items():
            if self.withinReach(w['xA'], w['xB'], w['yA'], w['yB'], x, y) == True: pass
            if self.withinReach(w['xA'], w['xB'], w['yA'], w['yB'], x, y) == False: return False
        return True


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

    def setMotors(self,l,r):
        self.leftMotor.power = l
        self.rightMotor.power = r
            
    def patinage(self):
        if self.timeSinceMouvement == 0:
            self.timeSinceMouvement = 1
            self.referencePositionX = self.x
            self.referencePositionY = self.y
            return(False)

        if self.timeSinceMouvement == 3:
            self.timeSinceMouvement = 0
            return(True)

        if 3>self.timeSinceMouvement >= 1 and self.referencePositionX-0.05<self.x<self.referencePositionX+0.05 and self.referencePositionY-0.05<self.y<self.referencePositionY+0.05:
            self.timeSinceMouvement = self.timeSinceMouvement+1
            self.sleep(0.1)
            return(False)
        else : 
            self.timeSinceMouvement = 0
            return(False)

    def getTarget(self, PillarName): 
            for tx in self.transmitters:
                if  (not tx.target_info.owned_by == self.zone) and self.isTargetReachable(tx) and tx.target_info.station_code == PillarName:
                    self.targetPillar = tx
                    self.pillarName = tx.target_info.station_code
                    self.targetWayPoint = self.wayPoints[self.pillarName]
                    return
                self.sleep(0.1)        

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
        elif self.tsAV or self.patinage() :
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
    if R.zone == 0 :
        R.setMotors(79,89)
    else :
        R.setMotors(87,80)
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
R.setMotors(speed,speed)
R.sleep(delay)
while True :
    R.update()
    R.goToTarget()
    R.sleep(delay)