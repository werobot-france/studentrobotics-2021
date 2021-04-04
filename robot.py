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
    reverse = False
    walls = {
        'WM' : {'xA' :  -1.1, 'yA' : 4.3, 'xB' : 0.795, 'yB' : 0.765},
        'WC' : {'xA' : -1.255 , 'yA' : 0.49, 'xB' : 0.595, 'yB' : 0.88},
        'EM' : {'xA' : -1.1 , 'yA' : -4.3, 'xB' : 0.795, 'yB' : -0.765},
        'EC' : {'xA' : -1.255 , 'yA' : -0.49, 'xB' : 0.595, 'yB' : -0.88}
    }
    # Rose
    pointsToCheck0 = {
    #    'OX' : {''},
        'TH' : {'PN'},
    #    'PN' : {''},
    #    'BG' : {''},
        'TS' : {'OX'},
        'EY' : {'PN'},
        'VB' : {'OX'},
        'FL' : {'EY'},
        'YT' : {'HA'},
        'HA' : {'BE'},
        'BE' : {'VB'},
        'PL' : {'VB'},
        'PO' : {'FL'},
        'SZ' : {'PO'},
        'SW' : {'BN'},
        'YL' : {'PO'},
        'HV' : {'SZ'},
        'SF' : {'YL'},
        'BN' : {'SZ'}
    }

    # Jaune
    pointsToCheck1 = {
        'OX' : {'VB'},
        'TH' : {'PN'},
        'PN' : {'EY'},
        'BG' : {'VB'},
        'TS' : {'OX'},
        'EY' : {'FL'},
        'VB' : {'BE'},
        'FL' : {'PO'},
        'YT' : {'HA'},
        'HA' : {'BE'},
        'BE' : {'SZ'},
        'PL' : {'SZ'},
        'PO' : {'YL'},
        'SZ' : {'BN'},
        'SW' : {'BN'},
    #    'YL' : {''},
    #    'HV' : {''},
        'SF' : {'YL'},
    #    'BN' : {''}
    }

    pillars = {
        "PN" : (-4.2, -1.8),
        "EY" : (-1.95, -0.75),
        "BE" : (0, 1.5),
        "PO" : (1.95, -0.75),
        "YL" : (4.2, -1.8),
        "BG" : (-4.2, 0),
        "OX" : (-6.6, 3),
        "TS" : (-2.75, 2.75),
        "VB" : (-1.95, 0.75),
        "HV" : (4.2, 0),
        "BN" : (6.6, 3),
        "SW" : (2.75, 2.75),
        "SZ" : (1.95, 0.75),
        "FL" : (0, -3),
        "YT" : (0, -1.5),
        "HA" : (0, 0),
        "PL" : (0, 3),
        "TH" : (-6.6, -3),
        "SF" : (6.6, -3),
    }

    wayPointsToPillars = {
        "PN" : (-4, -2.00, False),
        "EY" : (-1.95,-1.15, False),
        "BE" : (0, 1.25, False),
        "PO" : (1.95,-1.25, False),
        "YL" : (4, -2, False),
        "BG" : (-3.90, 0, False),
        "OX" : (-6.4, 2.80, False),
        "TS" : (-2.95, 2.55, False),
        "VB" : (-2.15, 0.55, False),
        "HV" : (3.9, 0, False),
        "BN" : (6.4, 2.8, False),
        "SW" : (2.95, 2.55, False),
        "SZ" : (1.75, 0.55, False),
        "FL" : (0, -2.6, False),
        "YT" : (0, -1.85, False),
        "HA" : (0, 0.4, False),
        "PL" : (0, 2.6, False),
        "TH" : (-6.25, -3, False),
        "SF" : (6.25, -3, False),
    }
    wayPoints0 = ["PN",  "EY", "FL", "PO", "YL", "SZ" ]
    wayPoints1 = ["BN", "SW", "SZ", "HV", "YL",  "PO", "FL", "EY", "PN" ]

    def __init__(self):
        Robot.__init__(self)
        self.startTime = self.time()
        self.leftMotor = self.motors[0].m0
        self.rightMotor = self.motors[0].m1
        self.opzone=abs(self.zone-1)
        if self.zone == 0 :
            self.x = -4.5
            self.wayPoints = self.wayPoints0
            self.pointsToCheck = self.pointsToCheck0
            #self.theta = self.toPiPi(3.926) #-pi/4
        else :
            self.x = 4.4
            self.wayPoints = self.wayPoints1
            self.pointsToCheck = self.pointsToCheck1
            #self.theta = self.toPiPi(-2.266) #-3*pi/4
        self.theta = self.toPiPi(pi/2 - self.compass.get_heading() )
        logging.debug(f"theta initial : {self.theta}")
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
        print(self.transmitters)

        if self.isClaimable():
            self.claim()
            if self.outcome()=="fail":
                self.addCheckpoint(self.transmitters[0].target_info.station_code)
                return

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

            print("les coordonnees en x1 et x2 des deux centres :", x[0], x[1],  "les coordonnees en y1 et y2 des deux centres :", y[0], y[1])
            print("les rayons des deux cercles/les distances aux centres :", d[0], d[1])

            ix1,iy1,ix2,iy2 = self.getIntersections(x[0],y[0],d[0],x[1],y[1],d[1])

            print('les coordonnees du premier point d\'intersection :', ix1, iy1)
            print('les coordonnees du second point d\'intersection:', ix2, iy2)
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


        if sqrt((self.x-self.wayPointsToPillars.get(self.wayPoints[0])[0]**2)+(self.y-self.wayPointsToPillars.get(self.wayPoints[0])[1])**2<0.1) :
            logging.debug("Cible atteinte : waypoint suivant") 
            self.wayPoints.pop(0)
        self.wayPointBearing = self.toPiPi(self.theta - (atan2(-(self.wayPointsToPillars.get(self.wayPoints[0])[1]-self.y),self.wayPointsToPillars.get(self.wayPoints[0])[0]-self.x)))
        self.reverse = self.wayPointsToPillars.get(self.wayPoints[0])[2]

        logging.info(f" Info sur les cibles :")
        logging.info(f" Way Point Targetted : { self.wayPointsToPillars.get(self.wayPoints[0])[0] }")
        logging.info(f"Way Point Bearing : {self.wayPointBearing}")


    def isClaimable(self):
        if not self.transmitters == [] :
            if sqrt(1/self.transmitters[0].signal_strength)<0.5 and (not self.transmitters[0].target_info.owned_by == self.zone) :
                return True
            return False
        return False

    def claim(self):
        self.setMotors(0,0)
        self.radio.claim_territory()


    def outcome(self):
        if self.transmitters[0].target_info.owned_by == self.zone:
            return "succes"
        if not self.transmitters[0].target_info.owned_by == self.zone:
            if self.transmitters[0].target_info.owned_by == self.opzone:
                self.claim()
            else :
                return "fail"

    def addCheckpoint(self, obj):
        self.wayPoints.insert(0, self.pointsToCheck[obj])

    def getIntersections(self, x0, y0, r0, x1, y1, r1):
        d=sqrt((x1-x0)**2 + (y1-y0)**2)

        a=(r0**2-r1**2+d**2)/(2*d)

        if r0**2-a**2 < 0 : 
            h=0
        else : 
            h=sqrt(r0**2-a**2)

        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return (x3, y3, x4, y4)

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
        # seuil d'origine : seuil = 0.05
        seuil = 0.07
        
        if self.timeSinceMouvement == 0:
            self.timeSinceMouvement = 1
            self.referencePositionX = self.x
            self.referencePositionY = self.y
            logging.debug(f"Patinage : premier tour")
            return(False)
        logging.debug(f"Patinage : refPos = ({self.referencePositionX},{self.referencePositionX}) currPos = ({self.x},{self.y})")
        d = (self.referencePositionX - self.x)**2 + (self.referencePositionY - self.y)**2
        d = sqrt(d)
        logging.debug(f"Patinage : d = {d}")
        if self.timeSinceMouvement == 3:
            if d > seuil : self.timeSinceMouvement = 0
            return(True)
        
        #if 3>self.timeSinceMouvement >= 1 and self.referencePositionX-seuil<self.x<self.referencePositionX+seuil and self.referencePositionY-seuil<self.y<self.referencePositionY+seuil:
        if d < seuil:
            self.timeSinceMouvement = self.timeSinceMouvement+1
            self.sleep(0.1)
            logging.debug(f"Patinage detecte : tour num {self.timeSinceMouvement-1}")
            return(False)
        else : 
            self.timeSinceMouvement = 0
            logging.debug(f"Patinage : reset")
            return(False)    


    def orientTo(self, wa):
        logging.debug("Entree dans orientTo")
        while -pi/9 < self.wayPointBearing > pi/9 :
            self.wayPointBearing = self.toPiPi(self.theta - (atan2(-(wa[1]-self.y),wa[0]-self.x)))
            if self.wayPointBearing > 0 :
                self.setMotors(-(cos(self.wayPointBearing)-1)**2*50,(cos(self.wayPointBearing)-1)**2*50)
                logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")

            if self.wayPointBearing < 0 :
                self.setMotors((cos(self.wayPointBearing)-1)**2*50,-(cos(self.wayPointBearing)-1)**2*50)
                logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")
            self.sleep(0.1)


    def goToTarget(self):
        logging.debug("Entree dans goToTarget")
        # expo d'origine : 3
        expo = 4
        if self.reverse : self.wayPointBearing = self.toPiPi(self.wayPointBearing + pi)
        patine = self.patinage()

        if self.tsAV or (patine and not self.reverse and not self.tsAR):
            logging.debug("Coince a l avant : je recule")
            if self.wayPointBearing > 0 :
                logging.debug("Nez vers la droite")
                left=-100*cos(self.wayPointBearing)
                right = -100
            else :
                logging.debug("Nez vers la gauche")
                left=-100
                right=-100*cos(self.wayPointBearing)
        elif self.tsAR or (patine and self.reverse):
            logging.debug("Coince a l'arriere : j'avance")
            if self.wayPointBearing > 0 :
                logging.debug("Nez vers la droite")
                left=-100*cos(self.wayPointBearing) # on fait le contraire de l'effet recherché
                right = -100                        # car à la fin on inverse dans le cas reverse
               
            else :
                logging.debug("Nez vers la gauche")
                left=-100                           # idem !
                right=-100*cos(self.wayPointBearing)
               


        elif self.wayPointBearing > 0 :
            logging.debug("Bifurque à droite")
            left = speed
            right = speed*cos(self.wayPointBearing)**expo
            
        elif self.wayPointBearing < 0 :
            logging.debug("Bifurque à gauche")            
            left = speed*cos(self.wayPointBearing)**expo
            right = speed
        

        if self.reverse :
            left,right = -right,-left
        self.setMotors(left,right)
        logging.debug(f"moteur gauche : {self.leftMotor.power} moteur droit : {self.rightMotor.power}")
       
R = Monrobot()
speed = 70
delay = 0.1

while True :
    R.update()
    R.goToTarget()
    R.sleep(delay)