from sr.robot import *
from math import *
import logging
import numpy as np
logging.basicConfig(level=logging.DEBUG)

class Monrobot(Robot):

    timeSinceMouvement = 0
    reClaim=0
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
        'OX' : ['OX'],
        'TH' : ['PN'],
        'PN' : ['PN'],
        'BG' : ['BG'],
        'TS' : ['OX'],
        'EY' : ['PN'],
        'VB' : ['BG'],
        'FL' : ['EY'],
        'YT' : ['HA'],
        'HA' : ['BE'],
        'BE' : ['SZ'],
        'PL' : ['VB'],
        'PO' : ['FL'],
        'SZ' : ['A3', 'A2', 'PO', 'A2', 'A3'],
        'SW' : ['BN'],
        'YL' : ['PO'],
        'HV' : ['SZ'],
        'SF' : ['YL'],
        'BN' : ['SZ']
    }

    # Jaune
    pointsToCheck1 = {
        'OX' : ['VB'],
        'TH' : ['PN'],
        'PN' : ['EY'],
        'BG' : ['VB'],
        'TS' : ['OX'],
        'EY' : ['FL'],
        'VB' : ["B3", "B2", "EY", "B2", "B3"],
        'FL' : ['PO'],
        'YT' : ['HA'],
        'HA' : ['BE'],
        'BE' : ['SZ'],
        'PL' : ['SZ'],
        'PO' : ['YL'],
        'SZ' : ['HV'],
        'SW' : ['BN'],
        'YL' : ['YL'],
        'HV' : ['HV'],
        'SF' : ['YL'],
        'BN' : ['BN']
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

    wayPointsToPillars0 = {
        "PN" : (-4.0, -2.05, False),
        "EY" : (-2.1,-1.14, False),
        "BE" : (0, 1.18, False),
        "PO" : (2.11,-1.15, False),
        "YL" : (4, -1.5, False),
        "BG" : (-3.85, 0, False),
        "OX" : (-6.3, 2.80, False),
        "TS" : (-2.95, 2.55, False),
        "VB" : (-2.2, 1.15, False),
        "HV" : (3.9, 0, False),
        "BN" : (6.4, 2.8, False),
        "SW" : (2.95, 2.55, False),
        "SZ" : (1.95, 1.1, False),
        "FL" : (0, -2.58, False),
        "YT" : (0, -1.9, False),
        "HA" : (0, 0.4, False),
        "PL" : (0, 2.6, False),
        "TH" : (-6.25, -3, False),
        "SF" : (6.25, -3, True),
        
        "A1" : (-3.75, 0.5, False),
        "A2" : (3.8, -0.75, False),
        "A2bis" : (3.85, -0.5, False),
        "A3" : (3.8, 0.5, False),

        "B1" : (3.75, 0.5, False),
        "B2" : (-3.80, -0.5, False),
        "B3" : (-3.80, 0.5, False),

        "C1" : (-5.4, 2.65, False),
        "C2" : (-3.65, -1.3, False),

        "T1" : (-7.2, -3.2, False),
        "T2" : (-5.2, -3.2, False),
        "T3" : (-3.65, -1.3, False),

        "U1" : (7.2, -3.35, False),
        "U2" : (5, -2.9, False),
        "U3" : (3.65, -1.3, False),
    }

    wayPointsToPillars1 = {
        "YL" : (4.0, -2.05, False),
        "PO" : (2.1,-1.14, False),
        "BE" : (0, 1.18, False),
        "EY" : (-2.11,-1.15, False),
        "PN" : (-4, -1.6, False),
        "HV" : (3.85, 0, False),
        "BN" : (6.3, 2.80, False),
        "SW" : (2.95, 2.55, False),
        "SZ" : (2.2, 1.15, False),
        "BG" : (-3.9, 0, False),
        "OX" : (-6.4, 2.8, False),
        "TS" : (-2.95, 2.55, False),
        "VB" : (-1.95, 1.1, False),
        "FL" : (0, -2.58, False),
        "YT" : (0, -1.9, False),
        "HA" : (0, 0.4, False),
        "PL" : (0, 2.6, False),
        "SF" : (6.25, -3, False),
        "TH" : (-6.25, -3, True),

        "A1" : (-3.75, 0.5, False),
        "A2" : (3.8, -0.75, False),
        "A2bis" : (3.85, -0.5, False),
        "A3" : (3.8, 0.5, False),

        "B1" : (3.75, 0.5, False),
        "B2" : (-3.80, -0.5, False),
        "B2bis" : (-3.80, -0.5, False),
        "B3" : (-3.80, 0.5, False),

        "C1" : (-5.4, 2.65, False),
        "C2" : (-3.65, -1.3, False),
        "C3" : (3.65, -1.3, False),

        "T1" : (-7.2, -3.35, False),
        "T2" : (-5, -2.9, False),
        "T3" : (-3.65, -1.3, False),

        "U1" : (7.2, -3.2, False),
        "U2" : (5.2, -3.2, False),
        "U3" : (3.65, -1.3, False),
    }

    wayPoints1 = ["U1", "U2", "YL", "PO", "FL", "EY", "B2", "B3", "VB", "BE", "HA", "SZ", "BE", "VB", "B3", "BG", "B3", "PN", "BG"]
    wayPoints0 = ["T1", "T2", "PN", "EY", "FL", "PO", "A2", "A3", "SZ", "BE", "HA", "VB", "BE", "SZ", "A3", "HV", "A2", "YL", "HV"]


    def __init__(self):
        Robot.__init__(self)
        self.startTime = self.time()
        self.leftMotor = self.motors[0].m0
        self.rightMotor = self.motors[0].m1
        self.opzone=abs(self.zone-1)
        if self.zone == 0 :
            self.x = -4.5
            self.wayPoints = self.wayPoints0
            self.wayPointsToPillars = self.wayPointsToPillars0
            self.pointsToCheck = self.pointsToCheck0
            #self.theta = self.toPiPi(3.926) #-pi/4
        else :
            self.x = 4.4
            self.wayPoints = self.wayPoints1
            self.wayPointsToPillars = self.wayPointsToPillars1
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
    
    def sign(self, v) :
        return v/abs(v)

    def camp(self) :
        if self.zone == 0 :
            for tx in self.transmitters :
                if tx.target_info.station_code.name == "SZ" and not tx.target_info.owned_by == self.zone :
                    self.wayPoints.extend(["A3", "SZ", "A3"])
                    return
                if tx.target_info.station_code.name == "YL" and not tx.target_info.owned_by == self.zone :
                    self.wayPoints.extend(["A2", "YL", "A2"])
                    return

        if self.zone == 1 :
            for tx in self.transmitters :
                if tx.target_info.station_code.name == "VB" and not tx.target_info.owned_by == self.zone :
                    self.wayPoints.extend(["B3", "VB", "B3"])
                    return
                if tx.target_info.station_code.name == "PN" and not tx.target_info.owned_by == self.zone :
                    self.wayPoints.extend(["B2", "PN", "B2"])
                    return
    
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
            self.sleep(0.5)
            transmitters = self.radio.sweep()
            self.transmitters = sorted(transmitters,key=lambda tx: tx.signal_strength, reverse = True)
            if self.outcome()=="fail":
                if self.wayPoints[0] == "VB" and self.zone == 0 :
                    self.wayPoints = ["B3", "B2", "PN", "EY", "FL", "PO", "A2", "A3", "SZ", "A3", "HV", "A2", "YL", "HV"]
                if self.wayPoints[0] == "SZ" and self.zone == 1 :
                    self.wayPoints = ["A2", "A2", "YL", "PO", "FL", "EY", "B2", "B3", "VB", "B3", "BG", "B2", "PN", "BG"]
                else :
                    self.addCheckpoint(self.transmitters[0].target_info.station_code.name)
            elif self.outcome()=="succes":
                logging.debug("Cible claimed : waypoint suivant") 
                self.wayPoints.pop(0)

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
                tx = self.transmitters[0]
                alpha = self.theta - tx.bearing
                d = sqrt(1/tx.signal_strength)
                self.x = self.pillars[tx.target_info.station_code][0] - d*cos(alpha)
                self.y = self.pillars[tx.target_info.station_code][1] + d*sin(alpha) 
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
        elif len(self.transmitters)==1:  
            logging.debug('Seulement un transmitter à portée')
            tx = self.transmitters[0]
            alpha = self.theta - tx.bearing
            d = sqrt(1/tx.signal_strength)
            self.x = self.pillars[tx.target_info.station_code][0] - d*cos(alpha)
            self.y = self.pillars[tx.target_info.station_code][1] + d*sin(alpha) 
            logging.debug(f"x = {self.x} et y = {self.y} et orientation en degre= {180*self.theta/pi}")
        else :
            logging.debug("Pas d'actualisation des coordonnees")            
        self.age = self.time()-self.actu

        print("wp : ", self.wayPoints)
        if len(self.wayPoints) != 0 :
            if sqrt((self.x-(self.wayPointsToPillars.get(self.wayPoints[0]))[0])**2+(self.y-(self.wayPointsToPillars.get(self.wayPoints[0]))[1])**2)<0.3 :
                if self.wayPoints[0] in self.pillars :
                    if self.transmitters[0].target_info.owned_by == self.zone :
                        self.wayPoints.pop(0)
                        logging.debug("Cible atteinte : waypoint suivant") 
                else :
                    self.wayPoints.pop(0)
                    logging.debug("Cible atteinte : waypoint suivant")
        
        while self.wayPoints==[]:
            self.setMotors(0,0)
            self.transmitters = self.radio.sweep()
            self.camp()

        # logging.debug("*******DEBUGGAGE*********")
        # logging.debug(f"self.wayPoints[0] : {self.wayPoints[0]}")
        # logging.debug(f"self.wayPointsToPillars.get(self.wayPoints[0]) : {self.wayPointsToPillars.get(self.wayPoints[0])}")
        # logging.debug(f"(self.wayPointsToPillars.get(self.wayPoints[0]))[1] : {(self.wayPointsToPillars.get(self.wayPoints[0]))[1]}")
        # logging.debug(f"self.y : {self.y}")
        # logging.debug(f"self.wayPoints[0] : {self.wayPoints[0]}")
        
        # logging.debug(f"(self.wayPointsToPillars.get(self.wayPoints[0]))[0] : {(self.wayPointsToPillars.get(self.wayPoints[0]))[0]}")
        # logging.debug(f"self.x : {self.x}")
        # logging.debug(f"\\\\\\\FIN DEBUGGAGE///////")      
        
        self.wayPointBearing = self.toPiPi(self.theta - (atan2(-((self.wayPointsToPillars.get(self.wayPoints[0]))[1]-self.y),(self.wayPointsToPillars.get(self.wayPoints[0]))[0]-self.x)))
        self.reverse = (self.wayPointsToPillars.get(self.wayPoints[0]))[2]

        logging.info(f" Info sur les cibles :")
        logging.info(f" Way Point Targetted : { self.wayPointsToPillars.get(self.wayPoints[0])[0] }")
        logging.info(f"Way Point Bearing : {self.wayPointBearing}")


    def isClaimable(self):
        if not self.transmitters == [] :
            if sqrt(1/self.transmitters[0].signal_strength)<0.5 and (not self.transmitters[0].target_info.owned_by == self.zone) and (self.transmitters[0].target_info.station_code.name==self.wayPoints[0]):
                return True
            return False
        return False

    def claim(self):
        self.setMotors(0,0)
        self.radio.claim_territory()


    def outcome(self):
        if self.transmitters[0].target_info.owned_by == self.zone or self.transmitters[0].target_info.station_code.name=='YT':
            return "succes"
        else :
            return "fail"

    def addCheckpoint(self, obj):
        self.wayPoints[0:0]=self.pointsToCheck[obj]

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
        logging.debug(f"Patinage : refPos = ({self.referencePositionX},{self.referencePositionY}) currPos = ({self.x},{self.y})")
        d = (self.referencePositionX - self.x)**2 + (self.referencePositionY - self.y)**2
        d = sqrt(d)
        logging.debug(f"Patinage : d = {d} tour num {self.timeSinceMouvement}")
        if self.timeSinceMouvement == 3:
            if d > seuil : 
                self.timeSinceMouvement = 0
                return(False)
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
        rotationSpeed =  60
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
        elif self.tsAR and not self.reverse:
            logging.debug(f"Coince a l'arriere : j'avance || tsAR = {self.tsAR} || reverse = {self.reverse}")
            if self.wayPointBearing > 0 :
                logging.debug("Nez vers la droite")
                right=100*cos(self.wayPointBearing) 
                left = -100                        
               
            else :
                logging.debug("Nez vers la gauche")
                right=100                           
                left=-100*cos(self.wayPointBearing)
        elif self.tsAR or (patine and self.reverse):
            logging.debug(f"Coince a l'arriere : j'avance || tsAR = {self.tsAR} || reverse = {self.reverse}")
            if self.wayPointBearing > 0 :
                logging.debug("Nez vers la droite")
                left=-100*cos(self.wayPointBearing) # on fait le contraire de l'effet recherché
                right = -100                        # car à la fin on inverse dans le cas reverse
               
            else :
                logging.debug("Nez vers la gauche")
                left=-100                           # idem !
                right=-100*cos(self.wayPointBearing)
               

        elif abs(self.wayPointBearing) > pi*0.8 :
            right = - rotationSpeed * self.sign(self.wayPointBearing)
            left = rotationSpeed * self.sign(self.wayPointBearing)

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
print("robot initialisé")

if R.zone==0:
    R.motors[0].m0.power = -100
    R.motors[0].m1.power = 100
    R.sleep(0.1)
    R.motors[0].m0.power = 100
    R.motors[0].m1.power = 100
    R.sleep(0.6)
else :
    print("je suis le jaune")
    R.motors[0].m0.power = 100
    R.motors[0].m1.power = -100
    R.sleep(0.1)
    R.motors[0].m0.power = 100
    R.motors[0].m1.power = 100
    R.sleep(0.6)
print("après if")
speed = 70
delay = 0.1
while True :
    R.update()
    R.goToTarget()
    R.sleep(delay)