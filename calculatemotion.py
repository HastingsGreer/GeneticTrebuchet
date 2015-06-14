import numpy as np
import scipy.integrate
import matplotlib
import sys
import time

def magnitude(vector):
    magnitudeSquared=(vector.getT())*(vector)
    return (magnitudeSquared.item(0))**(.5)

class Particle:
    def __init__(self, mass, position, velocity):
        self.r = position
        self.v = velocity
        self.m = mass
        self.f=0
        self.constraints=[]

class Force:
    def addOwnForce(self):
        print "you forgot to redefine addOwnForce"

class Gravity(Force):
    def __init__(self, targetParticle, forceVector):
        self.target=targetParticle
        self.gravityVector=forceVector

    def addOwnForce(self):
        self.target.f=self.target.f+self.gravityVector*self.target.m

class Spring(Force):
    def __init__(self, targetParticle1, targetParticle2, k, restLength):
        self.target1=targetParticle1
        self.target2=targetParticle2
        self.k=k
        self.l=restLength

    def addOwnForce(self):
        displacement=self.target1.r-self.target2.r
        force=self.k*(magnitude(displacement)-self.l)*displacement/magnitude(displacement)
        #print("spring force", force)
        self.target1.f=self.target1.f-force
        self.target2.f=self.target2.f+force

class SprungSlot(Force):
    def __init__(self, target, k,):
        pass

class Constraint:

    #returns a vector with units of 1/mass
    def affects(self, particle):
        pass
    
    #returns an acceleration magnitude
    def correctingAccelerationNeeded(self):
        pass
    
    def addOwnForce(self, strength):
        pass
    
    def calculateEffect(self, other):
        pass

class Rod(Constraint):
    def __init__(self, targetParticle1, targetParticle2, length):
        self.target1=targetParticle1
        self.target2=targetParticle2
        self.length=length
        self.target1.constraints.append(self)
        self.target2.constraints.append(self)
        self.strength = 0

    def affects(self, particle):
        if particle == self.target1:
            return -self.direction()/self.target1.m
        elif particle == self.target2:

            return self.direction()/self.target2.m
        else:
            return np.matrix(np.zeros([2,1]))    
            
    def currentValue(self):
        #print "current value", magnitude(self.target1.r-self.target2.r)
        return magnitude(self.target1.r-self.target2.r)

    def dydtValue(self):
        return magnitude((self.target1.v-self.target2.v).getT()*((self.target1.r-self.target2.r)/self.currentValue))

    def direction(self):
        #returns a unit vector
        #print (self.target1.r-self.target2.r)/self.currentvalue()
        return (self.target1.r-self.target2.r)/self.currentValue()

    def angle(self):
        dir=self.direction()
        x=dir.item(0)
        y=dir.item(1)
        return np.arctan2(x,y)
    
    def correctingAccelerationNeeded(self):
        centripetalAcceleration=(((magnitude((self.target1.v -self.target2.v).getT()*
                                             (np.matrix([[0.0,-1.0],[1.0,0.0]])*self.direction())))**2)/self.currentValue())
        

        netAccelerationOfParticles=(self.target1.f.getT()*self.direction()/self.target1.m-self.target2.f.getT()*self.direction()/self.target2.m)
                                 
        #returns a length vector   self.direction()/self.target2.m)

        
        return centripetalAcceleration+netAccelerationOfParticles.item(0)
    
    def addOwnForce(self, strength):
        self.strength = strength
        self.target1.f=self.target1.f-strength*self.direction()
        self.target2.f=self.target2.f+strength*self.direction()

    def calculateEffect(self, other):
        #print "calculated effect", (-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)
        return(-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)

        
class SliderOnBackground(Constraint):
    def __init__(self, targetParticle, normalVector, distance):
        self.target=targetParticle
        self.normal=normalVector
        self.distance=distance
        self.target.constraints.append(self)
       
    def affects(self,particle):
        if particle == self.target:
            return self.normal/self.target.m
        else:
            return np.matrix([[0.0],[0.0]])
    def correctingAccelerationNeeded(self):
        return -(self.normal.getT()*self.target.f).item(0)/self.target.m

    def addOwnForce(self, strength):
        #print "strength" ,strength
        self.target.f = self.target.f + strength*self.normal

    def calculateEffect(self, other):
        #print "calculatedeffect",(self.normal.getT()*other.affects(self.target)).item(0)
        return (self.normal.getT()*other.affects(self.target)).item(0)
    
            
                            
            
class ParticleSystem:
    def __init__(self):
        self.particleList=[]
        self.nonConstraintForces=[]
        self.constraintForces=[]
        self.numCalls=0
        
        self.endCondition = lambda system, y: False
        
    #Setup methods


    def addParticle(self, mass, x, y, vx, vy, color=(0,255,0)):
        self.particleList.append(Particle(mass,
                                          np.matrix([[x],
                                                     [y]]),
                                          np.matrix([[vx],
                                                     [vy]])))
                         
        return len(self.particleList)-1

    def addGravity(self, n, forcevector):
        self.nonConstraintForces.append(Gravity(self.particleList[n], forcevector))

    def addSpring(self, n1, n2, k, restLength):
        self.nonConstraintForces.append(Spring(self.particleList[n1], self.particleList[n2], k, restLength))    

    def addRod(self, n1, n2, length):
        newrod=Rod(self.particleList[n1], self.particleList[n2], length)
        self.constraintForces.append(newrod)
        newrod.ind=len(self.constraintForces) - 1
        return newrod.ind        

    def addSlider(self, n, normalvector, distance):
        newslider=SliderOnBackground(self.particleList[n], normalvector, distance)
        self.constraintForces.append(newslider)
        newslider.ind=len(self.constraintForces ) - 1     

    def addPin(self, n):
        self.addSlider(n, np.matrix([[1.],[0.]]), 0)
        self.addSlider(n, np.matrix([[0.],[1]]), 0)
      
    def fillStateVector(self):
        y=[]
        for P in self.particleList:
            y.append(P.r.item(0))
            y.append(P.r.item(1))
            y.append(P.v.item(0))
            y.append(P.v.item(1))
        return np.array(y)
   
    #Methods used by integration

    
    
    def fillFromStateVector(self, y):
        i = 0
        for P in self.particleList:

            P.r=np.matrix([[y.item(i)  ],
                           [y.item(i+1)]])

            P.v=np.matrix([[y.item(i+2)],
                           [y.item(i+3)]])

            P.f=np.matrix( [[0.0],
                            [0.0]])
            i=i+4

    def calculateNonConstraintForces(self):
        for eachForce in self.nonConstraintForces:
            eachForce.addOwnForce()
    
    def calculateConstraintForces(self, damn_update=True):
        correctingAccelerationsNeeded=[]
        for c in self.constraintForces:
            correctingAccelerationsNeeded.append(c.correctingAccelerationNeeded())

        D=np.matrix(correctingAccelerationsNeeded).getT()
        
        numConstraints=len(self.constraintForces)
        A=np.matrix(np.zeros([numConstraints, numConstraints]))
        

        for i in range(numConstraints):
            for j in range(numConstraints):
                A[i,j]=self.constraintForces[j].calculateEffect(self.constraintForces[i])      
        try:
            magnitudes = (A**(-1))*D
        except:
            self.numCalls=1000000000

        if damn_update:
            for i in range(numConstraints):
                self.constraintForces[i].addOwnForce(magnitudes.item(i))

    def checkLegality(self):
        correctingAccelerationsNeeded=[]
        for c in self.constraintForces:
            correctingAccelerationsNeeded.append(c.correctingAccelerationNeeded())

        D=np.matrix(correctingAccelerationsNeeded).getT()
        
        numConstraints=len(self.constraintForces)
        A=np.matrix(np.zeros([numConstraints, numConstraints]))
        

        for i in range(numConstraints):
            for j in range(numConstraints):
                A[i,j]=self.constraintForces[j].calculateEffect(self.constraintForces[i])
        
        magnitudes = (A**(-1))*D

    def dydt(self, y, t):
        self.numCalls = self.numCalls + 1
        if self.numCalls >= 100000:
            return np.zeros(len(y))

        if self.endCondition(self, y)==True:
            return np.zeros(len(y))
        
        self.fillFromStateVector(y)
        self.calculateNonConstraintForces()
        self.calculateConstraintForces()

        derivatives=[]

        for P in self.particleList:
            derivatives.append(P.v.item(0))
            derivatives.append(P.v.item(1))
            derivatives.append(P.f.item(0)/P.m)
            derivatives.append(P.f.item(1)/P.m)
            
        return np.array(derivatives)
        
    def simulate(self, tfinal=3.0, steps=1000):
        self.y0=self.fillStateVector()
        self.time = np.linspace(0.0, tfinal, steps)
        self.solution=scipy.integrate.odeint(self.dydt, self.y0, self.time)

        print "numcalls" ,self.numCalls
        self.xs=[]
        self.ys=[]
        for point in self.solution:
            pointxs=[]
            pointys=[]
            for j in range(len(point)/4):
                pointxs.append(point[4*j])
                pointys.append(point[4*j+1])
            
            self.xs.append(pointxs)
            self.ys.append(pointys)



if __name__ == "__main__":

    MySystem=Trebuchet()
    animation=Animation(MySystem, pygame.display.set_mode((300,400)))
    animation.simanimate()
    pygame.quit()
    sys.exit()


















