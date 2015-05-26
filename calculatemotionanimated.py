import numpy as np
import scipy.integrate
import matplotlib
import sys
import pygame
from pygame.locals import *
import time

pygame.init()
def magnitude(vector):
    magnitudeSquared=(vector.getT())*(vector)
    return (magnitudeSquared.item(0))**(.5)



class Particle:
    def __init__(self, mass, position, velocity, color=(0,255,0)):
        self.r = position
        self.v = velocity
        self.m = mass
        self.f=0
        self.constraints=[]
        self.radius=3+np.log(mass)
        self.color=color
    def draw(self, surface, transform, thickness=4):
        pygame.draw.circle(surface, self.color, transform(self.r) , int(self.radius), 1)


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
    def affects(self, particle):
        pass
    def correctingAccelerationNeeded(self):
        pass
    def addOwnForce(self, strength):
        pass
    def calculateEffect(self, other):
        pass
    def draw(self, surface, transform, thickness=1):
        pass
    

class Rod(Constraint):
    def __init__(self, targetParticle1, targetParticle2, length):
        self.target1=targetParticle1
        self.target2=targetParticle2
        self.length=length
        self.target1.constraints.append(self)
        self.target2.constraints.append(self)

    def affects(self, particle):
        #returns a vector with units of 1/mass
        if particle == self.target1:
            return -self.direction()/self.target1.m
        elif particle == self.target2:

            return self.direction()/self.target2.m
        else:
            return np.matrix(np.zeros([2,1]))
        
            
    def currentValue(self):
        #returns a length vector
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
        #returns an acceleration magnitude
        


        centripetalAcceleration=(((magnitude((self.target1.v -self.target2.v).getT()*
                                             (np.matrix([[0.0,-1.0],[1.0,0.0]])*self.direction())))**2)/self.currentValue())
        

        netAccelerationOfParticles=(self.target1.f.getT()*self.direction()/self.target1.m-self.target2.f.getT()*
                                    self.direction()/self.target2.m)

        
        return centripetalAcceleration+netAccelerationOfParticles.item(0)
    
    def addOwnForce(self, strength):
        self.target1.f=self.target1.f-strength*self.direction()
        self.target2.f=self.target2.f+strength*self.direction()

    def calculateEffect(self, other):
        #print "calculated effect", (-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)
        return(-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)

    def draw(self, surface, transform, color=(255,0,0), thickness=4):
        pygame.draw.line(surface, color, transform(self.target1.r), transform(self.target2.r), thickness)
        

        
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
    def draw(self, surface, transform, color=(0,0,255), thickness=4):
        pygame.draw.line(surface, color, transform(self.target.r+np.matrix([[0.0,40.0],[-40.0,0.0]])*self.normal), transform(self.target.r+np.matrix([[0.0,-40.0],[40.0,0.0]])*self.normal), thickness)
            
                            
            
class ParticleSystem:
    def __init__(self):
        self.particleList=[]
        self.nonConstraintForces=[]
        self.constraintForces=[]
        self.numCalls=0
        
    #Setup methods


    def addParticle(self, mass, x, y, vx, vy, color=(0,255,0)):
        self.particleList.append(Particle(mass,
                                          np.matrix([[x],
                                                     [y]]),
                                          np.matrix([[vx],
                                                     [vy]]), color=color))
                         
        return len(self.particleList)-1




    def addGravity(self, n, forcevector):
        self.nonConstraintForces.append(Gravity(self.particleList[n], forcevector))

    def addSpring(self, n1, n2, k, restLength):
        self.nonConstraintForces.append(Spring(self.particleList[n1], self.particleList[n2], k, restLength))    


    def addRod(self, n1, n2, length):
        newrod=Rod(self.particleList[n1], self.particleList[n2], length)
        self.constraintForces.append(newrod)
        newrod.ind=len(self.constraintForces)        

    def addSlider(self, n, normalvector, distance):
        newslider=SliderOnBackground(self.particleList[n], normalvector, distance)
        self.constraintForces.append(newslider)
        newslider.ind=len(self.constraintForces)      

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



    def endCondition(self,y):
        return False

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
        
	    #print "magnitudes, ", magnitudes:


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

        if self.endCondition(y)==True:
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

        print "numcalls" ,self.numcalls
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
                 

class Animation():
    def __init__(self, system, surface):
        self.system=system
        self.DISPLAYSURF=surface
        self.RED=(255,0,0)
        self.WHITE=(255,255,255)

    def transform(self, vector):
        x=int((vector.item(0)*300.0/40.0)+150.0)
        y=int((vector.item(1)*(0-300.0)/40.0)+150.0)
        return (x,y)
        
    def makeVideo(self, solution, filename):
        for n, x in enumerate(solution):
            self.system.FillFromStateVector(x)
            self.DISPLAYSURF.fill(self.WHITE)

            for constraint in self.system.ConstraintForces:
                constraint.draw(self.DISPLAYSURF, self.transform, thickness=1)

            for particle in self.system.particleList:
                particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
            
            pygame.display.update()
            pygame.image.save(self.DISPLAYSURF, "tmp/"+filename+str(n)+".jpg")
              
    

    def simanimate(self, tfinal=3.0, steps=1000):
        self.y0=self.system.fillStateVector()
        self.time = np.linspace(0.0, tfinal, steps)
        self.solution=scipy.integrate.odeint(self.dydt, self.y0, self.time, rtol=1e-3, atol=1e-3, mxstep=40)
        print "numCalls:", self.system.numCalls
        
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
        
    def dydt(self, y, t):
        result = self.system.dydt(y,t)

        self.DISPLAYSURF.fill(self.WHITE)

        for constraint in self.system.constraintForces:
            constraint.draw(self.DISPLAYSURF, self.transform, thickness=1)

        for particle in self.system.particleList:
            particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
        
        pygame.display.update()
        return result

    def animate(self, pathpoints):
        for point in pathpoints:
            self.system.fillFromStateVector(point)
            self.DISPLAYSURF.fill(self.WHITE)

            for constraint in self.system.constraintForces:
                constraint.draw(self.DISPLAYSURF, self.transform, thickness=1)

            for particle in self.system.particleList:
                particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
            
            pygame.display.update()
            time.sleep(.0051)
            



def  TwoPendulums():
    MySystem=ParticleSystem()
    Particle0=MySystem.addParticle(1.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.addParticle(1.0, 1.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.addParticle(10000.0, 2.0, 0.0, 0.0, 0.0)
    Anchor=MySystem.addParticle(10000.0, -1.0, 0.0, 0.0, 0.0)

    MySystem.addGravity(Particle0, np.matrix([[0.0],
                                              [-9.8]]))

    MySystem.addGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))

    MySystem.addSpring(Particle0, Particle1, 1.0, 1.0)
    #MySystem.addSpring(Particle0, Anchor, 1500.0, 1.0)

    MySystem.addRod(Particle0, Anchor, 1)
    MySystem.addRod(Particle2, Particle1, 1)
    #debug:
    #y=np.matrix([[0],[0],[0],[0],[2],[0],[0],[0]])
    #MySystem.FillFromStateVector(y)
    #MySystem.CalculateNonConstraintForces()
    #for P in MySystem.particleList:
    #    print P.f
    y0=MySystem.FillStateVector()
    return MySystem


def SpinningRod():
    MySystem=ParticleSystem()
    Particle0=MySystem.addParticle(2.0, 0.0, 0.0, 0.0, 1.0)
    Particle1=MySystem.addParticle(3.0, 1.0, 0.0, 0.0, -1.0)
    MySystem.addRod(Particle0, Particle1, 1)
    
    return MySystem



def SpringAndRodAligned():
    MySystem=ParticleSystem()
    Particle0=MySystem.addParticle(2.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.addParticle(2.0, 1.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.addParticle(1.0, -1.0, 0.0, 0.0, 0.0)
    MySystem.addSpring(Particle0, Particle2, 5, 2)
    MySystem.addRod(Particle0, Particle1, 1)
    return MySystem


def SliderWithNoMotion():
    MySystem=ParticleSystem()
    Particle0=MySystem.addParticle(3.0, 0.0, 0.0, 0.0, 0.0)
    MySystem.addSlider(Particle0, np.matrix([[0.0],[1.0]]), 0)
    MySystem.addSlider(Particle0, np.matrix([[1.0], [0.0]]), 0)
    MySystem.addGravity(Particle0, np.matrix([[0.0],
                                              [-9.8]]))
    return MySystem 

def SpringAndFixedPoint():
    MySystem=ParticleSystem()
    Particle0=MySystem.addParticle(3.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.addParticle(2.0, 1.0, 0.0, 0.0, 0.0)
    MySystem.addSpring(Particle0, Particle1, 9.8, 1.0) 
    MySystem.addSlider(Particle0, np.matrix([[0.0],[1.0]]), 0)
    MySystem.addSlider(Particle0, np.matrix([[3.0/5.0], [4.0/5.0]]), 0)
    MySystem.addGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))
    return MySystem 


def DoublePendulum():
    MySystem=ParticleSystem()

    Anchor=MySystem.addParticle(1.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.addParticle(1.0, 2.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.addParticle(1.0, 3.0, 1.0, 0.0, 0.0)

    MySystem.addGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))
    MySystem.addGravity(Particle2, np.matrix([[0.0],
                                              [-9.8]]))

    
    #MySystem.addSlider(Anchor, np.matrix([[0.0],[1.0]]), 0)
    #MySystem.addSlider(Anchor, np.matrix([[1.0], [0.0]]), 0)
    MySystem.addPin(Anchor)
    MySystem.addRod(Anchor,Particle1, 2.0)
    MySystem.addRod(Particle1,Particle2, 1.0)
    MySystem.addRod(Anchor, Particle2, 1.0)
    
    
    return MySystem

def Trebuchet():
    MySystem=ParticleSystem()
    MainAxle=MySystem.addParticle(10, 0.0 , 0.0, 0.0, 0.0)
    CounterweightAxle=MySystem.addParticle(10, -1.0, 1.0, 0.0, 0.0)
    ArmTip=MySystem.addParticle(10, 4.0, -3.0, 0.0, 0.0)
    Counterweight=MySystem.addParticle(100.0, -1.0, -1.5, 0.0, 0.0)
    Projectile=MySystem.addParticle(1, 0.0, -3.0, 0.0, 0.0)
    
    MySystem.addGravity(MainAxle, np.matrix([[0.0],
                                             [-9.8]]))
    MySystem.addGravity(ArmTip, np.matrix([[0.0],
                                           [-9.8]]))
    MySystem.addGravity(Counterweight, np.matrix([[0.0],
                                                  [-9.8]]))
    MySystem.addGravity(CounterweightAxle, np.matrix([[0.0],
                                                      [-9.8]]))
    MySystem.addGravity(Projectile, np.matrix([[0.0],
                                               [-9.8]]))
    


    MySystem.addPin(MainAxle)
    MySystem.addRod(MainAxle, ArmTip, 1.0)
    MySystem.addRod(MainAxle, CounterweightAxle, 1.0)
    MySystem.addRod(ArmTip, CounterweightAxle, 1.0)
    MySystem.addRod(ArmTip, Projectile, 1.0)
    MySystem.addRod(CounterweightAxle, Counterweight, 1.0)

    return MySystem

if __name__ == "__main__":

    MySystem=Trebuchet()
    animation=Animation(MySystem, pygame.display.set_mode((300,400)))
    animation.simanimate()
    pygame.quit()
    sys.exit()


















