import numpy as np
import scipy.integrate
import matplotlib
import sys
import pygame
from pygame.locals import *
import time

pygame.init()
def magnitude(vector):
    MagnitudeSquared=(vector.getT())*(vector)
    return (MagnitudeSquared.item(0))**(.5)



class Particle:
    def __init__(self, mass, position, velocity, color=(0,255,0)):
        self.r = position
        self.v = velocity
        self.m = mass
        self.f=0
        self.Constraints=[]
        self.radius=3+np.log(mass)
        self.color=color
    def draw(self, surface, transform, thickness=4):
        pygame.draw.circle(surface, self.color, transform(self.r) , int(self.radius), 1)


class Force:
    def AddOwnForce(self):
        print "you forgot to redefine AddOwnForce"


class Gravity(Force):
    def __init__(self, targetparticle, forcevector):
        self.target=targetparticle
        self.gravityvector=forcevector

    def AddOwnForce(self):
        self.target.f=self.target.f+self.gravityvector*self.target.m


    


class Spring(Force):
    def __init__(self, targetparticle1, targetparticle2, k, restlength):
        self.target1=targetparticle1
        self.target2=targetparticle2
        self.k=k
        self.l=restlength

    def AddOwnForce(self):
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
    def correctingaccelerationneeded(self):
        pass
    def AddOwnForce(self, strength):
        pass
    def calculateeffect(self, other):
        pass
    def draw(self, surface, transform, thickness=1):
        pass
    

class Rod(Constraint):
    def __init__(self, targetparticle1, targetparticle2, length):
        self.target1=targetparticle1
        self.target2=targetparticle2
        self.length=length
        self.target1.Constraints.append(self)
        self.target2.Constraints.append(self)

    def affects(self, particle):
        #returns a value in units of 1/mass
        if particle == self.target1:
            return -self.direction()/self.target1.m
        elif particle == self.target2:

            return self.direction()/self.target2.m
        else:
            return np.matrix(np.zeros([2,1]))
        
            
    def currentvalue(self):
        #returns a length vector
        #print "current value", magnitude(self.target1.r-self.target2.r)
        return magnitude(self.target1.r-self.target2.r)

    def dydtvalue(self):
        return magnitude((self.target1.v-self.target2.v).getT()*((self.target1.r-self.target2.r)/self.currentvalue))

    def direction(self):
        #returns a unit vector
        #print (self.target1.r-self.target2.r)/self.currentvalue()
        return (self.target1.r-self.target2.r)/self.currentvalue()

    def angle(self):
        dir=self.direction()
        x=dir.item(0)
        y=dir.item(1)
        return np.arctan2(x,y)
    
    def correctingaccelerationneeded(self):
        #returns an acceleration magnitude
        


        centripetalacceleration=(((magnitude((self.target1.v -self.target2.v).getT()*
                                             (np.matrix([[0.0,-1.0],[1.0,0.0]])*self.direction())))**2)/self.currentvalue())
        

        netaccelerationofparticles=(self.target1.f.getT()*self.direction()/self.target1.m-self.target2.f.getT()*
                                    self.direction()/self.target2.m)

        #print "centripetl acc", centripetalacceleration
        #print "net acc particles", netaccelerationofparticles.item(0)
        
        return centripetalacceleration+netaccelerationofparticles.item(0)
    
    def AddOwnForce(self, strength):
        self.target1.f=self.target1.f-strength*self.direction()
        self.target2.f=self.target2.f+strength*self.direction()

    def calculateeffect(self, other):
        #print "calculated effect", (-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)
        return(-self.direction().getT())*other.affects(self.target1)+(self.direction().getT())*other.affects(self.target2)

    def draw(self, surface, transform, color=(255,0,0), thickness=4):
        pygame.draw.line(surface, color, transform(self.target1.r), transform(self.target2.r), thickness)
        

        
class SliderOnBackground(Constraint):
    def __init__(self, targetparticle, normalvector, distance):
        self.target=targetparticle
        self.normal=normalvector
        self.distance=distance
        self.target.Constraints.append(self)
       
    def affects(self,particle):
        if particle == self.target:
            return self.normal/self.target.m
        else:
            return np.matrix([[0.0],[0.0]])
    def correctingaccelerationneeded(self):
        return -(self.normal.getT()*self.target.f).item(0)/self.target.m

    def AddOwnForce(self, strength):
        #print "strength" ,strength
        self.target.f = self.target.f + strength*self.normal

    def calculateeffect(self, other):
        #print "calculatedeffect",(self.normal.getT()*other.affects(self.target)).item(0)
        return (self.normal.getT()*other.affects(self.target)).item(0)
    def draw(self, surface, transform, color=(0,0,255), thickness=4):
        pygame.draw.line(surface, color, transform(self.target.r+np.matrix([[0.0,40.0],[-40.0,0.0]])*self.normal), transform(self.target.r+np.matrix([[0.0,-40.0],[40.0,0.0]])*self.normal), thickness)
            
                            
            
class ParticleSystem:
    def __init__(self):
        self.particlelist=[]
        self.NonConstraintForces=[]
        self.ConstraintForces=[]
        self.numcalls=0
        
    #Setup methods


    def AddParticle(self, mass, x, y, vx, vy, color=(0,255,0)):
        self.particlelist.append(Particle(mass,
                                          np.matrix([[x],
                                                     [y]]),
                                          np.matrix([[vx],
                                                     [vy]]), color=color))
                         
        return len(self.particlelist)-1




    def AddGravity(self, n, forcevector):
        self.NonConstraintForces.append(Gravity(self.particlelist[n], forcevector))

    def AddSpring(self, n1, n2, k, restlength):
        self.NonConstraintForces.append(Spring(self.particlelist[n1], self.particlelist[n2], k, restlength))    


    def AddRod(self, n1, n2, length):
        newrod=Rod(self.particlelist[n1], self.particlelist[n2], length)
        self.ConstraintForces.append(newrod)
        newrod.ind=len(self.ConstraintForces)        

    def AddSlider(self, n, normalvector, distance):
        newslider=SliderOnBackground(self.particlelist[n], normalvector, distance)
        self.ConstraintForces.append(newslider)
        newslider.ind=len(self.ConstraintForces)      

    def AddPin(self, n):
        self.AddSlider(n, np.matrix([[1],[0]]), 0)
        self.AddSlider(n, np.matrix([[0],[1]]), 0)
        

    def FillStateVector(self):
        y=[]
        for P in self.particlelist:
            y.append(P.r.item(0))
            y.append(P.r.item(1))
            y.append(P.v.item(0))
            y.append(P.v.item(1))
        return np.array(y)
    



    #Methods used by integration



    def endcondition(self,y):
        return False

    def FillFromStateVector(self, y):
        i=0
        for P in self.particlelist:

            P.r=np.matrix([[y.item(i)  ],
                           [y.item(i+1)]])

            P.v=np.matrix([[y.item(i+2)],
                           [y.item(i+3)]])

            P.f=np.matrix( [[0.0],
                            [0.0]])
            i=i+4
    
    
            
    
    def CalculateNonConstraintForces(self):
        for EachForce in self.NonConstraintForces:
            EachForce.AddOwnForce()
    
    def CalculateConstraintForces(self, damn_update=True):
        Correctingaccelerationsneeded=[]
        for C in self.ConstraintForces:
            Correctingaccelerationsneeded.append(C.correctingaccelerationneeded())

        D=np.matrix(Correctingaccelerationsneeded).getT()
        
        numconstraints=len(self.ConstraintForces)
        A=np.matrix(np.zeros([numconstraints, numconstraints]))
        

        for i in range(numconstraints):
            for j in range(numconstraints):
                A[i,j]=self.ConstraintForces[j].calculateeffect(self.ConstraintForces[i])
        #print "D",D
        
        #print "A",A
        
        try:
            Magnitudes = (A**(-1))*D
        except:
            self.numcalls=100000000009
        
	    #print "Magnitudes, ", Magnitudes:


        if damn_update:
            for i in range(numconstraints):
                self.ConstraintForces[i].AddOwnForce(Magnitudes.item(i))

    def checkLegality(self):
        Correctingaccelerationsneeded=[]
        for C in self.ConstraintForces:
            Correctingaccelerationsneeded.append(C.correctingaccelerationneeded())

        D=np.matrix(Correctingaccelerationsneeded).getT()
        
        numconstraints=len(self.ConstraintForces)
        A=np.matrix(np.zeros([numconstraints, numconstraints]))

        for i in range(numconstraints):
            for j in range(numconstraints):
                A[i,j]=self.ConstraintForces[j].calculateeffect(self.ConstraintForces[i])
        
        Magnitudes = (A**(-1))*D
        
                
    
               

    def dydt(self, y, t):
        self.numcalls=self.numcalls+1
        if self.numcalls>=100000:
            return np.zeros(len(y))

        if self.endcondition(y)==True:
            return np.zeros(len(y))
        
	
        self.FillFromStateVector(y)
        self.CalculateNonConstraintForces()
        self.CalculateConstraintForces()

        derivatives=[]

        for P in self.particlelist:
            derivatives.append(P.v.item(0))
            derivatives.append(P.v.item(1))
            derivatives.append(P.f.item(0)/P.m)
            derivatives.append(P.f.item(1)/P.m)
            
        return np.array(derivatives)
        
    def Simulate(self, tfinal=3.0, steps=1000):
        self.y0=MySystem.FillStateVector()
        self.time = np.linspace(0.0, tfinal, steps)
        self.solution=scipy.integrate.odeint(self.dydt, self.y0, self.time)

	print "numcalls" ,self.numcalls
        self.xs=[]
        self.ys=[]
        for point in MySystem.solution:
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

            for particle in self.system.particlelist:
                particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
            
            pygame.display.update()
            pygame.image.save(self.DISPLAYSURF, "tmp/"+filename+str(n)+".jpg")
              
    

    def simanimate(self, tfinal=3.0, steps=1000):
        self.y0=self.system.FillStateVector()
        self.time = np.linspace(0.0, tfinal, steps)
        self.solution=scipy.integrate.odeint(self.dydt, self.y0, self.time, rtol=1e-3, atol=1e-3, mxstep=40)
        print "numcalls:", self.system.numcalls
        
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

        for constraint in self.system.ConstraintForces:
            constraint.draw(self.DISPLAYSURF, self.transform, thickness=1)

        for particle in self.system.particlelist:
            particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
        
        pygame.display.update()
        return result

    def animate(self, pathpoints):
        for point in pathpoints:
            self.system.FillFromStateVector(point)
            self.DISPLAYSURF.fill(self.WHITE)

            for constraint in self.system.ConstraintForces:
                constraint.draw(self.DISPLAYSURF, self.transform, thickness=1)

            for particle in self.system.particlelist:
                particle.draw(self.DISPLAYSURF, self.transform, thickness=1)
            
            pygame.display.update()
            time.sleep(.0051)
            



def  TwoPendulums():
    MySystem=ParticleSystem()
    Particle0=MySystem.AddParticle(1.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.AddParticle(1.0, 1.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.AddParticle(10000.0, 2.0, 0.0, 0.0, 0.0)
    Anchor=MySystem.AddParticle(10000.0, -1.0, 0.0, 0.0, 0.0)

    MySystem.AddGravity(Particle0, np.matrix([[0.0],
                                              [-9.8]]))

    MySystem.AddGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))

    MySystem.AddSpring(Particle0, Particle1, 1.0, 1.0)
    #MySystem.AddSpring(Particle0, Anchor, 1500.0, 1.0)

    MySystem.AddRod(Particle0, Anchor, 1)
    MySystem.AddRod(Particle2, Particle1, 1)
    #debug:
    #y=np.matrix([[0],[0],[0],[0],[2],[0],[0],[0]])
    #MySystem.FillFromStateVector(y)
    #MySystem.CalculateNonConstraintForces()
    #for P in MySystem.particlelist:
    #    print P.f
    y0=MySystem.FillStateVector()
    return MySystem


def SpinningRod():
    MySystem=ParticleSystem()
    Particle0=MySystem.AddParticle(2.0, 0.0, 0.0, 0.0, 1.0)
    Particle1=MySystem.AddParticle(3.0, 1.0, 0.0, 0.0, -1.0)
    MySystem.AddRod(Particle0, Particle1, 1)
    
    return MySystem



def SpringAndRodAligned():
    MySystem=ParticleSystem()
    Particle0=MySystem.AddParticle(2.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.AddParticle(2.0, 1.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.AddParticle(1.0, -1.0, 0.0, 0.0, 0.0)
    MySystem.AddSpring(Particle0, Particle2, 5, 2)
    MySystem.AddRod(Particle0, Particle1, 1)
    return MySystem


def SliderWithNoMotion():
    MySystem=ParticleSystem()
    Particle0=MySystem.AddParticle(3.0, 0.0, 0.0, 0.0, 0.0)
    MySystem.AddSlider(Particle0, np.matrix([[0.0],[1.0]]), 0)
    MySystem.AddSlider(Particle0, np.matrix([[1.0], [0.0]]), 0)
    MySystem.AddGravity(Particle0, np.matrix([[0.0],
                                              [-9.8]]))
    return MySystem 

def SpringAndFixedPoint():
    MySystem=ParticleSystem()
    Particle0=MySystem.AddParticle(3.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.AddParticle(2.0, 1.0, 0.0, 0.0, 0.0)
    MySystem.AddSpring(Particle0, Particle1, 9.8, 1.0) 
    MySystem.AddSlider(Particle0, np.matrix([[0.0],[1.0]]), 0)
    MySystem.AddSlider(Particle0, np.matrix([[3.0/5.0], [4.0/5.0]]), 0)
    MySystem.AddGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))
    return MySystem 


def DoublePendulum():
    MySystem=ParticleSystem()

    Anchor=MySystem.AddParticle(1.0, 0.0, 0.0, 0.0, 0.0)
    Particle1=MySystem.AddParticle(1.0, 2.0, 0.0, 0.0, 0.0)
    Particle2=MySystem.AddParticle(1.0, 3.0, 0.0, 0.0, 0.0)

    MySystem.AddGravity(Particle1, np.matrix([[0.0],
                                              [-9.8]]))
    MySystem.AddGravity(Particle2, np.matrix([[0.0],
                                              [-9.8]]))

    
    MySystem.AddSlider(Anchor, np.matrix([[0.0],[1.0]]), 0)
    MySystem.AddSlider(Anchor, np.matrix([[1.0], [0.0]]), 0)
    MySystem.AddRod(Anchor,Particle1, 2.0)
    MySystem.AddRod(Particle1,Particle2, 1.0)
    
    
    return MySystem

def Trebuchet():
    MySystem=ParticleSystem()
    MainAxle=MySystem.AddParticle(10, 0.0 , 0.0, 0.0, 0.0)
    CounterweightAxle=MySystem.AddParticle(10, -1.0, 1.0, 0.0, 0.0)
    ArmTip=MySystem.AddParticle(10, 4.0, -3.0, 0.0, 0.0)
    Counterweight=MySystem.AddParticle(100.0, -1.0, -1.5, 0.0, 0.0)
    Projectile=MySystem.AddParticle(1, 0.0, -3.0, 0.0, 0.0)
    
    MySystem.AddGravity(MainAxle, np.matrix([[0.0],
                                             [-9.8]]))
    MySystem.AddGravity(ArmTip, np.matrix([[0.0],
                                           [-9.8]]))
    MySystem.AddGravity(Counterweight, np.matrix([[0.0],
                                                  [-9.8]]))
    MySystem.AddGravity(CounterweightAxle, np.matrix([[0.0],
                                                      [-9.8]]))
    MySystem.AddGravity(Projectile, np.matrix([[0.0],
                                               [-9.8]]))
    


    MySystem.AddPin(MainAxle)
    MySystem.AddRod(MainAxle, ArmTip, 1.0)
    MySystem.AddRod(MainAxle, CounterweightAxle, 1.0)
    MySystem.AddRod(ArmTip, CounterweightAxle, 1.0)
    MySystem.AddRod(ArmTip, Projectile, 1.0)
    MySystem.AddRod(CounterweightAxle, Counterweight, 1.0)

    return MySystem

if __name__ == "__main__":

    MySystem=Trebuchet()
    animation=Animation(MySystem, pygame.display.set_mode((300,400)))
    animation.simanimate()
    pygame.quit()
    sys.exit()


















