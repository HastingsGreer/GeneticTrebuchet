import numpy as np
from numpy import pi
import random
import calculatemotion

import types #this is always a good sign :D (this is used to redefine 

import sys
doAnimation = True
try:
    import animation
    import pygame
    from pygame.locals import *
    DISPLAYSURF=pygame.display.set_mode((300,400))
except:
    doAnimation = False


SLIDER = 0
RODTOBACKGROUND = 1
RODTOPARTICLE = 2
FIXTOBACKGROUND = 3


class Constraint:
    def addToSystem(self, n, particles, system):
        pass


class Slider(Constraint):
    def __init__(self, theta = "RANDOM"):
        if theta == "RANDOM":
            theta = random.random()*2*np.pi
        self.theta = theta
    def addToSystem(self, n, particles, system):
        normal=np.matrix([[np.sin(self.theta)],[np.cos(self.theta)]])                             #this is pretty much self evident, yes?
        system.addSlider(particles[n], normal, 0)
    def __repr__(self):
        return "Slider(" + repr(self.theta) + ")"
        
class Rod(Constraint):
    def __init__(self, disp = "RANDOM"):
        if disp == "RANDOM":
            disp = random.choice((2, 3))
        self.disp = disp
    def addToSystem(self, n, particles, system):
        if n + self.disp < len(particles) - 1:
            system.addRod(particles[n], particles[n+self.disp], 0)
    def __repr__(self):
        return "Rod(" + repr(self.disp) + ")"
class Pin(Constraint):
    def __init__(self):
        a = np.sqrt(.5)
        self.n1 = np.matrix([[a], [a]])
        self.n2 = np.matrix([[a], [-a]])
    def addToSystem(self, n, particles, system):
        system.addSlider(particles[n], self.n1, 0)
        system.addSlider(particles[n], self.n2, 0)
    def __repr__(self):
        return "Pin()"
def newConstraint():
    return random.choice((Rod(), Rod(), Pin(), Slider()))
        
class TrebLink:
    def __init__(self, angle, length, constraints):
        self.angle = angle
        self.length = length
        self.constraints = constraints
        
    def __repr__(self):
        return "TrebLink" + repr((self.angle, self.length, self.constraints))
    
    def addConstraints(self, n, particles, system):
        for constraint in self.constraints:
            constraint.addToSystem(n, particles, system)
    
    
    
    def mutate(self):
        a = random.choice((1, 2, 3, 4))
        if a == 1:
            length = self.length + random.uniform(-2, 2)
            if 10< length < 1:
                length = 10*random.random()
            return TrebLink(self.angle, length, self.constraints)
        elif a == 2:
            angle = self.angle + (.5-random.random())
            return TrebLink(angle, self.length, self.constraints)
        elif a == 3:
            return TrebLink(self.angle, self.length, self.constraints and random.sample(self.constraints, len(self.constraints)-1))
        elif a == 4:
            return TrebLink(self.angle, self.length, self.constraints + [newConstraint()])
            
         
                
               

trebuchetarchive=dict()


class LinkTrebuchet:
    def __init__(self, tLinkList):    
        self.tLinkList = tLinkList
         
    
    def assembleFromString(self, string):
        pass
    def __repr__(self):
        return "LinkTrebuchet(" + repr(self.tLinkList) + ")"
        
    def mutate(self):
        newtll = self.tLinkList[:]
        n = random.randint(0,len(newtll)-1)
        newtll[n] = newtll[n].mutate()
        return LinkTrebuchet(newtll)
        
    def crossover(self, other):
        n = random.randint(0, len(self.tLinkList)-1)
        m = random.randint(0, len(other.tLinkList)-1)
        return [
                LinkTrebuchet((self.tLinkList[:n] + other.tLinkList[m:])[:10]),
                LinkTrebuchet((other.tLinkList[m:] + self.tLinkList[:n])[:10]),
                ]
    def evaluate(self, savesystem = False, surface = None):
        
        
        if (str(self) in trebuchetarchive) and not savesystem:              #checks if has been tried before, uses previous result if so.
            print "efficiency", trebuchetarchive[str(self)]
            return trebuchetarchive[str(self)]
        
        
        system=calculatemotion.ParticleSystem()
        position = np.array([0.0, 0.0])
        angle = 0.0
        particles = []
        for link in self.tLinkList:
            particles = particles + [system.addParticle(1000 if (len(particles)==0) else 10, position[0], position[1], 0, 0, color = (255, 0, 0))]
            angle = angle + link.angle
            position = position + link.length*np.array([np.cos(angle),np.sin(angle)])
                                                               
        particles = particles + [system.addParticle(1, position[0], position[1], 0, 0, color = (255, 255, 0))]
        
        for particle in particles:
            system.addGravity(particle, np.matrix([[0.0],
                                              [-9.8]]))
        
        
        for pair in zip(particles[1:], particles[:-1]):
            sling = system.addRod(pair[0], pair[1], 42)    #I want sling to be the last of this set of rods
        
        
        
        
        
        for link , n in zip(self.tLinkList, range(len(self.tLinkList))):
            link.addConstraints(n, particles, system)
        
        def endCondition(sys, y):
            return (sys.constraintForces[sling]).strength < -5
        system.endCondition = endCondition
        
        if doAnimation:
            myAnimation = animation.Animation(system, DISPLAYSURF)
            myAnimation.simanimate()
        else:
            system.simulate()
            myAnimation = system
        miny=max(np.array(myAnimation.ys)[0])-min((np.array(myAnimation.ys)).flatten())                            #total height


        vx=[np.abs(step[-2])*step[-1] for step in myAnimation.solution]                #ranges
        
        
        #Suppress individuals that spin around too much:
        
        angles = np.unwrap(np.arctan2(myAnimation.solution[:, -4] - myAnimation.solution[:, -8],
                                      myAnimation.solution[:, -3] - myAnimation.solution[:, -7]))
        for n in xrange(len(myAnimation.solution)):
            if abs(angles[n]) > 3 * np.pi:
                for k in xrange(n, len(myAnimation.solution)):        
                    vx[k] = 0 
    

        vxa=np.array(vx)

        maxxy=np.max(vxa.flatten())
    
        print miny
        print maxxy
        print "efficiency:", maxxy/(9.8*1000*miny)
    
        if savesystem==True:
            return myAnimation
        trebuchetarchive[str(self)]=maxxy/(9.8*1000*miny)
        
    
        return maxxy/(9.8*1000*miny)
        
if __name__ == "__main__":
    firstLink = TrebLink(pi/2, 3, [])
    link2 = TrebLink(3*pi/4, 1.5, [Rod(2)])
    link3 = TrebLink(-pi/40, 6, [Slider(), Slider()])
    link4 = TrebLink(3*pi/4, 8, [])
    lt = LinkTrebuchet([firstLink, link2, link3, link4])
    lt2 = lt
    for n in range(1):
        lt2=lt2.mutate()
    print lt2
    lt2.evaluate()
    
        
                                                               
            
        
        
        

    
