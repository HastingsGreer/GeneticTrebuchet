import pygame
from pygame.locals import *
import numpy as np
import time
import calculatemotion
import scipy

def drawParticle(particle, surface, transform, thickness=4):
    r = 3 + np.log(particle.m)
    color = (0,255,0)
    pygame.draw.circle(surface, color, transform(particle.r) , int(r), 1)      
        
        
def drawRod(rod, surface, transform, thickness=4):
    color = (255,0,0)
    pygame.draw.line(surface, color, transform(rod.target1.r), 
                     transform(rod.target2.r), thickness)
            
def drawSlider(slider, surface, transform, color=(0,0,255), thickness=4):
    pygame.draw.line(surface, color, transform(slider.target.r+np.matrix([[0.0,40.0],[-40.0,0.0]])*slider.normal), transform(slider.target.r+np.matrix([[0.0,-40.0],[40.0,0.0]])*slider.normal), thickness)
        
        
        
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
            drawConstraintsAndParticles(x)
            pygame.display.update()
            pygame.image.save(self.DISPLAYSURF, "tmp/"+filename+str(n)+".jpg")
            
    def drawConstraintsAndParticles(self,x):
        self.system.fillFromStateVector(x)
        self.DISPLAYSURF.fill(self.WHITE)

        for constraint in self.system.constraintForces:
            if isinstance(constraint, calculatemotion.Rod):
                drawRod(constraint, self.DISPLAYSURF, self.transform, thickness=1)
            elif isinstance(constraint, calculatemotion.SliderOnBackground):
                drawSlider(constraint, self.DISPLAYSURF, self.transform, thickness=1)

        for particle in self.system.particleList:
            drawParticle(particle, self.DISPLAYSURF, self.transform, thickness=1)

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

        self.drawConstraintsAndParticles(y)
        pygame.display.update()
        return result

    def animate(self, pathpoints):
        for point in pathpoints:
            self.drawConstraintsAndParticles(point)
            pygame.display.update()
            time.sleep(.0051)
