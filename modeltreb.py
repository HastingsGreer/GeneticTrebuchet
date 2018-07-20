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
except ImportError:
    doAnimation = False


def Trebuchet2():
    MySystem=calculatemotion.ParticleSystem()
    MainAxle=MySystem.addParticle(10, 0.0 , 0.0, 0.0, 0.0)
    CounterweightAxle=MySystem.addParticle(10, -.50, 1.50, 0.0, 0.0)
    ArmTip=MySystem.addParticle(5, 0.0, -5.0, 0.0, 0.0)
    Counterweight1= MySystem.addParticle(200.0, -3.5, 3, 0.0, 0.0)
    Counterweight1Pin = MySystem.addParticle(10.0, -5, -1.5, 0.0, 0.0)
    Counterweight2= MySystem.addParticle(200.0, 3.5, 3, 0.0, 0.0)
    Counterweight2Pin = MySystem.addParticle(10.0, 5, -1.5, 0.0, 0.0)

    Slappa1 = MySystem.addParticle(40.0, 0, 5, 0.0, 0.0)
    
    Slappa2 = MySystem.addParticle(20.0, -1.5, 2.7, 0.0, 0.0)


    Projectile=MySystem.addParticle(1, -5.5, -5, 0.0, 0.0)
    
    MySystem.addGravity(MainAxle, np.matrix([[0.0],
                                             [-9.8]]))
    MySystem.addGravity(ArmTip, np.matrix([[0.0],
                                           [-9.8]]))
    MySystem.addGravity(Counterweight1, np.matrix([[0.0],
                                                  [-9.8]]))
    MySystem.addGravity(Counterweight2, np.matrix([[0.0],
                                                  [-9.8]]))
    MySystem.addGravity(CounterweightAxle, np.matrix([[0.0],
                                                      [-9.8]]))
    MySystem.addGravity(Projectile, np.matrix([[0.0],
                                               [-9.8]]))
    MySystem.addGravity(Slappa1, np.matrix([[0.0],
                                               [-9.8]]))
    MySystem.addGravity(Slappa2, np.matrix([[0.0],
                                               [-9.8]]))
    


    MySystem.addPin(MainAxle)
    MySystem.addPin(Counterweight1Pin)
    MySystem.addPin(Counterweight2Pin)
    MySystem.addRod(Counterweight1, Counterweight1Pin, 1.0)
    MySystem.addRod(Counterweight2, Counterweight2Pin, 1.0)

    MySystem.addRod(Counterweight1, Slappa1, 1.0)
    MySystem.addRod(Counterweight2, Slappa1, 1.0)
    MySystem.addRod(Slappa2, Slappa1, 1.0)
    MySystem.addRod(Slappa2, CounterweightAxle, 1.0)

    

    
    MySystem.addRod(MainAxle, ArmTip, 1.0)
    MySystem.addRod(MainAxle, CounterweightAxle, 1.0)
    MySystem.addRod(ArmTip, CounterweightAxle, 1.0)
    MySystem.addSling(Projectile, ArmTip, 1.0)
    MySystem.addOneWaySlider(Projectile, np.matrix([[0.0],
                                                  [1.0]]), 1)

    MySystem.addOneWaySlider(ArmTip, np.matrix([[-1.0],
                                                  [0.0]]), 1)
    #MySystem.addRod(CounterweightAxle, Counterweight, 1.0)

    return MySystem

def Trebuchet():
    MySystem=calculatemotion.ParticleSystem()
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
    MySystem.addOneWaySlider(Projectile, np.matrix([[0.0],
                                                  [1.0]]), 1)
    


    MySystem.addPin(MainAxle)
    MySystem.addRod(MainAxle, ArmTip, 1.0)
    MySystem.addRod(MainAxle, CounterweightAxle, 1.0)
    MySystem.addRod(ArmTip, CounterweightAxle, 1.0)
    MySystem.addSling(Projectile, ArmTip,  1.0)
    MySystem.addRod(CounterweightAxle, Counterweight, 1.0)

    

    return MySystem

if __name__ == "__main__":

    MySystem=Trebuchet2()
    animation=animation.Animation(MySystem, pygame.display.set_mode((300,400)))
    animation.simanimate(tfinal=6)
    animation.animate()
    #pygame.quit()
    #sys.exit()
