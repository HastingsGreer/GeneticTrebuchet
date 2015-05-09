import numpy as np
import calculatemotionanimated
import sys
import pygame
from pygame.locals import *


trebuchetarchive=dict()


DISPLAYSURF=pygame.display.set_mode((300,400))
def randomgenome():
    genome=[np.random.randint(9) for __ in range(39)]
    return genome

def transcribe(genome, savesystem=False):

    if (str(genome) in trebuchetarchive) and not savesystem:              #checks if has been tried before, uses previous result if so.
        print "efficiency", trebuchetarchive[str(genome)]
        return trebuchetarchive[str(genome)]


    pgene=genome[0:2]
    agene=genome[2:16]
    cgene=genome[16:18]
    lgene=genome[18:39]
    
    prna=pgene
    arna=zip(agene[::2], agene[1::2])
    crna=cgene
    lrna=zip(lgene[::3], lgene[1::3], lgene[2::3])

    print prna, arna, crna, lrna
    
    system=calculatemotionanimated.ParticleSystem()
    
    
    system.AddParticle(100, float(crna[0]),-float(crna[1]), 0.0,0.0, color=(0,0,0))
    for codon in arna:
        system.AddParticle(10, float(codon[0]), -float(codon[1]), 0.0, 0.0)

    system.AddParticle(10, 2.0, 2.0, 0.0, 0.0)
    system.AddParticle(1, float(prna[0]), -float(prna[1]), 0.0,0.0, color=(45,160,0))

    system.AddRod(1,9, 1)

    for codon in lrna:
        if codon[2]%2==1:
            if codon[1]!=codon[0]:
                system.AddRod(codon[0],codon[1], 1)
        else:
            angle= np.pi*codon[1]/9.0
            normal=np.matrix([[np.sin(angle)],[np.cos(angle)]])                             #this is pretty much self evident, yes?
            system.AddSlider(codon[0], normal, 0)

    for particle in range(len(system.particlelist)):
        if system.particlelist[particle].Constraints!=[]:
            system.AddGravity(particle, np.matrix([[0.0],
                                                   [-9.8]]))

    magicsouljar=system.FillStateVector()
   
    system.FillFromStateVector(magicsouljar) #revivivivie, revive from the depthes of helle!



    try:
        system.checkLegality()               #pretty sure this does nothing, supposed to catch errors before the integrator runs
    except:
        return 0

   
    

    system.projectiledirection=0           #this also seems to do nothing, but is supposed to prevent spinny things.  (nonetheless,                                       
    system.directionchanges=0              #spinny things seem to happen.) Spinny things are bad because they promote sensitive dependence, and also drugs
    def endafter4directionchanges(y):
	if (np.sign(y[38])!=system.projectiledirection) and abs(y[38]>=2):
	    system.projectiledirection=np.sign(y[38])
            system.directionchanges=system.directionchanges+1
        if system.directionchanges>=4:
            return True
        return False


    system.endcondition=endafter4directionchanges
    animation=calculatemotionanimated.Animation(system, DISPLAYSURF)

    



    animation.simanimate()             #the most important line in the world

    miny=.5-min((np.array(animation.ys)).flatten())                            #total height


    vx=[np.abs(step[38])*step[39] for step in animation.solution]                #ranges
    

    vxa=np.array(vx)

    maxxy=np.max(vxa.flatten())
    
    print miny
    print maxxy
    print "efficiency:", maxxy/(9.8*1000*miny)
    
    if savesystem==True:
        return system, animation
    trebuchetarchive[str(genome)]=maxxy/(9.8*1000*miny)

    
    return maxxy/(9.8*1000*miny)



    
    
            
