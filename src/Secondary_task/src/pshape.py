# -*- coding: utf-8 -*-
"""
Created on Mon Feb  7 16:02:10 2022

@author: willemet.l
"""

from pyhapi import Mechanisms
from pantograph import Pantograph
import math
import numpy as np
import pygame


class PShape:
    
    def createPantograph(screen,xh):
        window_scale = 3
        xc,yc = [0,300]
        colorLinks = (150,150,150)
        
        #################### Define Robot #######################
        # ROBOT PARAMETERS
        l = [80,100]*window_scale # links length l1, l2

        xrc = [300,0] ## center location of the robot in pygame
        
        pr = np.array([(xh[0]-xrc[0])/window_scale, -(xh[1]-xrc[1])/window_scale]) ##base is at (0,0) in robot coordinates
        #q = model.IK(pr)
        
        #################### Compute inverse kinematics#######################
        ql = np.zeros([2])
        qr = np.zeros([2])
        r = np.sqrt(pr[0]**2+pr[1]**2)
        try:
            ql[1] = np.pi - math.acos((l[0]**2+l[1]**2-r**2)/(2*l[0]*l[1]))
        except:
            ql[1]=0
        
        try:
            ql[0] = math.pi - math.atan2(pr[1],-pr[0]) - math.acos((l[0]**2-l[1]**2+r**2)/(2*l[0]*r))
        except:
            ql[0]=0
        try:
            qr[0] = math.atan2(pr[1],pr[0]) + math.acos((l[0]**2-l[1]**2+r**2)/(2*l[0]*r))
        except:
            qr[0]=0
        
        #################### Joint positions #######################

        xr0 =       np.dot(window_scale,[0.0,                      0.0])   #Position of the base
        xr1l = xr0 + np.dot(window_scale,[l[0]*np.cos(ql[0]),       l[0]*np.sin(ql[0])]) #Position of the first link (left)
        xr1r = xr0 + np.dot(window_scale,[l[0]*np.cos(qr[0]),       l[0]*np.sin(qr[0])]) #Position of the first link (right)
        xr2 = xr1l + np.dot(window_scale,[l[1]*np.cos(ql[0]+ql[1]), l[1]*np.sin(ql[0]+ql[1])]) #Position of the second link
        
        #################### Draw the joints and linkages #######################
        pygame.draw.lines (screen, colorLinks, False,\
                           [(xr0[0] + xrc[0], -xr0[1] + xrc[1]), \
                            (xr1l[0] + xrc[0], -xr1l[1] + xrc[1])], 15) # draw links
            
        pygame.draw.lines (screen, colorLinks, False,\
                           [(xr1l[0] + xrc[0]      ,-xr1l[1] + xrc[1]), \
                            (xr2[0] + xrc[0]      ,-xr2[1] + xrc[1])], 14)
            
        pygame.draw.lines (screen, colorLinks, False,\
                           [(xr0[0] + xrc[0], -xr0[1] + xrc[1]), \
                            (xr1r[0] + xrc[0], -xr1r[1] + xrc[1])], 15) # draw links
            
        pygame.draw.lines (screen, colorLinks, False,\
                           [(xr1r[0] + xrc[0]      ,-xr1r[1] + xrc[1]), \
                            (xr2[0] + xrc[0]      ,-xr2[1] + xrc[1])], 14)
        
        pygame.draw.circle(screen, (0, 0, 0),\
                           (int(xr0[0]) + xrc[0] ,int(-xr0[1]) + xrc[1]), 15) # draw shoulder / base
        pygame.draw.circle(screen, (200, 200, 200),\
                           (int(xr0[0]) + xrc[0] ,int(-xr0[1]) + xrc[1]), 6) # draw shoulder / base
        pygame.draw.circle(screen, (0, 0, 0),\
                           (int(xr1l[0]) + xrc[0],int(-xr1l[1]) + xrc[1]), 15) # draw elbow left
        pygame.draw.circle(screen, (200, 200, 200),\
                           (int(xr1l[0]) + xrc[0],int(-xr1l[1]) + xrc[1]), 6) # draw elbow left
        pygame.draw.circle(screen, (0, 0, 0),\
                           (int(xr1r[0]) + xrc[0],int(-xr1r[1]) + xrc[1]), 15) # draw elbow right
        pygame.draw.circle(screen, (200, 200, 200),\
                           (int(xr1r[0]) + xrc[0],int(-xr1r[1]) + xrc[1]), 6) # draw elbow right
        pygame.draw.circle(screen, (255, 0, 0),\
                           (int(xr2[0]) + xrc[0],int(-xr2[1]) + xrc[1]), 5) # draw hand / endpoint
            
