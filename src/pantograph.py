#!/usr/bin/env python3
# -*- coding: utf-8 -*-
__author__ = 'Antoine Weill--Duflos'
__version__ = '1.0.0'
__date__ = '2021/03/22'
__description__ = 'Python Pantograph mech. definitions'

from pyhapi import Mechanisms
import math

class Pantograph(Mechanisms):
    __l = __L = __d = 0
    __th1 = __th2 = 0
    __tau1 = __tau2 = 0
    __f_x = __f_y = 0
    __q_x = __q_y = 0
    __x_E = __y_E = 0
    __pi = math.pi
    __J11 = __J12 = __J21 = __J22 = 0
    __gain = 1

    def __init__(self):
        self.__l = 0.07
        self.__L = 0.09
        self.__d = 0
    
    def forwardKinematics(self, angles):
        l1 = self.__l
        l2 = self.__l
        L1 = self.__L
        L2 = self.__L

        self.__th1 = self.__pi / 180 * angles[0]
        self.__th2 = self.__pi / 180 * angles[1]

        c1 = math.cos(self.__th1)
        c2 = math.cos(self.__th2)
        s1 = math.sin(self.__th1)
        s2 = math.sin(self.__th2)

        xA = l1 * c1
        yA = l1 * s1
        xB = self.__d + l2 * c2
        yB = l2 * s2
        R = math.pow(xA, 2) + math.pow(yA, 2)
        S = math.pow(xB, 2) + math.pow(yB, 2)
        hx = xB-xA
        hy = yB-yA
        hh = math.pow(hx,2) + math.pow(hy,2)
        hm = math.sqrt(hh)
        if (hm == 0):
            cB = 0
            h1x = 0
            h1y = 0
        else:
            cB = - (math.pow(L2,2)-math.pow(L1,2)-hh)/(2*L1*hm)
            h1x = L1*cB * hx/hm
            h1y = L1*cB * hy/hm
        h1h1 = math.pow(h1x,2) + math.pow(h1y,2)
        h1m = math.sqrt(h1h1)
        sB = math.sqrt(1-pow(cB,2))
        if (h1m == 0):
            lx = 0
            ly = 0
        else:
            lx = -L1*sB*h1y/h1m
            ly = L1*sB*h1x/h1m

        x_P = xA + h1x + lx
        y_P = yA + h1y + ly

        phi1 = math.acos((x_P-l1*c1)/L1)
        phi2 = math.acos((x_P- self.__d-l2*c2)/L2)

        c11 = math.cos(phi1)
        s11 = math.sin(phi1)
        c22 = math.cos(phi2)
        s22 = math.sin(phi2)

        dn = L1 * (c11 * s22 - c22 * s11)
        if(dn == 0):
            eta = 0
            nu = 0
        else:
            eta = (-L1 * c11 * s22 + L1 * c22 * s11 - c1 * l1 * s22 + c22 * l1 * s1) / dn
            nu = l2 * (c2 * s22 - c22 * s2)/dn

        self.__J11 = -L1 * eta * s11 - L1 * s11 - l1 * s1
        self.__J12 = L1 * c11 * eta + L1 * c11 + c1 * l1
        self.__J21 = -L1 * s11 * nu
        self.__J22 = L1 * c11 * nu

        self.__x_E = x_P
        self.__y_E = y_P
    
    def torqueCalculation(self, force):
        self.__f_x = force[0]
        self.__f_y = force[1]

        self.__tau1 = self.__J11*self.__f_x + self.__J12*self.__f_y
        self.__tau2 = self.__J21*self.__f_x + self.__J22*self.__f_y

        self.__tau1*= self.__gain
        self.__tau2*= self.__gain

    def op_velocityCalculation(self, q):
        op_vels = [0.0,0.0]
        self.__q_x = q[0]
        self.__q_y = q[1]

        op_vels[0] = self.__J11*self.__q_x + self.__J21*self.__q_y
        op_vels[1] = self.__J12*self.__q_x + self.__J22*self.__q_y

        op_vels[0]*= self.__gain
        op_vels[1]*= self.__gain

        return op_vels

    def forceCalculation(self):
        pass

    def positionControl(self):
        pass

    def inverseKinematics(self):
        pass
    
    def set_mechanism_parameters(self, parameters):
        self.__l = parameters[0]
        self.__L = parameters[1]
        self.__d = parameters[2]

    def set_sensor_data(self, data):
        pass

    def get_coordinate(self):
        return [self.__x_E, self.__y_E]
    
    def get_torque(self):
        return [self.__tau1, self.__tau2]

    def get_angle(self):
        return [self.__th1, self.__th2]
