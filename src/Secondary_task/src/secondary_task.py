#!/usr/bin/env python


import pygame
import numpy as np
import matplotlib.pyplot as plt
from pantograph import Pantograph
from pyhapi import Board, Device, Mechanisms
from pshape import PShape
import serial
from serial.tools import list_ports
import time
import pandas as pd
import os
import uuid
import random
import sys
import time
import rospy
from co_learning_messages.msg import secondary_task_message
import rosgraph

class secondary_task():
    def __init__(self):

        self.ros_running = rosgraph.is_master_online()
        
        if self.ros_running:
            self.pub = rospy.Publisher('Task_status',secondary_task_message,queue_size=1)
            rospy.init_node("secondary_task")
        #self.rate = rospy.rate(50) #Hz
        
        self.other_needle_forces = False
        self.SimpleActuatorMech = Mechanisms
        self.pantograph = Pantograph
        self.robot = PShape

        self.spine_hit_count = 0
        self.success_count = 0

        ##initialize pygame window
        pygame.init()
        self.window = pygame.display.set_mode((1700, 400))   ##twice 600x400 for haptic and VR
        pygame.display.set_caption('Virtual Haptic Device')

        self.screenHaptics = pygame.Surface((800,400))
        self.screenBlank = pygame.Surface((100,400))
        self.screenBlank.fill((255,255,255))
        self.screenVR = pygame.Surface((800,400))

        self.file_path = os.path.realpath(__file__)
        self.directory_path = os.path.dirname(self.file_path)
        self.icon_path = os.path.join(self.directory_path,"robot.png")
        self.vertebra_path = os.path.join(self.directory_path,"vertebra_test.png")
        self.syringe_path = os.path.join(self.directory_path,"syringe2_transparent.png")


        ##add nice icon from https://www.flaticon.com/authors/vectors-market
        self.icon = pygame.image.load(self.icon_path)
        pygame.display.set_icon(self.icon)
        syringe_img = pygame.image.load(self.syringe_path).convert_alpha()
        self.syringe_img = pygame.transform.scale_by(syringe_img,0.25)
      

        ##add text on top to debugToggle the timing and forces
        self.font = pygame.font.Font('freesansbold.ttf', 14)
        self.font_low_time = pygame.font.Font('freesansbold.ttf', 20)

        pygame.mouse.set_visible(True)     ##Hide cursor by default. 'm' toggles it
        
        ##set up the on-screen debugToggle
        self.text = self.font.render('Virtual Haptic Device', True, (0, 0, 0),(255, 255, 255))
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (10, 10)

        self.xc,self.yc = self.screenVR.get_rect().center ##center of the screen
        self.center = np.array([self.xc,self.yc]) 

        ##initialize "real-time" clock
        self.FPS = 500   #in Hertz

        self.max_time = 20 # seconds

        ## Define colors to be used to render different tissue layers and haptic
        self.cSkin      = (210,161,140)
        self.cFat       = (255,174,66)
        self.cLig_one   = (232,229,221)
        self.cLig_two   = (146,146,146)
        self.cLig_three = (252,228,194)
        self.cFluid     = (255,132,115)
        self.cSpinal    = (255,215,0)
        self.cVerte     = (226,195,152)

        self.cOrange = (255,100,0)
        self.cBlack = (100,100,100)
        self.cWhite  = (255,255,255)
        self.cGreen = (0,230,0)


        self.start_screen()
        #self.initialise()

    def initialise(self):

        self.clock = pygame.time.Clock()
        ####Pseudo-haptics dynamic parameters, k/b needs to be <1
        self.K_TISSUE = .8      ##Stiffness between cursor and haptic display
        self.D = 1.5      ##Viscous of the pseudohaptic display

        ##################### Define sprites #####################

        # Load in transparant object images and convert to alpha channel
        self.vertebrae_layer  = pygame.image.load(self.vertebra_path).convert_alpha()
        self.vertebrae_layer  = pygame.transform.scale(self.vertebrae_layer,(63,85))

        # Create pixel masks for every object 
        self.vertebrae_mask   = pygame.mask.from_surface(self.vertebrae_layer)

        # Get the rectangles and obstacle locations for rendering and mask offset
        self.vertebrae_rect   = self.vertebrae_mask.get_rect()

        self.haptic  = pygame.Rect(*self.screenHaptics.get_rect().center, 0, 0).inflate(40,40)
        self.cursor  = pygame.mouse.get_pos()
        self.colorHaptic = self.cOrange #color of the wall

        '''Init all variables'''
        self.xh = np.array(self.haptic.center,dtype='int32')
        self.dxh = np.zeros(2)
        self.xhold = np.zeros(2)
        self.phold = np.zeros(2)

        self.damping = np.zeros(2)
        self.K = np.diag([1000,1000]) # stiffness matrix N/m
        self.kinetic_friction_coefficient = 0.1

        self.dt = 0.01 # intergration step timedt = 0.01 # integration step time
        self.i = 0 # loop counter
        self.t = 0 # time

        # Metrics
        self.max_force_exerted = np.zeros(2)
        self.bone_collision_count = 0
        self.record_deviation_y = []
        self.xhhold = np.zeros(2)
        self.spinal_coord_collision_hit = False
        self.update_prox = True
        self.a = 0
        self.b = 0

        
        self.alpha = random.randint(-100,100)*0.002
        self.tissue_index = 0

        # Declare some simulation booleans to switch between states
        self.robotToggle = True
        self.debugToggle = False
        self.away_from_bone = True
        self.spinal_coord_collision = False
        self.toggle_visual = False
        self.haptic_feedback = True
        self.proceed = False
        self.visual_feedback = True

        self.force_time =[]
        
        # Set all environment parameters to simulate damping in the various tissue layers
        # Damping values are relative to each other based on tissue density
        D_TISSUE_SKIN   = 14.4 + random.randint(-1,1) # dens = 1.1kg/L
        D_TISSUE_FAT    = 11.8 + random.randint(-1,1)# dens = 0.9kg/L
        D_TISSUE_SUPRA  = 15   + random.randint(-1,1)# dens = 1.142kg/L
        D_TISSUE_INTER  = 15   + random.randint(-1,1)# dens = 1.142kg/L
        D_TISSUE_FLAVUM = 15   + random.randint(-1,1)# dens = 1.142kg/L
        D_TISSUE_FLUID  = 13.2 + random.randint(-1,1)# dens = 1.007kg/L
        D_TISSUE_CORD   = 14   + random.randint(-1,1)# dens = 1.075/L
        D_TISSUE_CART   = 14.4 + random.randint(-1,1)# dens = 1.1kg/L


        difficulty_factor = 0.5
        #  Set all environment parameters to simulate elastic stiffness force upon contact in the various tissue layers
        MAX_TISSUE_SKIN     = 6000 + random.randint(-1,1)*1500 
        MAX_TISSUE_FAT      = 2200 + random.randint(-1,1)*550
        MAX_TISSUE_SUPRA    = 9000 + random.randint(-1,1)*2250
        MAX_TISSUE_INTER    = (7500 + random.randint(-1,1)*1875)
        MAX_TISSUE_FLAVUM   = (12000 + random.randint(-1,1)*1000)*difficulty_factor
        MAX_TISSUE_FLUID    = 2400
        MAX_TISSUE_CORD     = (2400 + random.randint(-1,1)*600)
        MAX_TISSUE_CART     = 50000 + random.randint(-1,1)*10000

        #  Set all environment parameters to simulate tissue cutting force upon traversing the various tissue layers
        CUTTING_FORCE_SKIN   = 6037 + random.randint(-1,1)*1500 
        CUTTING_FORCE_FAT    = 2200 + random.randint(-1,1)*550
        CUTTING_FORCE_SUPRA  = 9000 + random.randint(-1,1)*2250 
        CUTTING_FORCE_INTER  = 0#7500 + random.randint(-1,1)*1875 
        CUTTING_FORCE_FLAVUM = 0
        CUTTING_FORCE_FLUID  = 2400 + random.randint(-1,1)*600
        CUTTING_FORCE_CORD   = 2400 + random.randint(-1,1)*600
        CUTTING_FORCE_CART   = (50000 + random.randint(-1,1)*10000)  

        # Initialize total simulation space occupied by human subject (start_pos, end_pos, difference)
        self.simulation_space  = [[550, 0, 350], [0, 500, 500]]

        # Initialize adjustable scaling factors to play around with tissue size 2 px / mm
        wall_size_factor1 = 0.04      + random.randint(-1,1)/400 # SKIN , 5.6 mm 
        wall_size_factor2 = 0.045     + random.randint(-1,1)/400 # FAT 
        wall_size_factor3 = 1/150     + random.randint(-1,1)/4000 # SUPRASPINAL LIGAMENT 0.72mm
        wall_size_factor4 = 0.2       + random.randint(-1,1)/80# INTERSPINAL LIGAMENT 30 mm
        wall_size_factor5 = 0.03      + random.randint(-1,1)/533 # LIGAMENTUM FLAVUM 4.5 mm
        wall_size_factor6 = 2.5/(37.5 *difficulty_factor)    # CEREBROSPINAL FLUID 4 mm
        wall_size_factor7 = 0.1       + random.randint(-1,1)/160 # SPINAL CORD 15 mm
        wall_size_factor8 = 1/13      + random.randint(-1,1)/200 # VERTEBRAE ONE 11.5 mm
        wall_size_factor9 = 0.393333  + random.randint(-1,1)/40 # CARTILAGE disk 118 mm
        wall_size_factor10 = 1/11.5   # VERTEBRAE TWO 

        # Vertical wall layers (x, y, width, height)
        self.wall_layer1  = pygame.Rect(self.simulation_space[0][0],self.simulation_space[1][0],wall_size_factor1*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer2  = pygame.Rect(self.wall_layer1[0]+self.wall_layer1[2],self.simulation_space[1][0],wall_size_factor2*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer3  = pygame.Rect(self.wall_layer2[0]+self.wall_layer2[2],self.simulation_space[1][0],wall_size_factor3*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer4  = pygame.Rect(self.wall_layer3[0]+self.wall_layer3[2],self.simulation_space[1][0],wall_size_factor4*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer5  = pygame.Rect(self.wall_layer4[0]+self.wall_layer4[2],self.simulation_space[1][0],wall_size_factor5*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer6  = pygame.Rect(self.wall_layer5[0]+self.wall_layer5[2],self.simulation_space[1][0],wall_size_factor6*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer7  = pygame.Rect(self.wall_layer6[0]+self.wall_layer6[2],self.simulation_space[1][0],wall_size_factor7*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer8  = pygame.Rect(self.wall_layer7[0]+self.wall_layer7[2],self.simulation_space[1][0],wall_size_factor6*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer9  = pygame.Rect(self.wall_layer8[0]+self.wall_layer8[2],self.simulation_space[1][0],wall_size_factor3*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer10 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.simulation_space[1][0],wall_size_factor9*(self.simulation_space[0][2]),self.simulation_space[1][2])
        self.wall_layer11 = pygame.Rect(self.wall_layer10[0]+self.wall_layer10[2],self.simulation_space[1][0],wall_size_factor3*(self.simulation_space[0][2]),self.simulation_space[1][2])

        # Vertebrae layers modelled as rectangles
        self.wall_layer12 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.simulation_space[1][0],wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])
        self.wall_layer13 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],wall_size_factor8*self.simulation_space[1][2]+30,wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])
        self.wall_layer14 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.wall_layer13[1] + wall_size_factor8*self.simulation_space[1][2]+30,wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])
        self.wall_layer15 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.wall_layer14[1] + wall_size_factor8*self.simulation_space[1][2]+30,wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])
        self.wall_layer16 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.wall_layer15[1] + wall_size_factor8*self.simulation_space[1][2]+30,wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])
        self.wall_layer17 = pygame.Rect(self.wall_layer9[0]+self.wall_layer9[2],self.wall_layer16[1] + wall_size_factor8*self.simulation_space[1][2]+30,wall_size_factor9*(self.simulation_space[0][2]),wall_size_factor8*self.simulation_space[1][2])

        # Store all objects in a dict which can be accessed for collision detection
        self.objects_dict = {'Skin': self.wall_layer1, 'Fat': self.wall_layer2, 'Supraspinal ligament one': self.wall_layer3, 'Interspinal ligament': self.wall_layer4,
                        'Ligamentum flavum': self.wall_layer5, 'Cerebrospinal fluid one': self.wall_layer6, 'Spinal cord': self.wall_layer7,
                        'Cerebrospinal fluid two': self.wall_layer8, 'Supraspinal ligament two':  self.wall_layer9, 'Cartilage': self.wall_layer10,
                        'Supraspinal ligament three': self.wall_layer11, 'Vertebrae one': self.wall_layer12, 'Vertebrae two': self.wall_layer13,
                        'Vertebrae three': self.wall_layer14, 'Vertebrae four': self.wall_layer15, 'Vertebrae five': self.wall_layer16,'Vertebrae six':self.wall_layer17}

        # Initialize a collision dictionary to store booleans corresponding to all objects that are in collision
        self.collision_dict = {'Skin': False, 'Fat': False, 'Supraspinal ligament one': False, 'Interspinal ligament': False,
                            'Ligamentum flavum': False, 'Cerebrospinal fluid one': False, 'Spinal cord': False,
                            'Cerebrospinal fluid two': False, 'Supraspinal ligament two':  False, 'Cartilage': False,
                            'Supraspinal ligament three': False, 'Vertebrae one': False, 'Vertebrae two': False,
                            'Vertebrae three': False, 'Vertebrae four': False, 'Vertebrae five': False,'Vertebrae six':False}

        # Initialize a dictonary which holds all the simulation parameters for efficiency
        self.variable_dict = {'Skin': {'D_TISSUE': D_TISSUE_SKIN, 'max_tissue_force': MAX_TISSUE_SKIN,'tissue_cutting_force': CUTTING_FORCE_SKIN, 'collision_bool': True, 'update_bool': True, 'penetration_bool': False},
                            'Fat' : {'D_TISSUE': D_TISSUE_FAT,'max_tissue_force': MAX_TISSUE_FAT,'tissue_cutting_force': CUTTING_FORCE_FAT, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Supraspinal ligament one': {'D_TISSUE': D_TISSUE_SUPRA,'max_tissue_force': MAX_TISSUE_SUPRA,'tissue_cutting_force': CUTTING_FORCE_SUPRA, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Interspinal ligament': {'D_TISSUE': D_TISSUE_INTER,'max_tissue_force': MAX_TISSUE_INTER,'tissue_cutting_force': CUTTING_FORCE_INTER, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Ligamentum flavum': {'D_TISSUE': D_TISSUE_FLAVUM,'max_tissue_force': MAX_TISSUE_FLAVUM,'tissue_cutting_force': CUTTING_FORCE_FLAVUM, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Cerebrospinal fluid one': {'D_TISSUE': D_TISSUE_FLUID,'max_tissue_force': MAX_TISSUE_FLUID,'tissue_cutting_force': CUTTING_FORCE_FLUID, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Spinal cord': {'D_TISSUE': D_TISSUE_CORD,'max_tissue_force': MAX_TISSUE_CORD,'tissue_cutting_force': CUTTING_FORCE_CORD, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Cerebrospinal fluid two': {'D_TISSUE': D_TISSUE_FLUID,'max_tissue_force': MAX_TISSUE_FLUID ,'tissue_cutting_force': CUTTING_FORCE_FLUID, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Supraspinal ligament two': {'D_TISSUE': D_TISSUE_SUPRA,'max_tissue_force': MAX_TISSUE_SUPRA,'tissue_cutting_force': CUTTING_FORCE_SUPRA, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Cartilage': {'D_TISSUE': D_TISSUE_CART,'max_tissue_force': MAX_TISSUE_CART,'tissue_cutting_force': CUTTING_FORCE_CART, 'collision_bool': True, 'update_bool': True,'penetration_bool': False},
                            'Supraspinal ligament three': {'D_TISSUE': D_TISSUE_SUPRA,'max_tissue_force': MAX_TISSUE_SUPRA,'tissue_cutting_force': CUTTING_FORCE_SUPRA, 'collision_bool': True, 'update_bool': True,'penetration_bool': False}}


        # Compose the rectangles belonging to every vertebrae
        self.vert_rect1 = [self.wall_layer3[0],-0.7*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rect2 = [self.wall_layer3[0],0.3*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rect3 = [self.wall_layer3[0],1.3*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rect4 = [self.wall_layer3[0],2.3*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rect5 = [self.wall_layer3[0],3.3*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rect6 = [self.wall_layer3[0],4.3*self.vertebrae_rect[3]+wall_size_factor8*self.simulation_space[1][2]]
        self.vert_rects = [self.vert_rect1,self.vert_rect2,self.vert_rect3,self.vert_rect4,self.vert_rect5,self.vert_rect6]

        self.window_scale = 3

        self.previous_cursor = None
        self.smoothing_factor = 0.1
        self.proceed = False
        self.time_up = False

        self.time_start = time.time()

        self.run = True

        self.max_needle_pressure = 5000

        self.task_failed = False

        self.fluid = self.max_needle_pressure
        self.render_bar = False
        self.rotate_up = False
        self.rotate_down = False           
        self.update_status = True
        self.start_handover = False

        self.run_simulation()
    
    def send_task_status(self,success,tries):
        if self.ros_running:
            message = secondary_task_message()
            message.success = success
            message.tries = tries
            self.pub.publish(message)
        else:
            return

    def start_screen(self):
        self.run = True

        # Create a surface for the button
        button_surface = pygame.Surface((150, 50))
        
        # Render text on the button
        font = pygame.font.Font(None, 24)
        text = font.render("Start Simulation", True, (0, 0, 0))
        text_rect = text.get_rect(center=(button_surface.get_width()/2, button_surface.get_height()/2))

        # Create a pygame.Rect object that represents the button's boundaries
        button_rect = pygame.Rect(0, 0, 150, 50)  # Adjust the position as needed



        while self.run:
            #Create black canvas to which text can be written
            self.window.blit(self.screenHaptics, (0,0))
            self.window.blit(self.screenBlank,(800,0))
            self.window.blit(self.screenVR, (900,0))

            self.screenHaptics.fill(self.cWhite)
            

            for event in pygame.event.get(): # interrupt function
                if event.type == pygame.KEYUP:
                    if event.key == pygame.QUIT: # enter the main loop after 'e' is pressed
                        self.run = False
                        pygame.display.quit()
                        pygame.quit()     
                    if event.key == ord('q'):   ##Force to quit
                        self.run = False       

                # Check for the mouse button down event
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    # Call the on_mouse_button_down() function
                    if button_rect.collidepoint(event.pos):
                        self.initialise()

            # Check if the mouse is over the button. This will create the button hover effect
            if button_rect.collidepoint(pygame.mouse.get_pos()):
                pygame.draw.rect(button_surface, (220, 220, 220), (1, 1, 148, 48))
            else:
                pygame.draw.rect(button_surface, (0, 0, 0), (0, 0, 150, 50))
                pygame.draw.rect(button_surface, (255, 255, 255), (1, 1, 148, 48))
                pygame.draw.rect(button_surface, (0, 0, 0), (1, 1, 148, 1), 2)
                pygame.draw.rect(button_surface, (0, 100, 0), (1, 48, 148, 10), 2)
                
                
            
            # Shwo the button text
            button_surface.blit(text, text_rect)

            # Draw the button on the screen
            self.screenHaptics.blit(button_surface, (button_rect.x, button_rect.y))

            # Update the game state
            pygame.display.update()


    def rotMat(self,angle):
        transformation_matrix = np.array([[np.cos(-angle), np.sin(-angle)],[-np.sin(-angle),  np.cos(-angle)]])
        return transformation_matrix

    def compute_line(self,begin_pos, end_pos):
        x1 = begin_pos[0]
        x2 = end_pos[0]
        y1 = begin_pos[1]
        y2 = end_pos[1]

        a = (y2-y1)/(x2-x1) #flip gradient due to flipped y
        b = (y2-a*x2)
        
        return a, b
    
    def draw_progress_bar(self,needle_pressure):
        max_height = 160
        bar_height = max_height - (needle_pressure*max_height)/(self.max_needle_pressure)
        bar = pygame.Rect((95,190),(40,bar_height)) #((topleft corner),(width,height))
        pygame.draw.rect(self.screenVR,self.cGreen,bar)
        self.screenVR.blit(self.syringe_img,(70,80))

    def check_collision_with_vertebrae(self,haptic_endpoint):
        # Create endpoint mask for collision detection between endpoint and drawn vertebrae
        haptic_endpoint_mask = pygame.mask.Mask((haptic_endpoint.width, haptic_endpoint.height))
        haptic_endpoint_mask.fill()

        # Compute offset between haptic endpoint and every vertebrae mask
        offsets = [(rect[0] - haptic_endpoint[0], rect[1] - haptic_endpoint[1]) for rect in self.vert_rects]

        # Check collision for every vertebrae and endpoint
        collisions = [haptic_endpoint_mask.overlap(self.vertebrae_mask, offset) for offset in offsets]

        # Check if any of the drawn vertebrae are in collision with the needle tip
        return any(collisions)
    
    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.run = False
            elif event.type == pygame.KEYUP:
                if event.key == ord('m'):
                    pygame.mouse.set_visible(not pygame.mouse.get_visible())
                elif event.key == ord('q'):
                    self.run = False
                    pygame.display.quit()
                    pygame.quit()
                elif event.key == ord('r'):
                    self.rotate_up = False
                elif event.key == ord('e'):
                    self.rotate_down = False
                elif event.key == ord('v'):
                    self.toggle_visual = not self.toggle_visual
                elif event.key == ord('h'):
                    self.haptic_feedback = not self.haptic_feedback
                elif event.key == ord('p'):
                    self.proceed = not self.proceed
                elif event.key == ord('d'):
                    self.debugToggle = not self.debugToggle
                elif event.key == ord('o'):
                    self.visual_feedback = not self.visual_feedback
                elif event.key == ord(' '):
                    self.render_bar = False
            elif event.type == pygame.KEYDOWN:
                if event.key == ord(' '):
                    self.render_bar = True
                elif event.key == ord('r'):
                    self.rotate_up = True
                elif event.key == ord('e'):
                    self.rotate_down = True

        

    def run_simulation(self):
        # Initialises some variables
        

        while self.run:
            # Set some booleans
            self.penetration    = True
            self.collision_bone = False
            self.collision_any  = False
         
            # Keyboard events
            self.process_events()
                
            if self.rotate_up:
                self.alpha += np.deg2rad(0.1)
            if self.rotate_down:
                self.alpha -= np.deg2rad(0.1)
              
            self.xh = np.array(self.haptic.center)
            
            ## Apply low pass filter to mouse position
            if self.i != 0:
                self.cursor = pygame.mouse.get_pos()
                self.cursor = [self.smoothing_factor*self.cursor[0] + (1-self.smoothing_factor)*self.previous_cursor[0], self.smoothing_factor*self.cursor[1] + (1-self.smoothing_factor)*self.previous_cursor[1]]

            else:
                self.cursor = pygame.mouse.get_pos()

            self.xm = np.array(self.cursor) 
            
            ######################################## COMPUTE COLLISIONS WITH ANY TISSUE #######################################
            # Define haptic center and endpoint of our haptic (needle tip)
            self.haptic_endpoint = pygame.Rect(self.haptic.center[0]+np.cos(self.alpha)*250,self.haptic.center[1]+np.sin(self.alpha)*250, 1, 1)
            
            


            # Initialize zero endpoint force and reference position based on cursor or Haply
            self.fe = np.zeros(2)
            self.reference_pos  = self.cursor


            self.collision_bone = self.check_collision_with_vertebrae(self.haptic_endpoint)

            # Update the collision dict to all False to reset collision dict 
            self.collision_dict.update((value, False) for value in self.collision_dict)
            self.collision_flag = False
            
            # Loop over all the objects and check for collision
            for value in self.objects_dict:
                Collision = self.haptic_endpoint.colliderect(self.objects_dict[value])

                # If collision is detected in specific layer only update this value to True
                if Collision:
                    self.collision_any = True
                    self.collision_dict.update({value: True})
                
            ######################################## UPDATE ENVIRONMENT PARAMETERS BASED ON COLLISIONS ########################################

        

            # Loop over all the rectangular objects and check for collision, note that we exclude the vertebrae from the loop
            # The reason being that these are modelled infinitely stiff so they don't need damping etc.
            
            Bones = {'Vertebrae one', 'Vertebrae two', 'Vertebrae three', 'Vertebrae four', 'Vertebrae five','Vertebrae six'}
            for collision in self.collision_dict:
                if collision not in Bones and self.collision_dict[collision] == True:

                    # For the objects(tissues) in collision with the needle tip set the damping value and tissue cuttin force of the environment accordingly
                    # Additionally, flip position, collision boolean and tissue layer index.
                    
                    self.damping        = self.variable_dict[collision]['D_TISSUE'] * self.K
                    self.cutting_force  = self.variable_dict[collision]['tissue_cutting_force']
                    self.collision_bool = self.variable_dict[collision]['collision_bool']
                    self.update_bool    = self.variable_dict[collision]['update_bool']
                    self.tissue_index   = list(self.variable_dict).index(collision) + 1  #We start counting index at zero so +1 for further computation
            
            
            # In case no collisions are detected default the damping value and tissue cutting force of the environment to zero
            if all(value == False for value in self.collision_dict.values()):
                self.damping          = np.zeros(2)
                self.cutting_force    = 0
            
            # Check if any of the rectangular vertebrae are in collision, if so flip bone collision boolean to limit needle movement
            if self.collision_dict['Vertebrae one'] or self.collision_dict['Vertebrae two'] or self.collision_dict['Vertebrae three'] or self.collision_dict['Vertebrae four'] or self.collision_dict['Vertebrae five'] or self.collision_dict['Vertebrae six']:
                self.collision_bone = True
            else:
                pass
            
            ######################################## FORCE FEEDBACK COMPUTATIONS + IMPEDANCE CONTROL ########################################
        
            # Calculate endpoint velocity and update previous haptic state
            self.endpoint_velocity = (self.xhold - self.xh)/self.FPS
            self.xhold = self.xh

            # Implements a sine wave parallel to the needle
            needle_direction = [np.cos(self.alpha),np.sin(self.alpha)]
            needle_perp_direction = np.array([needle_direction[1],-needle_direction[0]])
            faulty_force = 0#np.array(needle_direction)*40000*np.sin(self.t/5)
            
            # Calculate force feedback from impedance controller 
            self.fe = (self.K @ (self.xm-self.xh) - (2*0.7*np.sqrt(np.abs(self.K)) @ self.dxh)) +faulty_force
            
            # Fix the proxy position of needle contact point with skin and update only when no contact is made with any tissue (so when needle is retracted)
            if self.update_prox and self.collision_dict['Skin']:
                begin_pos = (self.haptic.center[0],self.haptic.center[1])
                end_pos   = (self.haptic.center[0]+np.cos(self.alpha)*250, self.haptic.center[1]+ np.sin(self.alpha)*250)
                self.a,self.b = self.compute_line(begin_pos, end_pos)

                self.update_prox = False

            # Enable needle proxy position update as soon as no contact is made with any tissue (so when needle is retracted)
            if all(value == False for value in self.collision_dict.values()):
                self.update_prox = True

            # If collision exists with any tissue layer create a virtual needle path along the needle
            # to compute tissues normal force acting on needle when moving inside tissue 
            if any(value == True for value in self.collision_dict.values()):
                
                # Compute the perpendicular distance to a line (works both for horizontal and diagonal needle path)
                distance_from_line = (self.a*(self.xm[0]-np.cos(self.alpha)*250)-1*(self.xm[1]- np.sin(self.alpha)*250) +self.b)/np.sqrt(self.a**2+(-1)**2)
                self.record_deviation_y.append(distance_from_line)


                # Set tissue stiffness matrix depending on tissue stiffness (assumed equal for all tissues), normal force acting on needle
                # depends on how far reference pos for needle is from projected needle path. 
                tissue_stiffness_matrix = np.diag([750,750])

                # Compute the force and scale with respective angle along x and y axis.
                needle_offset_force = (tissue_stiffness_matrix * distance_from_line)*np.array([np.sin(self.alpha), np.cos(self.alpha)])

                # Add the needle_offset_force to the endpoint force (note that sign of force in x-direction flips if alpha != 0)
                if self.alpha != 0:
                    self.fe += [-needle_offset_force[0,0], needle_offset_force[1,1]]  
                else:
                    self.fe += [needle_offset_force[0,0], needle_offset_force[1,1]]  
                
                # We will use the normal force exerted on the needle by the tissue layers to implement kinetic friction 
                # (note that for every layer passed the kinetic friction increases as the amount of tissues exerting friction increases)
                if self.alpha == 0 and self.dxh[0] > 0:

                    self.tissue_normal_force_x = (tissue_stiffness_matrix * distance_from_line)[0,0]
                    self.tissue_normal_force_y = (tissue_stiffness_matrix * distance_from_line)[1,1]
                    
                    # Note that the kinetic friction is based on normal force so F_x = mu_kinetic * Fn_y and F_y = mu_kinetic * Fn_x
                    frictional_force = (self.tissue_normal_force_x*self.kinetic_friction_coefficient)*self.tissue_index

                    self.fe[0] += frictional_force
                    
                elif self.alpha !=0 and self.dxh[0] > 0:

                    tissue_normal_force = (tissue_stiffness_matrix * distance_from_line)*np.array([np.sin(self.alpha), np.cos(self.alpha)])
                
                    # Note that the kinetic friction is based on normal force so F_x = mu_kinetic * Fn_y and F_y = mu_kinetic * Fn_x
                    frictional_force = (tissue_normal_force*self.kinetic_friction_coefficient)*self.tissue_index

                    self.fe[0] += frictional_force[1,1]
                    self.fe[1] += frictional_force[0,0]
                else:
                    self.fe += np.array([0,0])

            # Compute damping force acting on endpoint due to viscosity of current tissue layer
            self.fd = -self.damping @ self.endpoint_velocity 

            # Apply tissue specific cutting force to needle endpoint (only if needle is moving)
            if any(value == True for value in self.collision_dict.values()) and self.dxh[0]>0:
                    cutting_force_x = self.cutting_force*np.cos(self.alpha)
                    cutting_force_y = self.cutting_force*np.sin(self.alpha)

                    self.fe[0] += -cutting_force_x
                    self.fe[1] += -cutting_force_y

            # Find maximum exerted force during simulation for metric analysis
            if self.i>120:
                if self.fe[0] > self.max_force_exerted[0]:
                    self.max_force_exerted[0] = self.fe[0]
                if self.fe[1] > self.max_force_exerted[1]:
                    self.max_force_exerted[1] = self.fe[1]
            
           
         
            if self.haptic_feedback:
                self.ddxh = self.fe
            
                # Update velocity to accomodate damping
                self.dxh += self.ddxh*self.dt -self.fd

                #dx_perp = np.dot(needle_perp_direction,self.dxh)
     

                #self.dxh[0] = self.dxh[0] - dx_perp*np.sin(self.alpha)
                #self.dxh[1] = self.dxh[1] - dx_perp*np.cos(self.alpha)


                # In case collision occurs with vertebrae simulate an infinitely stiff bone
                if self.collision_bone and self.away_from_bone:
                    self.phold = self.xh
                    self.away_from_bone = False
                    self.bone_collision_count += 0.5

                if self.reference_pos[0] >= self.phold[0] and not self.away_from_bone:
                    self.dxh = np.zeros(2)
                else: 
                    self.away_from_bone =  True

                # Loop over the detected collisions dictonary (excluding vertebrae), in case collision is detected retrieve tissue parameters from parameter dict
                Bones = {'Vertebrae one', 'Vertebrae two', 'Vertebrae three', 'Vertebrae four', 'Vertebrae five', 'Vertebrae six'}
                for collision in self.collision_dict:
                    if collision not in Bones and self.collision_dict[collision] == True:

                        # Set the maximum tissue force, the maximum force exerted by needle pre-puncture
                        max_tissue_force = self.variable_dict[collision]['max_tissue_force']

                        if collision == 'Spinal cord' and self.i > 120:
                            self.spinal_coord_collision_hit = True
                            self.spinal_coord_collision = True
                        else:
                            self.spinal_coord_collision = False

                        # Check if collision has occured and fix the current position of the haptic as long as no puncture has occured 
                        if self.update_bool and self.i>120:
                            phold = self.xh
                            self.variable_dict[collision]['update_bool'] = False

                        

                        # Compute total endpoint force applied to haptic by the user and check if it exceeds the penetration threshold
                        penetration_bool = self.variable_dict[collision]['penetration_bool']
                        if not penetration_bool:
                            F_pen = (self.K @ (self.xm-self.xh) - (2*0.7*np.sqrt(np.abs(self.K)) @ self.dxh))[0]*np.cos(self.alpha) 
                        else:
                            F_pen = 0
                        
                        if F_pen > max_tissue_force:
                            self.variable_dict[collision]['penetration_bool'] = True
                            penetration_bool = True
                    
                        if self.xh[0] > self.reference_pos[0]:
                            pass
                        elif not penetration_bool:
                            self.dxh = np.zeros(2)
            else:

                # Loop over this if haptic feedback is turned
                self.ddxh = (self.K @ (self.xm-self.xh) - (2*0.7*np.sqrt(np.abs(self.K)) @ self.dxh)) 
                self.dxh += self.ddxh*self.dt

            if all(value == False for value in self.collision_dict.values()):
                for collision in self.collision_dict:
                    if collision not in Bones:
                        self.variable_dict[collision]['penetration_bool'] = False
                        self.variable_dict[collision]['update_bool'] = True

            # Loop trough remaining integration steps       
            self.xhhold = self.xh
            self.xh = self.dxh*self.dt + self.xh
            self.i += 1
            self.t += self.dt
        
            self.haptic.center = self.xh 

           
            self.render_screen()
            
       

            self.previous_cursor = self.cursor 

            

            #Slow down the loop to match FPS
            self.clock.tick(self.FPS)


            if self.spinal_coord_collision or self.task_failed:
                self.run = False
                self.spine_hit_count += 1
                time.sleep(0.5)
                self.send_task_status(False, self.spine_hit_count + self.success_count)
                self.end_screen()
                
           
            if self.time_left<=0:
                self.run = False
                time.sleep(0.5)
                self.time_up = True
                self.send_task_status(False, self.spine_hit_count + self.success_count)
                self.end_screen()

            if self.fluid <=0:
                # self.run = False
                # self.success_count += 1
                # time.sleep(0.5)
                # self.send_task_status(False, self.spine_hit_count + self.success_count)
                # self.end_screen()
                pass
                
    def render_screen(self):
         ######################################## Graphical output ########################################

        ##Render the haptic surface
        self.screenHaptics.fill(self.cWhite)
        
        ##Change color based on effort
        colorMaster = (255,\
            255-np.clip(np.linalg.norm(np.abs(self.K_TISSUE)*(self.xm-self.xh)/self.window_scale)*15,0,255),\
            255-np.clip(np.linalg.norm(np.abs(self.K_TISSUE)*(self.xm-self.xh)/self.window_scale)*15,0,255)) #if collide else (255, 255, 255)
        
        pygame.draw.line(self.screenHaptics, (0, 0, 0), (self.haptic.center),(self.haptic.center+2*self.K_TISSUE*(self.xm-self.xh)))
        pygame.draw.rect(self.screenHaptics, colorMaster, self.haptic,border_radius=4)
        
        
        ######################################## Robot visualization ########################################
        # update individual link position
        if self.robotToggle:
            self.robot.createPantograph(self.screenHaptics,self.xh)
            
        ##Render the VR surface
        self.screenVR.fill(self.cWhite)
    
        ### Visualize all components of the simulation

        # Draw all the vertical tissue layers
        pygame.draw.rect(self.screenVR,self.cSkin,self.wall_layer1, border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cFat,self.wall_layer2,  border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_one,self.wall_layer3, border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_two,self.wall_layer4,border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_three,self.wall_layer5,border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cFluid,self.wall_layer6,border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cSpinal,self.wall_layer7,border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cFluid,self.wall_layer8,border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_one,self.wall_layer9, border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_two,self.wall_layer10, border_radius = 2)
        pygame.draw.rect(self.screenVR,self.cLig_one,self.wall_layer11, border_radius = 2)
        
        # Draw all the vertebrae
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer12, border_radius = 4)
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer13, border_radius = 4)
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer14, border_radius = 4)
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer15, border_radius = 4)
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer16, border_radius = 4)
        pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer17, border_radius = 4)

        # Draw all the vertebrae     
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect1[0],self.vert_rect1[1]))
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect2[0],self.vert_rect2[1]))
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect3[0],self.vert_rect3[1]))
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect4[0],self.vert_rect4[1]))
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect5[0],self.vert_rect5[1]))  
        self.screenVR.blit(self.vertebrae_layer,(self.vert_rect6[0],self.vert_rect6[1]))
        
        # Draw needle 
        x1 = self.haptic.center[0]
        y1 = self.haptic.center[1]
        x2 = x1 + np.cos(self.alpha)*250
        y2 = y1 + np.sin(self.alpha)*250
        x2_2 = np.sin(-self.alpha)*25
        y2_2 = np.cos(-self.alpha)*25

        pygame.draw.line(self.screenVR, self.cBlack, (x1,y1), (x2, y2), 2 )
        pygame.draw.line(self.screenVR, self.cBlack, (x1,y1), (x1+x2_2, y1+ y2_2), 2 )
        pygame.draw.line(self.screenVR, self.cBlack, (x1,y1), (x1-x2_2, y1- y2_2), 2 )
        
        # handles the logic behind the progress bar of draining the fluid
        if self.haptic_feedback:
            if self.collision_dict['Cerebrospinal fluid one'] and self.i > 350 and self.visual_feedback:
                if self.render_bar:
                    self.fluid -= 1
                    if self.fluid >0 :
                        fluid_left = round(self.fluid/100,2)
                        text_surface = self.font.render(f'Fluid left: {fluid_left}ml', True, (0,0,0))
                        self.screenVR.blit(text_surface, (0, 60))
                        self.draw_progress_bar(self.fluid)
                    else:
                        self.start_handover = True
                        self.draw_progress_bar(0)
                elif not self.render_bar and not self.start_handover:
                    space_bar_text = self.font_low_time.render(f'Press space bar to start draining the fluid!', True, (20,150,40))
                    self.screenVR.blit(space_bar_text, (0, 60))

        time_elapsed =  time.time() - self.time_start
        self.time_left = self.max_time - time_elapsed  

        # Handles the logic of the handover phase
        if self.start_handover:
            if self.update_status:
                self.time_start = time.time()
                self.send_task_status(True, self.spine_hit_count + self.success_count)
                self.update_status = False
            keep_mouse_in_fluid_text_1 = self.font_low_time.render('Don\'t remove the needle from the epidural space', True, (20,150,40))
            keep_mouse_in_fluid_text_2 = self.font_low_time.render('untill the robot has provided a piece of cotton!',True,(20,150,40))
            keep_mouse_in_fluid_text_3 = self.font_low_time.render('You can release the spacebar now!',True,(20,150,40))
            self.screenVR.blit(keep_mouse_in_fluid_text_1, (0, 60))
            self.screenVR.blit(keep_mouse_in_fluid_text_2, (0, 80))
            self.screenVR.blit(keep_mouse_in_fluid_text_3, (0, 100))
            if not self.collision_dict['Cerebrospinal fluid one']:
                self.send_task_status(False, self.spine_hit_count + self.success_count)
                self.task_failed = True

            if self.time_left <= 10:
                text_time = self.font_low_time.render(f"Time left for handover: {self.time_left:.2f}s",True,(255,0,0))
                self.screenVR.blit(text_time,(0,378))
            else:
                text_time = self.font.render(f"Time left for handover: {self.time_left:.2f}s",True,(0,0,0)) # Display the time left to complete the simulation
                self.screenVR.blit(text_time,(0,380))
                        

        #toggle a mask over the spine
        if self.toggle_visual:
            pygame.draw.rect(self.screenVR,self.cSkin,(self.simulation_space[0][0],0,self.simulation_space[0][1],self.simulation_space[1][1]), border_radius = 0)

        if self.i < 350:
            # # Draw all the vertebrae
            pygame.draw.rect(self.screenVR,self.cWhite,(self.simulation_space[0][0],0,self.simulation_space[0][1],self.simulation_space[1][1]), border_radius = 0)

            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer12, border_radius = 4)
            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer13, border_radius = 4)
            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer14, border_radius = 4)
            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer15, border_radius = 4)
            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer16, border_radius = 4)
            pygame.draw.rect(self.screenVR,self.cVerte,self.wall_layer17, border_radius = 4)

            # Draw all the vertebrae
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect1[0],self.vert_rect1[1])) 
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect2[0],self.vert_rect2[1]))
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect3[0],self.vert_rect3[1]))
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect4[0],self.vert_rect4[1]))
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect5[0],self.vert_rect5[1]))  
            self.screenVR.blit(self.vertebrae_layer,(self.vert_rect6[0],self.vert_rect6[1]))
        
        # Visualize toggles on display
        text_surface1 = self.font.render("Press 'e' to rotate needle up", True, (0, 0, 0),(255, 255, 255))
        text_surface2 = self.font.render("Press 'r' to rotate needle down", True,(0, 0, 0), (255, 255, 255))
        text_surface3 = self.font.render("Press 'v' to hide epidural space", True, (0, 0, 0), (255, 255, 255))

        
        self.screenVR.blit(text_surface1, (0, 0))
        self.screenVR.blit(text_surface2, (0, 20))
        #self.screenVR.blit(text_surface3, (0, 40))

        ##Fuse it back together
        self.window.blit(self.screenHaptics, (0,0))
        self.window.blit(self.screenBlank,(800,0))
        self.window.blit(self.screenVR, (900,0))

        ##Print status in  overlay
        if self.debugToggle: 
            
            text = self.font.render("FPS = " + str(round(self.clock.get_fps())) + \
                                "  xm = " + str(np.round(10*self.xm)/20000) +\
                                "  xh = " + str(np.round(10*self.xh)/20000) +\
                                "  fe = " + str(np.round(10*self.fe)/20000) \
                                , True, (0, 0, 0), (255, 255, 255))
            self.window.blit(text, self.textRect)
        self.force_time.append(self.fe[0])

        if self.haptic_feedback and self.visual_feedback:
            if self.collision_dict['Spinal cord']:
                self.spinal_coord_collision_hit = True
                GB = min(255, max(0, round(255 * 0.5)))
                self.window.fill((255, GB, GB), special_flags = pygame.BLEND_MULT)

        pygame.display.flip()  


    def end_screen(self):
        self.run = True

        # Create a surface for the button
        button_surface = pygame.Surface((150, 50))
        
        # Render text on the button
        font = pygame.font.Font(None, 24)
        text = font.render("Restart Simulation", True, (0, 0, 0))
        font_message = pygame.font.Font(None, 38)
        bad_text = font_message.render("You hit the patient's spine! Please try again", True, (240, 0, 0))
        good_text_1 = font_message.render(f"You succesfully drained all the fluid", True, (20,150,40))
        good_text_2 = font_message.render(f"from the epidural space, well done!", True, (20,150,40))
        hit_count_text = font.render(f"Spine hits:  {self.spine_hit_count}", True, (0, 0, 0))
        success_text = font.render(f"Successes: {self.success_count}", True, (0, 0, 0))
        completion_text = font.render(f"You succesfully completed: {self.spine_hit_count+self.success_count} simulation runs!", True, (0, 0, 0))
        completion_text_2 = font.render(f"press 'q' to exit simulation", True, (0, 0, 0))
        text_rect = text.get_rect(center=(button_surface.get_width()/2, button_surface.get_height()/2))
        time_up_text = font_message.render(f"You did not obtain the item in time!", True, (240, 0, 0))
        please_again_text = font_message.render(f"Please try again", True, (240, 0, 0))
        handover_failed_text = font_message.render(f"Needle removed from epidural space without", True, (240, 0, 0))
        handover_failed_text_2 = font_message.render(f"successfull handover of item, task failed!", True, (240, 0, 0))

        # Create a pygame.Rect object that represents the button's boundaries
        button_rect = pygame.Rect(0, 0, 150, 50)  # Adjust the position as needed

        tries = self.spine_hit_count+self.success_count
        # Increase time constraint by 2 seconds after each successfull attempt
        self.max_time = 30 - self.success_count*2



        while self.run:
            #Create black canvas to which text can be written
            self.window.blit(self.screenHaptics, (0,0))
            self.window.blit(self.screenBlank,(800,0))
            self.window.blit(self.screenVR, (900,0))

            self.screenHaptics.fill(self.cWhite)


            for event in pygame.event.get(): 
                if event.type == pygame.KEYUP:
                    if event.key == ord('q') and tries <10:   
                        self.run = False 
                        pygame.display.quit()
                        pygame.quit()   
                    elif event.key == ord('q') and tries >= 10:
                        self.run = False
                        self.save_stats()


                # Check for the mouse button down event
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and tries < 10:
                    # Call the on_mouse_button_down() function
                    if button_rect.collidepoint(event.pos):
                        self.initialise()

            if tries < 10:
                # Check if the mouse is over the button. This will create the button hover effect
                if button_rect.collidepoint(pygame.mouse.get_pos()):
                    pygame.draw.rect(button_surface, (220, 220, 220), (1, 1, 148, 48))
                else:
                    pygame.draw.rect(button_surface, (0, 0, 0), (0, 0, 150, 50))
                    pygame.draw.rect(button_surface, (255, 255, 255), (1, 1, 148, 48))
                    pygame.draw.rect(button_surface, (0, 0, 0), (1, 1, 148, 1), 2)
                    pygame.draw.rect(button_surface, (0, 100, 0), (1, 48, 148, 10), 2)
                    
                # Show the button text
                button_surface.blit(text, text_rect)
                # Draw the button on the screen
                self.screenHaptics.blit(button_surface, (button_rect.x, button_rect.y))

                self.screenHaptics.blit(hit_count_text,(450,20))
                self.screenHaptics.blit(success_text,(450,50))

                # Draw respective good and bad text on screen
                if self.spinal_coord_collision and not self.time_up and not self.task_failed:
                    self.screenHaptics.blit(bad_text,(20,120))
                elif not self.spinal_coord_collision and not self.time_up and not self.task_failed:
                    self.screenHaptics.blit(good_text_1,(20,120))
                    self.screenHaptics.blit(good_text_2,(25,150))
                elif self.task_failed:
                    self.screenHaptics.blit(handover_failed_text,(20,120))
                    self.screenHaptics.blit(handover_failed_text_2,(20,150))
                else:
                    self.screenHaptics.blit(time_up_text,(20,120))
                    self.screenHaptics.blit(please_again_text,(120,150))

            else:
                self.screenHaptics.blit(completion_text,(20,120))
                self.screenHaptics.blit(completion_text_2,(20,140))

            pygame.display.update()


    def save_stats(self):

        pygame.display.quit()
        pygame.quit()

        record_deviation_y = np.array(self.record_deviation_y)
        std_y = np.std(record_deviation_y)

        # save metrics to the csv file
        d = [f"Haptic feedback: {self.haptic_feedback}" ,f"Time taken: {0:.2f} s",f"Distance to fluid: {(self.wall_layer6[0] - self.haptic_endpoint[0])*2}mm",f"Number of bone hits:{int(self.bone_collision_count)}",f"Spinal coord hit: {self.spinal_coord_collision_hit}",f"Maximum exerted force X:{self.max_force_exerted[0]/10000}",f"Maximum exerted force Y: {self.max_force_exerted[1]/10000}",f"Deviation inside of tissue: {std_y}"]

        df = pd.DataFrame(data=d)

        unique_identifier = str(uuid.uuid4())

        save_path = os.path.join(self.directory_path,f"Data_files/Data_id:{unique_identifier}.csv")

        print(f"saved file to: {save_path}")

        df.to_csv(save_path,mode='a',header=False,index=False)

        xs = [x for x in range(len(self.force_time[150:]))]

        plt.plot(xs, self.force_time[150:])
        plt.show()
        # Make sure to close the plt object once done
        plt.close()

if __name__ == '__main__':
    game = secondary_task()
    game.initialise()
