#!/usr/bin/env python3

import signal
import sys
import pygame
import numpy as np
import time
import os
import random
import time
import rospy
import rosgraph
from math import exp

from std_msgs.msg import String

from co_learning_messages.msg import secondary_task_message,control_status_message
from co_learning_secondary_task.pantograph import Pantograph
from co_learning_secondary_task.pyhapi import Mechanisms
from co_learning_secondary_task.pshape import PShape      

class secondary_task():
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        self.initialise_ros()
        self.initialise_pygame()
        self.initialise_others()
        self.start_screen()
 
    def signal_handler(self, sig, frame):
        sys.exit(0)

    def initialise_others(self):
        self.SimpleActuatorMech = Mechanisms
        self.pantograph = Pantograph
        self.robot = PShape
    
        self.fail_count = 0
        self.success_count = 0

        pygame.mouse.set_visible(True)     ##Hide cursor by default. 'm' toggles it
        
        ##initialize "real-time" clock
        self.FPS = 400   #in Hertz (maybe)

    def initialise_ros(self):
        self.ros_running = rosgraph.is_master_online()
        self.msg = None
        self.display_text = None
        
        if self.ros_running:
            self.pub = rospy.Publisher('task_status',secondary_task_message,queue_size=1)
            rospy.Subscriber('control_status',control_status_message,self.status_callback)
            rospy.init_node("secondary_task")

    def status_callback(self,msg):
        self.reset = msg.reset
        self.phase = msg.phase

        if self.phase == 5:
            self.send_task_status(start=0, end=0, draining_status=0, time=0)

    def check_print_text(self):
        if self.display_text_flag and self.display_text is not None:
            lines = self.split_text("Robot: " + "\"" + self.display_text + "\"", self.font, max_width = 500)
            y_offset = 450
            
            for line in lines:
                text_surface = self.font.render(line, True, (0, 0, 0), (255, 255, 255))
                self.screenVR.blit(text_surface, (10, y_offset))
                y_offset += self.font.get_linesize()  # Move to the next line height

            if self.get_the_time:
                self.time_init = pygame.time.get_ticks()
                self.get_the_time = False

            time_current = pygame.time.get_ticks() - self.time_init

            if time_current > 1000:
                self.get_the_time = True
                self.display_text_flag = False

    def initialise_pygame(self):
        ##initialize pygame window
        pygame.init()
        pygame.mixer.init()
        self.window = pygame.display.set_mode((1700, 500)) 
        pygame.display.set_caption('Virtual Haptic Device')

        self.screenHaptics = pygame.Surface((800,500))
        self.screenBlank = pygame.Surface((100,500))
        self.screenBlank.fill((255,255,255))
        self.screenVR = pygame.Surface((800,500))

        # Sets_fonts
        self.font = pygame.font.Font('freesansbold.ttf', 14)
        self.font_low_time = pygame.font.Font('freesansbold.ttf', 20)

        self.load_images()
        self.initialise_simulation_parameters()

    def initialise_simulation_parameters(self):
        # Define colors to be used to render different tissue layers and haptic
        self.cSkin      = (210,161,140)
        self.cFat       = (255,174,66)
        self.cLig_one   = (232,229,221)
        self.cLig_two   = (146,146,146)
        self.cLig_three = (252,228,194)
        self.cFluid     = (255,132,115)
        self.cSpinal    = (255,215,0)
        self.cVerte     = (226,195,152)
        self.cOrange    = (255,100,0)
        self.cBlack     = (100,100,100)
        self.cWhite     = (255,255,255)
        self.cGreen     = (0,230,0)

        self.time_start = None

        self.clock = pygame.time.Clock()

        # Pseudo-haptics dynamic parameters, k/b needs to be <1
        self.K_TISSUE = .8      # Stiffness between cursor and haptic display
        self.D = 1.5            # Viscous of the pseudohaptic display

        # Create pixel masks for every object 
        self.vertebrae_mask   = pygame.mask.from_surface(self.vertebrae_layer)

        # Get the rectangles and obstacle locations for rendering and mask offset
        self.vertebrae_rect   = self.vertebrae_mask.get_rect()

        self.haptic  = pygame.Rect(*self.screenHaptics.get_rect().center, 0, 0).inflate(40,40)
        self.cursor  = pygame.mouse.get_pos()
        self.colorHaptic = self.cOrange #color of the wall

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
        self.handover_successful = False
        self.haptic_feedback = True
        self.proceed = False
        self.visual_feedback = True
        self.penetration    = True
        self.collision_bone = False
        self.collision_any  = False
        self.bar_pressed = False
        self.needle_removed_too_soon = False
        self.needle_removed_too_soon_2 = False
        self.bar_released_too_soon = True
        self.update_draining_start = True
        self.update_collision = True
        self.bone_collision_flag = True
        self.success = False
        self.needle_removed_during_draining = True
        self.border_rendered = False
        self.reset = False
        self.display_text_flag = False
        self.get_the_time = True
        self.window_scale = 3
        self.time_init = 0
        self.lock = False

        self.previous_cursor = None
        self.smoothing_factor = 0.1
        self.proceed = False
        self.time_up = False

        self.run = True
        self.count = 0

        self.max_time = 60 # seconds
        self.time_left = self.max_time

        self.max_needle_pressure = 6000
        self.max_tries = 100

        self.task_failed = False

        self.fluid = self.max_needle_pressure
        self.render_bar = False
        self.rotate_up = False
        self.rotate_down = False           
        self.update_status = True
        self.start_handover = False
        self.needle_removed_too_soon_update = True

        self.xc,self.yc = self.screenVR.get_rect().center ##center of the screen
        self.center = np.array([self.xc,self.yc]) 

        self.initialise_layers()

    def load_images(self):
        self.file_path = os.path.realpath(__file__)
        self.directory_path = os.path.dirname(self.file_path)
        self.icon_path = os.path.join(self.directory_path,"robot.png")
        self.vertebra_path = os.path.join(self.directory_path,"vertebra_test.png")
        self.syringe_path = os.path.join(self.directory_path,"syringe2_transparent.png")

        sucess_sound_path = os.path.join(self.directory_path,"success_sound.mp3")
        failure_sound_path = os.path.join(self.directory_path,"failure_sound.mp3")


        self.success_chime = pygame.mixer.Sound(sucess_sound_path)
        self.failure_claxon = pygame.mixer.Sound(failure_sound_path)

        ##add nice icon from https://www.flaticon.com/authors/vectors-market
        self.icon = pygame.image.load(self.icon_path)
        pygame.display.set_icon(self.icon)
        syringe_img = pygame.image.load(self.syringe_path).convert_alpha()
        self.syringe_img = pygame.transform.scale_by(syringe_img,0.25)

        # Load in transparant object images and convert to alpha channel
        self.vertebrae_layer  = pygame.image.load(self.vertebra_path).convert_alpha()
        self.vertebrae_layer  = pygame.transform.scale(self.vertebrae_layer,(63,85))


    def initialise_layers(self):
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
        self.collision_dict = {
                            'Skin': False, 'Fat': False, 'Supraspinal ligament one': False, 'Interspinal ligament': False,
                            'Ligamentum flavum': False, 'Cerebrospinal fluid one': False, 'Spinal cord': False,
                            'Cerebrospinal fluid two': False, 'Supraspinal ligament two': False, 'Cartilage': False,
                            'Supraspinal ligament three': False, 'Vertebrae 1': False, 'Vertebrae 2': False,
                            'Vertebrae 3': False, 'Vertebrae 4': False, 'Vertebrae 5': False, 'Vertebrae 6': False}


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

        self.Bones = {'Vertebrae one', 'Vertebrae two', 'Vertebrae three', 'Vertebrae four', 'Vertebrae five','Vertebrae six'}



    def send_task_status(self, start=None, end=None, draining_status=None, time=None):
        if self.ros_running:
            if self.msg is None:
                message = secondary_task_message()
            else:
                message = self.msg
            if start is not None:
                message.draining_starts = start
            if end is not None:
                message.draining_successful = end
            if draining_status is not None:
                message.draining_status = draining_status
            if time is not None:
                message.time_left = time
            
            self.pub.publish(message)
            self.msg = message  # Save the message for future updates

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
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.reset:
                    # Call the on_mouse_button_down() function
                    if button_rect.collidepoint(event.pos):
                        self.run_simulation()

            # Check if the mouse is over the button. This will create the button hover effect
            if self.reset:
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
        needle_location_x = 95
        needle_location_y = 240
        bar_height = max_height - (needle_pressure*max_height)/(self.max_needle_pressure)
        bar = pygame.Rect((needle_location_x,needle_location_y),(40,bar_height)) #((topleft corner),(width,height))
        pygame.draw.rect(self.screenVR,self.cGreen,bar)
        self.screenVR.blit(self.syringe_img,(needle_location_x-25,needle_location_y-110))#80

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
                pygame.display.quit()
                pygame.quit()
                pygame.mixer.quit()
            elif event.type == pygame.KEYUP:
                if event.key == ord('m'):
                    pygame.mouse.set_visible(not pygame.mouse.get_visible())
                elif event.key == ord('q'):
                    self.run = False
                    pygame.display.quit()
                    pygame.quit()
                    pygame.quit()
                elif event.key == ord('r'):
                    self.rotate_up = False
                elif event.key == ord('e'):
                    self.rotate_down = False
                elif event.key == ord('s'):
                    self.success = True
                elif event.key == ord('v'):
                    self.toggle_visual = not self.toggle_visual
                elif event.key == ord('h'):
                    self.haptic_feedback = not self.haptic_feedback
                elif event.key == ord('d'):
                    self.debugToggle = not self.debugToggle
                elif event.key == ord('o'):
                    self.visual_feedback = not self.visual_feedback
                elif event.key == ord(' '):
                    self.render_bar = False
                elif event.key == ord('p'):
                    self.handover_successful = True
            elif event.type == pygame.KEYDOWN:
                if event.key == ord(' '):
                    self.render_bar = True
                elif event.key == ord('r'):
                    self.rotate_up = True
                elif event.key == ord('e'):
                    self.rotate_down = True

    def run_simulation(self):
        self.send_task_status(start=0, end=0, draining_status=0, time=self.max_time)  # reset message
        while self.run:

            self.process_events()  # Keyboard events

            if self.time_start is not None:
                self.time_left = self.max_time - (time.time() - self.time_start)
                if self.time_left < 0:
                    self.time_left = 0

            if self.success:
                self.send_task_status(draining_status=1, time=self.time_left,start=0,end=0)
                self.success = False

            if self.check_termination_conditions():
                self.end_screen()
                break  # Exit the loop after handling termination

            self.update_rotation()
            self.apply_low_pass_filter()
            
            self.xm = np.array(self.cursor)
            self.update_fe()
            self.render_screen()
            
            self.previous_cursor = self.cursor
            self.clock.tick(self.FPS)


    

    def update_rotation(self):
        if self.rotate_up:
            self.alpha += np.deg2rad(0.1)
        if self.rotate_down:
            self.alpha -= np.deg2rad(0.1)
        self.xh = np.array(self.haptic.center)

    def apply_low_pass_filter(self):
        self.cursor = pygame.mouse.get_pos()
        if self.i != 0:
            self.cursor = [
                self.smoothing_factor * self.cursor[0] + (1 - self.smoothing_factor) * self.previous_cursor[0],
                self.smoothing_factor * self.cursor[1] + (1 - self.smoothing_factor) * self.previous_cursor[1]
            ]

    def check_termination_conditions(self):
        if self.spinal_coord_collision or self.task_failed:
            self.render_screen_border(False)
            self.failure_claxon.play()  # Play failure sound
            pygame.time.delay(1000)  # Add delay to make the border visible
            self.time_left = 0
            self.send_task_status(draining_status=-1, time=0)
            self.reset = False
            self.fail_count += 1
            return True
        
        if self.time_left <= 0 and self.start_handover:
            self.render_screen_border(False)
            self.failure_claxon.play()  # Play failure sound
            pygame.time.delay(1000)  # Add delay to make the border visible
            self.time_up = True
            self.time_left = 0
            self.send_task_status(draining_status=-1, time=0)
            self.reset = False
            self.fail_count += 1
            return True
        
        if self.handover_successful:
            self.render_screen_border(True)
            self.success_chime.play()  # Play success sound
            pygame.time.delay(1000)  # Add delay to make the border visible
            self.send_task_status(draining_status=1, time=self.time_left,start=0,end=0)
            self.success_count +=1
            return True

        return False

    def update_fe(self):
        cos_alpha = np.cos(self.alpha)
        sin_alpha = np.sin(self.alpha)
        self.haptic_endpoint = pygame.Rect(self.haptic.center[0] + cos_alpha * 250, self.haptic.center[1] + sin_alpha * 250, 1, 1)

        # Initialize zero endpoint force and reference position based on cursor 
        self.fe = np.zeros(2)
        self.reference_pos = self.cursor

        self.collision_bone = self.check_collision_with_vertebrae(self.haptic_endpoint)

        # Reset collision dict 
        self.collision_dict = {key: False for key in self.collision_dict}
        self.collision_flag = False

        # Check for collisions
        self.collision_any = False
        for value in self.objects_dict:
            if self.haptic_endpoint.colliderect(self.objects_dict[value]):
                self.collision_dict[value] = True
                self.collision_any = True

        for collision, collided in self.collision_dict.items():
            if collision not in self.Bones and collided:
                self.damping = self.variable_dict[collision]['D_TISSUE'] * self.K
                self.cutting_force = self.variable_dict[collision]['tissue_cutting_force']
                self.collision_bool = self.variable_dict[collision]['collision_bool']
                self.update_bool = self.variable_dict[collision]['update_bool']
                self.tissue_index = list(self.variable_dict).index(collision) + 1

        if not any(self.collision_dict.values()):
            self.damping = np.zeros(2)
            self.cutting_force = 0

        # Calculate endpoint velocity and update previous haptic state
        self.endpoint_velocity = (self.xhold - self.xh) / self.FPS
        self.xhold = self.xh


        # Non linear function to modify difficulty of the task
        if self.fail_count != 0:
            helper_modifier = - 0.1 * exp(0.2 * self.fail_count) + 1.1 
            hinder_modifier = (self.success_count/10) * exp(helper_modifier-1)
            difficulty_modifier = min(helper_modifier+hinder_modifier,1)
            difficulty_modifier = max(difficulty_modifier,0.7)
        else:
            difficulty_modifier = 1

        # Implements a sine wave parallel to the needle
        needle_direction = np.array([cos_alpha, sin_alpha])
        faulty_force = needle_direction * 45000 * np.sin(difficulty_modifier * self.t / 6) * difficulty_modifier

        # Calculate force feedback from impedance controller 
        self.fe = (self.K @ (self.xm - self.xh) - (2 * 0.7 * np.sqrt(np.abs(self.K)) @ self.dxh)) #+ faulty_force

        if self.collision_any:
            self.count += 1
            # Compute the perpendicular distance to a line
            distance_from_line = (self.a * (self.xm[0] - cos_alpha * 250) - (self.xm[1] - sin_alpha * 250) + self.b) / np.sqrt(self.a ** 2 + 1)
            tissue_stiffness_matrix = np.diag([750, 750])
            needle_offset_force = tissue_stiffness_matrix @ np.array([sin_alpha, cos_alpha]) * distance_from_line 

            if self.count > 3:
                if self.alpha != 0:
                    self.fe += [-needle_offset_force[0], needle_offset_force[1]]
                else:
                    self.fe += [needle_offset_force[0], needle_offset_force[1]]

            if self.dxh[0] > 0:
                if self.alpha == 0:
                    self.tissue_normal_force_x = (tissue_stiffness_matrix * distance_from_line)[0]
                    frictional_force = self.tissue_normal_force_x * self.kinetic_friction_coefficient * self.tissue_index
                    self.fe[0] += frictional_force
                else:
                    tissue_normal_force = tissue_stiffness_matrix @ np.array([sin_alpha, cos_alpha]) * distance_from_line
                    frictional_force = tissue_normal_force * self.kinetic_friction_coefficient * self.tissue_index
                    # self.fe[0] += frictional_force[1]
                    # self.fe[1] += frictional_force[0]
        else:
            self.fe += np.array([0, 0])

        # Compute damping force acting on endpoint due to viscosity of current tissue layer
        self.fd = -self.damping @ self.endpoint_velocity

        # Apply tissue specific cutting force to needle endpoint (only if needle is moving)
        if self.collision_any and self.dxh[0] > 0:
            cutting_force_x = self.cutting_force * cos_alpha
            cutting_force_y = self.cutting_force * sin_alpha
            self.fe[0] += -cutting_force_x
            self.fe[1] += -cutting_force_y

        if self.haptic_feedback:
            self.ddxh = self.fe
            self.dxh += self.ddxh * self.dt - self.fd

            # State management for bone collision
            if self.collision_bone and self.bone_collision_flag:
                self.phold = self.xm
                self.bone_collision_flag = False
                self.away_from_bone = False

            if not self.collision_bone:
                self.bone_collision_flag = True

            # Set forward velocity to zero if collision with bone is detected
            if not self.away_from_bone:
                self.dxh = np.zeros(2)
                buffer_distance = 20  
               
                if self.xm[0] < self.phold[0] - buffer_distance:
                    self.away_from_bone = True
        

            Bones = {'Vertebrae 1', 'Vertebrae 2', 'Vertebrae 3', 'Vertebrae 4', 'Vertebrae 5', 'Vertebrae 6'}
            for collision, collided in self.collision_dict.items():
                if collision not in Bones and collided:
                    max_tissue_force = self.variable_dict[collision]['max_tissue_force']
                    if collision == 'Spinal cord' and self.i > 120:
                        self.spinal_coord_collision_hit = True
                        self.spinal_coord_collision = True
                    else:
                        self.spinal_coord_collision = False

                    if self.update_bool and self.i > 120:
                        self.variable_dict[collision]['update_bool'] = False

                    if not self.variable_dict[collision]['penetration_bool']:
                        F_pen = (self.K @ (self.xm - self.xh) - (2 * 0.7 * np.sqrt(np.abs(self.K)) @ self.dxh))[0] * cos_alpha
                        if F_pen > max_tissue_force:
                            self.variable_dict[collision]['penetration_bool'] = True

                    if self.xh[0] > self.reference_pos[0]:
                        pass
                    elif not self.variable_dict[collision]['penetration_bool']:
                        self.dxh = np.zeros(2)
        else:
            self.ddxh = self.K @ (self.xm - self.xh) - (2 * 0.7 * np.sqrt(np.abs(self.K)) @ self.dxh)
            self.dxh += self.ddxh * self.dt

        if not any(self.collision_dict.values()):
            for collision in self.collision_dict:
                if collision not in Bones:
                    self.variable_dict[collision]['penetration_bool'] = False
                    self.variable_dict[collision]['update_bool'] = True

        if self.update_prox and self.collision_dict['Skin']:
            begin_pos = self.haptic.center
            end_pos = (self.haptic.center[0] + cos_alpha * 250, self.haptic.center[1] + sin_alpha * 250)
            self.a, self.b = self.compute_line(begin_pos, end_pos)
            self.update_prox = False

        if not any(self.collision_dict.values()):
            self.update_prox = True

        self.xhhold = self.xh
        self.xh = self.dxh * self.dt + self.xh
        self.i += 1
        self.t += self.dt
        self.haptic.center = self.xh

    def render_screen(self):
        def draw_layers(screen, colors, layers, border_radius):
            #pygame.draw.rect(self.screenVR, (0,0,0), pygame.Rect(0, 440, 550, 60),  2)
            for color, layer in zip(colors, layers):
                pygame.draw.rect(screen, color, layer, border_radius=border_radius)

        def draw_vertebrae(screen, layer_image, rects):
            for rect in rects:
                screen.blit(layer_image, (rect[0], rect[1]))

        def display_timer():
            if self.time_start is not None:
                time_color = (255, 0, 0) if self.time_left <= 10 else (0, 0, 0)
                if self.start_handover:
                    text = f"Time left for handover: {self.time_left:.2f}s"
                else:
                    text = f"Time left: {self.time_left:.2f}s"
                text_time = self.font.render(text, True, time_color)
                self.screenVR.blit(text_time, (0, 478 if self.time_left <= 10 else 480))

        def handle_draining():
            if self.collision_dict['Cerebrospinal fluid one'] and self.i > 350 and self.visual_feedback:
                if self.render_bar:
                    self.bar_pressed = True
                    if self.update_draining_start:
                        self.update_draining_start = False
                        self.send_task_status(start=1)
                        if self.time_start is None:
                            self.time_start = time.time()
                    self.fluid -= 1
                    if self.fluid > 0:
                        fluid_left = round(self.fluid / 100, 2)
                        text_surface = self.font.render(f'Fluid left: {fluid_left}ml', True, (0, 0, 0))
                        self.screenVR.blit(text_surface, (0, 60))
                        self.draw_progress_bar(self.fluid)
                    else:
                        self.start_handover = True
                        self.draw_progress_bar(0)
                        self.bar_released_too_soon = False
                    if self.update_draining_start:
                        self.update_draining_start = False
                        self.send_task_status(start=1)
                elif not self.render_bar and not self.start_handover and not self.bar_pressed:
                    space_bar_text = self.font_low_time.render('Press space bar to start draining the fluid!', True, (20, 150, 40))
                    self.screenVR.blit(space_bar_text, (0, 60))

        def handle_handover():
            if self.update_status:
                self.send_task_status(end=1,start=0)
                self.update_status = False

            keep_mouse_in_fluid_texts = [
                "Don't remove the needle from the epidural space until", 
                "the robot has provided the object!",
                "You can release the spacebar now!"
            ]
            y_offset = 60
            for text in keep_mouse_in_fluid_texts:
                rendered_text = self.font_low_time.render(text, True, (20, 150, 40))
                self.screenVR.blit(rendered_text, (0, y_offset))
                y_offset += 20

            if not self.collision_dict['Cerebrospinal fluid one']:
                self.needle_removed_too_soon_2 = True
                self.send_task_status(draining_status=-1, time=self.time_left)
                self.reset = False
                self.task_failed = True

        # Render the haptic surface
        self.screenHaptics.fill(self.cWhite)

        # Change color based on effort
        norm_effort = np.linalg.norm(np.abs(self.K_TISSUE) * (self.xm - self.xh) / self.window_scale) * 15
        clipped_effort = np.clip(norm_effort, 0, 255)
        colorMaster = (255, 255 - clipped_effort, 255 - clipped_effort)

        pygame.draw.line(self.screenHaptics, (0, 0, 0), self.haptic.center, self.haptic.center + 2 * self.K_TISSUE * (self.xm - self.xh))
        pygame.draw.rect(self.screenHaptics, colorMaster, self.haptic, border_radius=4)

        if self.robotToggle:
            self.robot.createPantograph(self.screenHaptics, self.xh)

        # Render the VR surface
        self.screenVR.fill(self.cWhite)

        # Draw all the vertical tissue layers
        tissue_colors = [self.cSkin, self.cFat, self.cLig_one, self.cLig_two, self.cLig_three, self.cFluid,
                        self.cSpinal, self.cFluid, self.cLig_one, self.cLig_two, self.cLig_one]
        tissue_layers = [self.wall_layer1, self.wall_layer2, self.wall_layer3, self.wall_layer4, self.wall_layer5,
                        self.wall_layer6, self.wall_layer7, self.wall_layer8, self.wall_layer9, self.wall_layer10,
                        self.wall_layer11]
        draw_layers(self.screenVR, tissue_colors, tissue_layers, border_radius=2)

        # Draw all the vertebrae
        vertebrae_layers = [self.wall_layer12, self.wall_layer13, self.wall_layer14, self.wall_layer15, self.wall_layer16, self.wall_layer17]
        draw_layers(self.screenVR, [self.cVerte] * len(vertebrae_layers), vertebrae_layers, border_radius=4)

        # Draw vertebrae images
        vertebrae_rects = [self.vert_rect1, self.vert_rect2, self.vert_rect3, self.vert_rect4, self.vert_rect5, self.vert_rect6]
        draw_vertebrae(self.screenVR, self.vertebrae_layer, vertebrae_rects)

        # Draw needle
        x1, y1 = self.haptic.center
        x2 = x1 + np.cos(self.alpha) * 250
        y2 = y1 + np.sin(self.alpha) * 250
        x2_2 = np.sin(-self.alpha) * 25
        y2_2 = np.cos(-self.alpha) * 25

        needle_coords = [(x1, y1), (x2, y2), (x1 + x2_2, y1 + y2_2), (x1 - x2_2, y1 - y2_2)]
        for coord in needle_coords:
            pygame.draw.line(self.screenVR, self.cBlack, (x1, y1), coord, 2)

        # Handles the logic behind the progress bar of draining the fluid
        if self.haptic_feedback:
            handle_draining()


        if self.bar_pressed and not self.start_handover:
            if not self.collision_dict['Cerebrospinal fluid one']:
                self.task_failed = True
                self.needle_removed_too_soon = True

        if self.needle_removed_too_soon and self.needle_removed_too_soon_update:
            self.send_task_status(draining_status = -1,start=0,end=0)
            self.reset = False
            self.needle_removed_too_soon_update = False

        if self.bar_pressed and not self.render_bar and not self.start_handover:
            self.task_failed = True
            self.bar_released_too_soon = True
            self.send_task_status(draining_status=-1,start=0,end=0)
            self.reset = False
        elif self.start_handover:
            self.bar_released_too_soon = False
            

        # Handles the logic of the handover phase
        if self.start_handover:
            handle_handover()

        # Toggle a mask over the spine
        if self.toggle_visual:
            pygame.draw.rect(self.screenVR, self.cSkin, (self.simulation_space[0][0], 0, self.simulation_space[0][1], self.simulation_space[1][1]), border_radius=0)

        if self.i < 350:
            pygame.draw.rect(self.screenVR, self.cWhite, (self.simulation_space[0][0], 0, self.simulation_space[0][1], self.simulation_space[1][1]), border_radius=0)
            draw_layers(self.screenVR, [self.cVerte] * len(vertebrae_layers), vertebrae_layers, border_radius=4)
            draw_vertebrae(self.screenVR, self.vertebrae_layer, vertebrae_rects)

        # Visualize toggles on display
        toggle_texts = [
            "Press 'e' to rotate needle up",
            "Press 'r' to rotate needle down"
        ]
        y_offset = 0
        for text in toggle_texts:
            text_surface = self.font.render(text, True, (0, 0, 0), (255, 255, 255))
            self.screenVR.blit(text_surface, (0, y_offset))
            y_offset += 20

        #self.check_print_text() Disabled one-way communication
        display_timer()
            
        # Fuse it back together
        self.window.blit(self.screenHaptics, (0, 0))
        self.window.blit(self.screenBlank, (800, 0))
        self.window.blit(self.screenVR, (900, 0))

        if self.haptic_feedback and self.visual_feedback and self.collision_dict['Spinal cord']:
            self.spinal_coord_collision_hit = True
            
        pygame.display.flip()

    def render_screen_border(self, status):
        if status:
            GB = min(255, max(0, round(255 * 0.5)))
            self.window.fill((GB, 255, GB), special_flags=pygame.BLEND_MULT)
        else:
            GB = min(255, max(0, round(255 * 0.5)))
            self.window.fill((255, GB, GB), special_flags=pygame.BLEND_MULT)
        
        self.update_screen()  # Update the display after setting the border color


    def update_screen(self):
        pygame.display.flip()
        self.process_events()

    def split_text(self,text, font, max_width):
            words = text.split(' ')
            lines = []
            current_line = words[0]
            for word in words[1:]:
                if font.size(current_line + ' ' + word)[0] <= max_width:
                    current_line += ' ' + word
                else:
                    lines.append(current_line)
                    current_line = word
            lines.append(current_line)
            return lines

    def end_screen(self):
        def render_end_texts():
            return [
                ("Restart Simulation", (0, 0, 0), pygame.font.Font(None, 24)),
                ("You hit the patient's spine! Please try again", (240, 0, 0), pygame.font.Font(None, 38)),
                ("You successfully drained all the fluid from the epidural space, well done!", (20, 150, 40), pygame.font.Font(None, 38)),
                (f"Fails: {self.fail_count}", (0, 0, 0), pygame.font.Font(None, 24)),
                (f"Successes: {self.success_count}", (0, 0, 0), pygame.font.Font(None, 24)),
                (f"You successfully completed: {self.fail_count + self.success_count} simulation runs!", (0, 0, 0), pygame.font.Font(None, 24)),
                ("Press 'q' to exit simulation", (0, 0, 0), pygame.font.Font(None, 24)),
                ("You did not obtain the item in time! Please try again", (240, 0, 0), pygame.font.Font(None, 38)),
                ("Needle removed from epidural space without successfully draining the epidural space! Please try again", (240, 0, 0), pygame.font.Font(None, 38)),
                ("Needle removed from epidural space without successful handover of the item!", (240, 0, 0), pygame.font.Font(None, 38)),
                ("Spacebar was released before all of the fluid was drained, please try again!", (240, 0, 0), pygame.font.Font(None, 38)),
                ("Item obtained! The procedure is successful", (20, 150, 40), pygame.font.Font(None, 38)),
                (f"Score: {round(self.time_left, 2)}", (0, 0, 0), pygame.font.Font(None, 30)),
                ("First run successfully completed! Congratulations",(20, 150, 40), pygame.font.Font(None, 38))
            ]

        
        self.run = True

        button_surface = pygame.Surface((150, 50))
        button_rect = pygame.Rect(0, 0, 150, 50)
        texts = render_end_texts()

        tries = self.fail_count + self.success_count

        flag = True
        while self.run:
            if self.phase == 5 and flag:
                self.flag = False
            self.window.blit(self.screenHaptics, (0, 0))
            self.window.blit(self.screenBlank, (800, 0))
            self.window.blit(self.screenVR, (900, 0))
            self.screenHaptics.fill(self.cWhite)

            for event in pygame.event.get():
                if event.type == pygame.KEYUP:
                    if event.key == ord('q'):
                        if tries < self.max_tries:
                            self.run = False
                            pygame.display.quit()
                            pygame.quit()
                        else:
                            self.run = False

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and tries < self.max_tries and self.reset:
                    if button_rect.collidepoint(event.pos):
                        self.initialise_simulation_parameters()
                        self.run_simulation()

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and tries < self.max_tries and self.phase == 6:
                    if button_rect.collidepoint(event.pos):
                        self.initialise_simulation_parameters()
                        self.initialise_others()
                        self.start_screen()


            if tries < self.max_tries:
                if self.reset or self.phase == 6:
                    if button_rect.collidepoint(pygame.mouse.get_pos()):
                        pygame.draw.rect(button_surface, (220, 220, 220), (1, 1, 148, 48))
                    else:
                        pygame.draw.rect(button_surface, (0, 0, 0), (0, 0, 150, 50))
                        pygame.draw.rect(button_surface, (255, 255, 255), (1, 1, 148, 48))
                        pygame.draw.rect(button_surface, (0, 0, 0), (1, 1, 148, 1), 2)
                        pygame.draw.rect(button_surface, (0, 100, 0), (1, 48, 148, 10), 2)

                    text_surface = texts[0][2].render(texts[0][0], True, texts[0][1])
                    text_rect = text_surface.get_rect(center=(button_surface.get_width() / 2, button_surface.get_height() / 2))
                    button_surface.blit(text_surface, text_rect)
                    self.screenHaptics.blit(button_surface, (button_rect.x, button_rect.y))

                self.screenHaptics.blit(texts[3][2].render(texts[3][0], True, texts[3][1]), (450, 20))
                self.screenHaptics.blit(texts[4][2].render(texts[4][0], True, texts[4][1]), (450, 40))
                self.screenHaptics.blit(texts[12][2].render(texts[12][0], True, texts[12][1]), (450, 70))

                if self.spinal_coord_collision and not self.phase == 6:
                    self.screenHaptics.blit(texts[1][2].render(texts[1][0], True, texts[1][1]), (20, 120))
                elif self.handover_successful == 1 and not self.phase == 6:
                    self.screenHaptics.blit(texts[11][2].render(texts[11][0], True, texts[11][1]), (20, 120))
                elif self.needle_removed_too_soon and not self.phase == 6:
                    split_lines = self.split_text(texts[8][0], texts[8][2], self.screenHaptics.get_width())
                    y_offset = 120
                    for line in split_lines:
                        self.screenHaptics.blit(texts[8][2].render(line, True, texts[8][1]), (20, y_offset))
                        y_offset += 30
                elif self.needle_removed_too_soon_2 and not self.phase == 6:
                    split_lines = self.split_text(texts[9][0], texts[9][2], self.screenHaptics.get_width())
                    y_offset = 120
                    for line in split_lines:
                        self.screenHaptics.blit(texts[9][2].render(line, True, texts[9][1]), (20, y_offset))
                        y_offset += 30
                elif self.bar_released_too_soon and not self.time_up and not self.phase == 6:
                    split_lines = self.split_text(texts[10][0], texts[10][2], self.screenHaptics.get_width())
                    y_offset = 120
                    for line in split_lines:
                        self.screenHaptics.blit(texts[10][2].render(line, True, texts[10][1]), (20, y_offset))
                        y_offset += 30
                elif self.phase == 6:
                    self.screenHaptics.blit(texts[13][2].render(texts[13][0], True, texts[13][1]), (20, 120))
                else:
                    split_lines = self.split_text(texts[7][0], texts[7][2], self.screenHaptics.get_width())
                    y_offset = 120
                    for line in split_lines:
                        self.screenHaptics.blit(texts[7][2].render(line, True, texts[7][1]), (20, y_offset))
                        y_offset += 30
            else:
                self.screenHaptics.blit(texts[5][2].render(texts[5][0], True, texts[5][1]), (20, 120))
                self.screenHaptics.blit(texts[6][2].render(texts[6][0], True, texts[6][1]), (20, 140))

            pygame.display.update()
            self.clock.tick(60) # slow down the loop


if __name__ == '__main__':
    game = secondary_task()