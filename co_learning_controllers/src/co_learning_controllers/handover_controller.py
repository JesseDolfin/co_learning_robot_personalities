#!/usr/bin/env python3
import pygame
import rospy
from co_learning_messages.msg import secondary_task_message
from std_msgs.msg import Header

class HandoverStatusGUI:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('handover_status_gui', anonymous=True)
        self.pub = rospy.Publisher('/Task_status', secondary_task_message, queue_size=10)
        self.sub = rospy.Subscriber('/Task_status', secondary_task_message, self.message_callback)
        
        # Store the latest message
        self.current_msg = secondary_task_message()
        
        # Initialize PyGame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption('Handover Status GUI')
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        
        # Font
        self.font = pygame.font.Font(None, 36)
        
        # Status variable
        self.handover_status = 0
        
    def message_callback(self, msg):
        self.current_msg = msg
        # Reset handover status if reset is True
        if msg.reset:
            self.handover_status = 0
        
    def publish_message(self):
        # Create new message with same content as current message
        msg = self.current_msg
        
        # Only update the handover_successful field
        msg.handover_successful = self.handover_status
        
        self.pub.publish(msg)
        
    def draw_status(self):
        self.screen.fill(self.WHITE)
        
        # Draw instructions
        instructions = [
            "Press 'S' for Success (+1)",
            "Press 'F' for Fail (-1)",
            "Press 'Q' to Quit"
        ]
        
        for i, text in enumerate(instructions):
            text_surface = self.font.render(text, True, self.BLACK)
            self.screen.blit(text_surface, (20, 20 + i * 40))
        
        # Draw current status
        status_text = f"Handover Status: {self.handover_status}"
        status_color = self.GREEN if self.handover_status == 1 else self.RED if self.handover_status == -1 else self.BLACK
        status_surface = self.font.render(status_text, True, status_color)
        self.screen.blit(status_surface, (20, 160))
        
        # Draw reset status from current message
        reset_text = f"Reset Status: {self.current_msg.reset}"
        reset_color = self.BLUE if self.current_msg.reset else self.BLACK
        reset_surface = self.font.render(reset_text, True, reset_color)
        self.screen.blit(reset_surface, (20, 200))
        
        pygame.display.flip()
        
    def run(self):
        running = True
        clock = pygame.time.Clock()
        
        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s:  # Success
                        self.handover_status = 1
                        self.publish_message()
                        
                    elif event.key == pygame.K_f:  # Fail
                        self.handover_status = -1
                        self.publish_message()
                        
                    elif event.key == pygame.K_q:  # Quit
                        running = False
            
            self.draw_status()
            clock.tick(30)
            
        pygame.quit()

if __name__ == '__main__':
    try:
        gui = HandoverStatusGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass