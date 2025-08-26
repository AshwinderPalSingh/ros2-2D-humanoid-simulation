#!/usr/bin/env python3

import pygame
import math
import sys

class WalkingRobot:
    def __init__(self, x=400, y=300):
        self.x = x
        self.y = y
        self.angle = 0
        self.walking = False
        self.walk_cycle = 0
        self.target_x = None
        self.target_y = None
        
        # Robot dimensions
        self.body_width = 40
        self.body_height = 60
        self.head_radius = 15
        self.arm_length = 45
        self.leg_length = 60
        
        # Joint angles for animation
        self.left_leg_angle = 0
        self.right_leg_angle = 0
        self.left_arm_angle = 0
        self.right_arm_angle = 0

    def update(self):
        if self.target_x and self.target_y:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 5:
                self.target_x = None
                self.target_y = None
                self.walking = False
            else:
                self.angle = math.atan2(dy, dx)
                self.walking = True
        
        if self.walking:
            self.walk_cycle += 0.2
            
            # Leg swing
            self.left_leg_angle = math.sin(self.walk_cycle) * 0.5
            self.right_leg_angle = -math.sin(self.walk_cycle) * 0.5
            
            # Arm swing (opposite to legs)
            self.left_arm_angle = -math.sin(self.walk_cycle) * 0.3
            self.right_arm_angle = math.sin(self.walk_cycle) * 0.3
            
            # Move forward
            self.x += math.cos(self.angle) * 2
            self.y += math.sin(self.angle) * 2
        else:
            # Reset to standing pose
            self.left_leg_angle *= 0.9
            self.right_leg_angle *= 0.9
            self.left_arm_angle *= 0.9
            self.right_arm_angle *= 0.9

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y

    def handle_keys(self, keys):
        self.walking = False
        
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.angle = -math.pi/2
            self.walking = True
        elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.angle = math.pi/2
            self.walking = True
        elif keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self.angle = math.pi
            self.walking = True
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self.angle = 0
            self.walking = True

    def draw(self, screen):
        # Body
        body_rect = pygame.Rect(self.x - self.body_width//2, 
                               self.y - self.body_height//2,
                               self.body_width, self.body_height)
        pygame.draw.rect(screen, (0, 100, 200), body_rect)
        
        # Head
        head_y = self.y - self.body_height//2 - self.head_radius
        pygame.draw.circle(screen, (255, 200, 150), (int(self.x), int(head_y)), self.head_radius)
        
        # Arms
        left_arm_x = self.x - self.body_width//2
        left_arm_y = self.y - self.body_height//4
        left_arm_end_x = left_arm_x + math.cos(self.left_arm_angle + math.pi/2) * self.arm_length
        left_arm_end_y = left_arm_y + math.sin(self.left_arm_angle + math.pi/2) * self.arm_length
        
        right_arm_x = self.x + self.body_width//2
        right_arm_y = self.y - self.body_height//4
        right_arm_end_x = right_arm_x + math.cos(self.right_arm_angle + math.pi/2) * self.arm_length
        right_arm_end_y = right_arm_y + math.sin(self.right_arm_angle + math.pi/2) * self.arm_length
        
        pygame.draw.line(screen, (200, 50, 50), (left_arm_x, left_arm_y), 
                        (left_arm_end_x, left_arm_end_y), 8)
        pygame.draw.line(screen, (200, 50, 50), (right_arm_x, right_arm_y), 
                        (right_arm_end_x, right_arm_end_y), 8)
        
        # Legs
        left_leg_x = self.x - self.body_width//4
        left_leg_y = self.y + self.body_height//2
        left_leg_end_x = left_leg_x + math.cos(self.left_leg_angle + math.pi/2) * self.leg_length
        left_leg_end_y = left_leg_y + math.sin(self.left_leg_angle + math.pi/2) * self.leg_length
        
        right_leg_x = self.x + self.body_width//4
        right_leg_y = self.y + self.body_height//2
        right_leg_end_x = right_leg_x + math.cos(self.right_leg_angle + math.pi/2) * self.leg_length
        right_leg_end_y = right_leg_y + math.sin(self.right_leg_angle + math.pi/2) * self.leg_length
        
        pygame.draw.line(screen, (50, 200, 50), (left_leg_x, left_leg_y), 
                        (left_leg_end_x, left_leg_end_y), 10)
        pygame.draw.line(screen, (50, 200, 50), (right_leg_x, right_leg_y), 
                        (right_leg_end_x, right_leg_end_y), 10)
        
        # Feet
        pygame.draw.circle(screen, (50, 50, 50), (int(left_leg_end_x), int(left_leg_end_y)), 8)
        pygame.draw.circle(screen, (50, 50, 50), (int(right_leg_end_x), int(right_leg_end_y)), 8)
        
        # Direction indicator
        indicator_x = self.x + math.cos(self.angle) * 30
        indicator_y = self.y + math.sin(self.angle) * 30
        pygame.draw.line(screen, (255, 255, 0), (self.x, self.y), (indicator_x, indicator_y), 3)
        
        # Target indicator
        if self.target_x and self.target_y:
            pygame.draw.circle(screen, (255, 0, 0), (int(self.target_x), int(self.target_y)), 10, 2)

def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 700))
    pygame.display.set_caption("Walking Humanoid Robot - Keyboard & Mouse Control")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)
    
    robot = WalkingRobot(500, 350)
    
    input_mode = False
    input_text = ""
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    if input_mode:
                        try:
                            coords = input_text.split(',')
                            if len(coords) == 2:
                                x, y = float(coords[0].strip()), float(coords[1].strip())
                                robot.set_target(x, y)
                            input_text = ""
                            input_mode = False
                        except:
                            input_text = "Invalid format!"
                    else:
                        input_mode = True
                        input_text = ""
                elif event.key == pygame.K_ESCAPE:
                    input_mode = False
                    input_text = ""
                    robot.target_x = None
                    robot.target_y = None
                elif input_mode:
                    if event.key == pygame.K_BACKSPACE:
                        input_text = input_text[:-1]
                    else:
                        input_text += event.unicode
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    robot.set_target(*pygame.mouse.get_pos())
        
        # Handle continuous key presses
        keys = pygame.key.get_pressed()
        if not input_mode:
            robot.handle_keys(keys)
        
        robot.update()
        
        # Draw everything
        screen.fill((200, 220, 255))  # Light blue background
        
        # Draw grid
        for x in range(0, 1000, 50):
            pygame.draw.line(screen, (180, 180, 180), (x, 0), (x, 700), 1)
        for y in range(0, 700, 50):
            pygame.draw.line(screen, (180, 180, 180), (0, y), (1000, y), 1)
        
        robot.draw(screen)
        
        # Instructions
        instructions = [
            "CONTROLS:",
            "Arrow Keys / WASD - Walk",
            "Mouse Click - Walk to point",
            "Enter - Input coordinates",
            "ESC - Stop/Cancel",
            "",
            f"Position: ({int(robot.x)}, {int(robot.y)})",
            f"Walking: {robot.walking}",
            f"Target: {robot.target_x, robot.target_y if robot.target_x else 'None'}"
        ]
        
        for i, text in enumerate(instructions):
            color = (50, 50, 50) if text.startswith("CONTROLS") or text.startswith("Position") else (0, 0, 0)
            surface = font.render(text, True, color)
            screen.blit(surface, (10, 10 + i * 25))
        
        # Input field
        if input_mode:
            input_surface = font.render(f"Enter coordinates (x,y): {input_text}", True, (0, 0, 255))
            screen.blit(input_surface, (10, 650))
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
