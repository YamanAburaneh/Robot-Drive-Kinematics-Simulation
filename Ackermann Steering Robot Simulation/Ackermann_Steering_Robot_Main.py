# Rear Wheel Drive Ackermann Steering Robot Simulation 
# Autonomous Robotics Assignment 2
# Yaman H.S. Aburaneh - 1720907


import pygame
from math import *


class Envir:
    def __init__(self, dimensions):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)
        # map dimensions
        self.height = dimensions[0]
        self.width = dimensions[1]
        # window settings
        pygame.display.set_caption("Rear Wheel Drive Ackermann Steering Robot Simulation")
        self.map = pygame.display.set_mode((self.width, self.height))
        # text variables
        self.font = pygame.font.Font("freesans.ttf", 50)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (400, dimensions[0] - 100)
        self.font1 = pygame.font.Font("freesans.ttf", 20)
        self.text1 = self.font1.render('default', True, self.white, self.black)
        self.textRect1 = self.text1.get_rect()
        self.textRect1.center = (600, dimensions[0] - 35)
        self.font2 = pygame.font.Font("freesans.ttf", 20)
        self.text2 = self.font2.render('default', True, self.white, self.black)
        self.textRect2 = self.text2.get_rect()
        self.textRect2.center = (dimensions[1]-650, 50)


    def write_info(self, V, alpha, theta):
        txt = f"Velocity = {V}m/s  Steering Angle = {alpha}  theta = {theta}  T = {int(pygame.time.get_ticks()/1000)}s"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        txt1= f"Grid Size = 1m  Axles Distance, L = 1m  Axle Width, W = 0.5m  Matric: 1720907"
        self.text1 = self.font1.render(txt1, True, self.white, self.black)
        self.map.blit(self.text1, self.textRect1)
        txt2= f"Left: Steer left    Right: Steer right    Up: Accelerate    Down: Deccelerate"
        self.text2 = self.font2.render(txt2, True, self.white, self.black)
        self.map.blit(self.text2, self.textRect2)

    def axis(self, Zero):
        self.zero_x = Zero[0]
        self.zero_y = Zero[1]
        gridSize = 100
        pygame.draw.line(self.map, self.white, (self.zero_x, self.zero_y), (self.width, self.zero_y), 1) # X axis
        pygame.draw.line(self.map, self.white, (self.zero_x, self.height), (self.zero_x, 0), 1) # Y axis
        for i in range(3):
            pygame.draw.line(self.map, self.white, (self.zero_x-10, self.zero_y+gridSize*i), (self.zero_x+10, self.zero_y+gridSize*i), 1)
            pygame.draw.line(self.map, self.white, (self.zero_x-10, self.zero_y-gridSize*i), (self.zero_x+10, self.zero_y-gridSize*i), 1)
        for i in range(20):
            pygame.draw.line(self.map, self.white, (self.zero_x+gridSize*i, self.zero_y-10), (self.zero_x+gridSize*i, self.zero_y+10), 1)
            pygame.draw.line(self.map, self.white, (self.zero_x-gridSize*i, self.zero_y-10), (self.zero_x-gridSize*i, self.zero_y+10), 1)

 
class robot:
    def __init__(self, startpos, width):
    # robot dimensions
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
    # robot attributes
        self.theta = 0
        self.alpha = 0
        self.orientation = 0.0
        self.steering_angle = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.turn = radians(45.0)
        self.velocity = 3.0
        self.centerx = 0
        self.centery = 0
        self.trail_set = []

    def move(self, map, direction, event=None):

        self.turn *= direction
        theta = self.orientation # initial orientation
        alpha = self.turn # steering angle
        dist = self.velocity # distance to be moved
        length = robot_length # length of the robot

        # in local coordinates of robot
        beta = (dist/length)*tan(alpha) # turning angle
        
        # Steering Kinematics
        _x = _y = _theta = 0.0
        if beta > 0.001 or beta < -0.001:
            radius = dist/beta
            radius = dist/beta # turning radius
            self.cx = self.x - sin(theta)*radius # center of the circle x
            self.cy = self.y - cos(theta)*radius # center of the circle y

            # center of the circle the robot is going about
            pygame.draw.circle(map, red, (int(self.cx), int(self.cy)), 2)
            pygame.draw.circle(map, red, (int(self.cx), int(self.cy)), abs(radius), 2)
            # in global coordinates of robot
            _x = self.cx + sin(theta + beta)*radius
            _y = self.cy + cos(theta + beta)*radius
            _theta = (theta + beta)%(2*pi)

        else: # straight motion
            _x = self.x + dist*cos(theta)
            _y = self.y - dist*sin(theta)
            _theta = (theta + beta)%(2*pi)

        self.x = _x
        self.y = _y
        self.orientation = _theta
        self.steering_angle = alpha

        self.x %= world_size
        self.y %= world_size

    def draw_robot(self, map):
        car_x = self.x
        car_y = self.y
        orientation = self.orientation
        steering_angle = self.steering_angle
        cx = self.cx
        cy = self.cy
        turn = self.turn
        offsetx = 11
        offsety = 2

        p1 = [car_x-robot_length/4,car_y-robot_width/2]
        p2 = [car_x+(0.75*robot_length),car_y-robot_width/2]
        p3 = [car_x+(0.75*robot_length),car_y+robot_width/2]
        p4 = [car_x-robot_length/4,car_y+robot_width/2]

    # Robot Body
        draw_rect([car_x, car_y], [p1, p2, p3, p4], orientation, blue)

    # Wheel 1
        w1_c_x = (car_x) - offsetx
        w1_c_y = (car_y - robot_width/3) - offsety
        length = sqrt((w1_c_x - car_x)**2 + (car_y - w1_c_y)**2)
        angle = atan2(car_y - w1_c_y, w1_c_x - car_x)
        angle += orientation
        w1_c_x = car_x + length*cos(angle)
        w1_c_y = car_y - length*sin(angle)
        w1_p1 = [w1_c_x-wheel_length/2, w1_c_y-wheel_width/2]
        w1_p2 = [w1_c_x+wheel_length/2, w1_c_y-wheel_width/2]
        w1_p3 = [w1_c_x+wheel_length/2, w1_c_y+wheel_width/2]
        w1_p4 = [w1_c_x-wheel_length/2, w1_c_y+wheel_width/2]
        draw_rect([w1_c_x, w1_c_y], [w1_p1, w1_p2, w1_p3, w1_p4], orientation, purple)

    # Wheel 2
        w2_c_x = (car_x + robot_length/2) + offsetx
        w2_c_y = (car_y - robot_width/3 ) - offsety
        length = sqrt((w2_c_x - car_x)**2 + (car_y - w2_c_y)**2)
        angle = atan2(car_y - w2_c_y, w2_c_x - car_x)
        angle += orientation
        w2_c_x = car_x + length*cos(angle)
        w2_c_y = car_y - length*sin(angle)
        w2_p1 = [w2_c_x-wheel_length/2, w2_c_y-wheel_width/2]
        w2_p2 = [w2_c_x+wheel_length/2, w2_c_y-wheel_width/2]
        w2_p3 = [w2_c_x+wheel_length/2, w2_c_y+wheel_width/2]
        w2_p4 = [w2_c_x-wheel_length/2, w2_c_y+wheel_width/2]
        draw_rect([w2_c_x, w2_c_y], [w2_p1, w2_p2, w2_p3, w2_p4], steering_angle + orientation, purple)

    # Wheel 3
        w3_c_x = (car_x + robot_length/2) + offsetx
        w3_c_y = (car_y + robot_width/3) + offsety
        length = sqrt((w3_c_x - car_x)**2 + (car_y - w3_c_y)**2)
        angle = atan2(car_y - w3_c_y, w3_c_x - car_x)
        angle += orientation
        w3_c_x = car_x + length*cos(angle)
        w3_c_y = car_y - length*sin(angle)
        w3_p1 = [w3_c_x-wheel_length/2, w3_c_y-wheel_width/2]
        w3_p2 = [w3_c_x+wheel_length/2, w3_c_y-wheel_width/2]
        w3_p3 = [w3_c_x+wheel_length/2, w3_c_y+wheel_width/2]
        w3_p4 = [w3_c_x-wheel_length/2, w3_c_y+wheel_width/2]
        draw_rect([w3_c_x, w3_c_y], [w3_p1, w3_p2, w3_p3, w3_p4], steering_angle + orientation, purple)

    # Wheel 4
        w4_c_x = (car_x) - offsetx
        w4_c_y = (car_y + robot_width/3) + offsety
        length = sqrt((w4_c_x - car_x)**2 + (car_y - w4_c_y)**2)
        angle = atan2(car_y - w4_c_y, w4_c_x - car_x)
        angle += orientation
        w4_c_x = car_x + length*cos(angle)
        w4_c_y = car_y - length*sin(angle)
        w4_p1 = [w4_c_x-wheel_length/2, w4_c_y-wheel_width/2]
        w4_p2 = [w4_c_x+wheel_length/2, w4_c_y-wheel_width/2]
        w4_p3 = [w4_c_x+wheel_length/2, w4_c_y+wheel_width/2]
        w4_p4 = [w4_c_x-wheel_length/2, w4_c_y+wheel_width/2]
        draw_rect([w4_c_x, w4_c_y], [w4_p1, w4_p2, w4_p3, w4_p4], orientation, purple)

        # draw axle
        pygame.draw.line(map, black, (w1_c_x, w1_c_y),(w4_c_x, w4_c_y), 1)
        pygame.draw.line(map, black, (w2_c_x, w2_c_y),(w3_c_x, w3_c_y), 1)
        # Axle rotation lines
        if turn != 0:
            pygame.draw.line(map, red, (int((w1_c_x+w4_c_x)/2), int((w1_c_y+w4_c_y)/2)),(int(cx), int(cy)), 1)
            pygame.draw.line(map, red, (w3_c_x, w3_c_y), (int(cx), int(cy)), 1)
            pygame.draw.line(map, red, (w2_c_x, w2_c_y), (int(cx), int(cy)), 1)

        # draw mid of axle
        self.centerx = (w1_c_x+w4_c_x)/2
        self.centery = (w1_c_y+w4_c_y)/2
        pygame.draw.circle(map, red, (int((w1_c_x+w4_c_x)/2), int((w1_c_y+w4_c_y)/2)), 3)

    def robot_frame(self, rotation, map):
        n=50
        m = 120
        center_x = self.centerx
        center_y = self.centery
        x_axis=(center_x + m*cos(-rotation),center_y + m*sin(-rotation))
        y_axis=(center_x + n*cos(-rotation+pi/2),center_y + n*sin(-rotation+pi/2))

        pygame.draw.line(map, red,(center_x,center_y), x_axis, 3)
        pygame.draw.line(map, green,(center_x,center_y), y_axis, 3)
    
    def trail(self, map):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, yellow, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]))
        self.trail_set.append((self.centerx, self.centery))

def draw_rect(center, corners, rotation_angle, color):
    c_x = center[0]
    c_y = center[1]
    delta_angle = rotation_angle
    rotated_corners = []

    for p in corners:
        temp = []
        length = sqrt((p[0] - c_x)**2 + (c_y - p[1])**2)
        angle = atan2(c_y - p[1], p[0] - c_x)
        angle += delta_angle
        temp.append(c_x + length*cos(angle))
        temp.append(c_y - length*sin(angle))
        rotated_corners.append(temp)

    # draw rectangular polygon --> car body
    rect = pygame.draw.polygon(environment.map, color, (rotated_corners[0],rotated_corners[1],rotated_corners[2],rotated_corners[3]))

# colors
black = (0, 0, 0)
red = (200, 0, 0)
blue = (0, 0, 255)
purple = (51, 0, 102)
green = (0, 255, 0)
yellow = (255, 255, 0)

# Initialization
pygame.init()

# dimensions
dims = (1000, 1860)
world_size = dims[1]

# start Position
start = (100, 500)

# running variable
running = True

# the Environment
environment = Envir(dims)

# the robot
robot = robot(start, 0.01 * 3779.52)
wheel_length = 24.0
wheel_width = 8.0
robot_length = 100.0
robot_width = 50.0
velocity = 0.005 * 3779.52

# Timer variables
clock = pygame.time.Clock()
currentTime = 0
lastTime = 0
direction = 1

# Simulation loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    currentTime = pygame.time.get_ticks()
    if currentTime - lastTime > 750:
        direction = direction * (-1)
        lastTime = pygame.time.get_ticks()
    else:
        direction = 1
    
    pygame.display.update()
    clock.tick(60)
    environment.map.fill(environment.black)
    robot.move(environment.map, direction)
    robot.robot_frame(robot.orientation, environment.map)
    environment.write_info(round(float(robot.velocity),2), degrees(robot.turn)*-1, int(degrees(robot.orientation)))
    robot.draw_robot(environment.map)
    robot.trail(environment.map)
    environment.axis(start)