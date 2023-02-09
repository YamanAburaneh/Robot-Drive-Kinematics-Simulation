# Differential Drive Robot Simulation 
# Autonomous Robotics Assignment 2
# Yaman H.S. Aburaneh - 1720907


import math
import pygame


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
        pygame.display.set_caption("Differential Drive Robot Simulation")
        self.map = pygame.display.set_mode((self.width, self.height))
    # text variables
        self.font = pygame.font.Font("FreeSans.ttf", 50)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimensions[1] - 1100, dimensions[0] - 100)
        self.font1 = pygame.font.Font("FreeSans.ttf", 20)
        self.text1 = self.font1.render('default', True, self.white, self.black)
        self.textRect1 = self.text.get_rect()
        self.textRect1.center = (dimensions[1] - 1100, dimensions[0] - 35)
    # trail
        self.trail_set = []

    def write_info(self, Vl, Vr, theta):
        scalefactor=0.02653846
        txt = f"Vl = {round(Vl*scalefactor,2)} m/s  Vr = {round(Vr*scalefactor,2)} m/s  Theta = {int(math.degrees(theta))}  T = {int(pygame.time.get_ticks()/1000)}s"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        txt1= f"Grid Size = 1m  Axles Distance, L = 1m  Axle Width, W = 0.5m  Matric: 1720907"
        self.text1 = self.font1.render(txt1, True, self.white, self.black)
        self.map.blit(self.text1, self.textRect1)

    def trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.yellow, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]))
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n=80
        center_x, center_y = pos
        x_axis=(center_x + n*math.cos(-rotation),center_y + n*math.sin(-rotation))
        y_axis=(center_x + n*math.cos(-rotation+math.pi/2),center_y + n*math.sin(-rotation+math.pi/2))
        pygame.draw.line(self.map, self.red,(center_x,center_y), x_axis, 3)
        pygame.draw.line(self.map, self.green,(center_x,center_y), y_axis, 3)

    def axis(self, Zero):
        self.zero_x = Zero[0]
        self.zero_y = Zero[1]
        gridSize = 100
        pygame.draw.line(self.map, self.white, (0, self.zero_y), (self.width, self.zero_y), 1) # X axis
        pygame.draw.line(self.map, self.white, (self.zero_x, self.height-180), (self.zero_x, 0), 1) # Y axis
        for i in range(3):
            pygame.draw.line(self.map, self.white, (self.zero_x-10, self.zero_y+gridSize*i), (self.zero_x+10, self.zero_y+gridSize*i), 1)
            pygame.draw.line(self.map, self.white, (self.zero_x-10, self.zero_y-gridSize*i), (self.zero_x+10, self.zero_y-gridSize*i), 1)
        for i in range(6):
            pygame.draw.line(self.map, self.white, (self.zero_x+gridSize*i, self.zero_y-10), (self.zero_x+gridSize*i, self.zero_y+10), 1)
            pygame.draw.line(self.map, self.white, (self.zero_x-gridSize*i, self.zero_y-10), (self.zero_x-gridSize*i, self.zero_y+10), 1)

class Robot:
    def __init__(self, startpos, width, velocity):
    # Velocity
        m2p = 37.7952 # meters to pixels
        self.v = velocity * m2p # velocity = 3 m/s
    # robot dimensions
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0
        self.vl = self.v + (0.15*self.v) # 3.45 m/s
        self.vr = self.v - (0.15*self.v) # 2.55 m/s
        self.maxspeed =  2 * self.v
        self.minspeed = -2 * self.v
    # graphics
        self.img = pygame.image.load("Robot_img.png")
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def move(self, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.vl += 0.1 * self.v
                elif event.key == pygame.K_a:
                    self.vl -= 0.1 * self.v
                elif event.key == pygame.K_e:
                    self.vr += 0.1 * self.v
                elif event.key == pygame.K_d:
                    self.vr -= 0.1 * self.v
                elif event.key == pygame.K_w:
                    self.vf = abs((self.vl+self.vr)/2)
                    self.vl = self.vf
                    self.vr = self.vf
                elif event.key == pygame.K_s:
                    self.vl = 0
                    self.vr = 0
    # Differential Drive Kinematics
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.theta) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.theta) * dt
        self.theta += (self.vr - self.vl) / self.w * dt
    # reset theta
        if self.theta>2*math.pi or self.theta<-2*math.pi:
            self.theta=0
    # set max speed
        self.vr=min(self.vr, self.maxspeed)
        self.vl=min(self.vl, self.maxspeed)
    # set min speed
        self.vr=max(self.vr, self.minspeed)
        self.vl=max(self.vl, self.minspeed)

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))


# Initialization
pygame.init()

# dimensions
dims = (640, 1200)

# start Position
start = (dims[1]/2, 200)

# running variable
running = True

# the Environment
environment = Envir(dims)

# velocity
velocity = 3 # m/s  (Walking speed of human = 1m/s)

# the robot
robot = Robot(start, 0.01 * 3779.52, velocity)

# dt
dt = 0
lastTime = pygame.time.get_ticks()

# Simulation loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        robot.move(event)

    dt = (pygame.time.get_ticks() - lastTime) / 1000
    lastTime = pygame.time.get_ticks()
    pygame.display.update()
    environment.map.fill(environment.black)
    robot.move()
    environment.robot_frame((robot.x, robot.y), robot.theta)
    environment.write_info(float(robot.vl), float(robot.vr), robot.theta)
    robot.draw(environment.map)
    environment.trail((robot.x, robot.y))
    environment.axis(start)
