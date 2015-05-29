import cv2, cv 
import numpy as np
import math
import random
import time

# Various parameters of the simulation

# Wind
MAX_ANGLE = np.pi / 20. # radians
MAX_ANGLE_CHANGE = MAX_ANGLE / 10.
WIND_FACTOR = 20 # how many timesteps to change the wind
MAX_SPEED_CHANGE = .1 # m/s
MAX_SPEED = 1.1
MIN_SPEED = .7

# Particles
PARTICLE_SIZE = 0.5 # meters
MAX_PARTICLES = 50 # stop spawning after get to this many
PARTICLE_LIFETIME = 10 # seconds after which to delete particle
WAIT_BEFORE_PARTICLE = .85 # seconds

# Simulation
DELTA_T = 0.01 # seconds
SIZE_FACTOR = 25 # how much bigger to display things
DISTRIBUTION_DISTANCE = 1 # meters -- radius of circle in which particles appear
START_X = 2 # meters
START_Y = 15 # meters

# Class representing full odour plume, containing many particles and various
# methods for dealing with them.
class Odour_Plume:
    def __init__(self):
        self.wind_r = 0.8 # m/s
        self.wind_t = 0   # rad
        self.particles = [] # List of particles comprising the plume
        self.x = START_X
        self.y = START_Y

    def change_wind(self):
        # angle change
        delta = random.random() * MAX_ANGLE_CHANGE
        if(self.wind_t + delta > MAX_ANGLE):
            self.wind_t = self.wind_t - delta
        elif(self.wind_t - delta < -MAX_ANGLE):
            self.wind_t = self.wind_t + delta
        else:
            self.wind_t = self.wind_t + cmp(random.random() - 0.5, 0.)
        
        # speed change
        delta_s = random.random() * MAX_SPEED_CHANGE
        sign_r = random.choice([-1.0, 1.0])
        if(self.wind_r + delta_s*sign_r < MAX_SPEED) and (self.wind_r + delta_s*sign_r > MIN_SPEED):
            self.wind_r = self.wind_r + delta_s*sign_r           
        
        # Inform the child particles
        for particle in self.particles:
            particle.update_speed(self.wind_r, self.wind_t)

    def update(self):
        for particle in self.particles:
            particle.update_position()
        # Remove all old particles
        self.particles[:] = [p for p in self.particles if p.age <= PARTICLE_LIFETIME]

    def spawn_particle(self):
        if self.num_particles() >= MAX_PARTICLES:
            return
        particle = Odour_Particle(PARTICLE_SIZE, 0, 0, 
                self.x + (random.random() - 0.5) * 2 * DISTRIBUTION_DISTANCE,
                self.y + (random.random() - 0.5) * 2 * DISTRIBUTION_DISTANCE)
        particle.update_speed(self.wind_r, self.wind_t)
        self.particles.append(particle)

    def draw(self, img):
        for particle in self.particles:
            particle.draw(img)

    def num_particles(self):
       return len(self.particles)

# The class representing a single odour particle.
# Contains position and speed, various methods.
class Odour_Particle:
    def __init__(self, size, x_speed, y_speed, x, y):
        self.size = size
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.x = x
        self.y = y
        self.age = 0

    def update_speed(self, wind_r, wind_t):
        self.x_speed = wind_r * math.cos(wind_t)
        self.y_speed = wind_r * math.sin(wind_t)

    def update_position(self):
        self.x = self.x + self.x_speed * DELTA_T
        self.y = self.y + self.y_speed * DELTA_T
        self.age = self.age + DELTA_T
        

    def draw(self, img):
        x_pos = (int) (self.x * SIZE_FACTOR)
        y_pos = (int) (self.y * SIZE_FACTOR)
        rad = (int) (self.size * SIZE_FACTOR)
        cv2.circle(img, (x_pos, y_pos), rad, (255, 0, 0), 2)
        #cv2.circle(img2, tuple([int(i) for i in cont_mean]), 5, (255, 0,0))


# Start program
plume = Odour_Plume()

# Main loop
while(1):
    img = np.ones((700, 600, 3), np.uint8) * 255
    plume.spawn_particle();
    plume.draw(img);
    #print "\n"
    #time.sleep(WAIT_BEFORE_PARTICLE)
    cv2.circle(img, (START_X * SIZE_FACTOR,  START_Y * SIZE_FACTOR), 
            DISTRIBUTION_DISTANCE * SIZE_FACTOR, (0, 0, 255), 3)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.imshow('image', img)
    #cv2.waitKey((int)(1000 * WAIT_BEFORE_PARTICLE))
    cv2.waitKey(1)
    i = 0
    while i < WAIT_BEFORE_PARTICLE / DELTA_T:
        img = np.ones((700,600,3), np.uint8) * 255
        plume.update()
        plume.draw(img)
        cv2.circle(img, (START_X * SIZE_FACTOR,  START_Y * SIZE_FACTOR), 
                DISTRIBUTION_DISTANCE * SIZE_FACTOR, (0, 0, 255), 3)
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', img)
        #cv2.waitKey((int)(1000 * WAIT_BEFORE_PARTICLE))
        cv2.waitKey(1)
        if i % WIND_FACTOR == 0:
            plume.change_wind()
        time.sleep(DELTA_T)
        i = i + 1