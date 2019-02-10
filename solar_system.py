import bpy
from bpy import context
#import sys
import numpy as np

class Planet:
    def __init__(self, name, radius, mass, pos, vel):
        self.name = name
        self.radius = radius
        self.mass = mass
        self.pos = pos
        self.vel = vel
        self.acc = np.array([0.0, 0.0, 0.0])

G  = 6.674E-17 # Newton * Kmeter^2 / Kg^2
dt = 800.0
       
sun     = Planet(name = "Object_Sun", \
                 radius = 30*696342.00, \
                 mass = 1.98910E30, \
                 pos = np.array([0.0, 0.0, 0.0]), \
                 vel = np.array([0.0, 25*00000.0/1000, 0.0]))
                 
earth   = Planet(name = "Object_Earth", \
                 radius = 300*006371.00, \
                 mass = 5.97219E24, \
                 pos  = np.array([0152098232.0, 0.0, 0.0]), \
                 vel = np.array([0.0, 25*29300.0/1000, 0.0]))


planets = [sun, earth]     
N = len(planets)

planet_force = np.zeros([N, N])     # magnitude of the force between 2 planets (every pair)
planet_vector = np.zeros([N, N, 3]) # vector of the direction between 2 planets (every pair)


def calculate_forces():
    for i in range(N):
        for j in range(i):
            # force exerted from planet j
            planet_force[i][j] = (G * planets[i].mass * planets[j].mass) / np.linalg.norm(planets[i].pos - planets[j].pos)**2
            planet_vector[i][j] = planets[j].pos - planets[i].pos
            planet_vector[i][j] =  planet_vector[i][j] / np.linalg.norm(planet_vector[i][j])        
            
            # symmetrical
            planet_force[j][i]  =  planet_force[i][j]
            planet_vector[j][i] = -planet_vector[i][j] 
   

def update_pos_vel():
    for i, planet_i in enumerate(planets):
        force_total = np.zeros(3)
        for j, planet_j in enumerate(planets):
            force_total += planet_force[i][j] * planet_vector[i][j]
        
        planet_i.acc  = force_total / planet_i.mass
        planet_i.vel += planet_i.acc * dt
        planet_i.pos += 0.5 * planet_i.acc * dt**2 + planet_i.vel* dt

scale = 0.00000005
frame = 1;
n_frames = 270;

for t in range(1,n_frames):
        
    bpy.context.scene.frame_set(frame)
    frame += 1;
    
    calculate_forces()
    update_pos_vel()
        
    for planet in planets:
        planetObject = bpy.data.objects[planet.name]
        planetObject.select = True
        planetObject.location = [scale*planet.pos[0], scale*planet.pos[1], scale*planet.pos[2]]
        planetObject.rotation_euler.z = t/80

    bpy.ops.anim.keyframe_insert(type='LocRotScale', confirm_success=True);