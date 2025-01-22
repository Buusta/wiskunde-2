# -*- coding: utf-8 -*-
"""
Created on Mon Jul  3 14:33:49 2023

@author: Rik
"""

import wis_2_2_utilities as util
import wis_2_2_systems as systems
from scipy.signal import place_poles
#import random
import numpy as np
import sys

#set timestep
timestep = 5e-4

state0 = 300

# calculate moment of intertia
r1 = 0.6
a1 = 0.02**2
m1 = r1 * a1 * 700

r2 = 0.1
a2 = np.pi * r2**2
h1 = 0.01
m2 = a2 * h1 * 7870

I1 = (1/3) * m1 * r1**2
I2 = 1/2 * m2 * r2**2

g = -9.81



a = m1 * r1**2 + 4 * m2 * r1**2 + I1
b = (m1 + I2) * g * r1

# state pace matrix A
A = np.array([[0, 1, 0, 0],
              [-b/(a+I2), 0, 0, -I2 / (a+I2)],
              [0, 0, 0, 1],
              [0, 0, -1/I2, 0]])

# state space matrix A
B = np.array([[0],
              [0],
              [0],
              [1/I2]])

C = np.array([[0, 1, 0, 0],    # meten van de hoek θ
              [0, 0, 0, 1]])   # meten van de hoek φ



# n = A.shape[0]  # System state dimension
# observability_matrix = np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])
# rank = np.linalg.matrix_rank(observability_matrix)

# print("Rank of observability matrix:", rank)
# print("System state dimension:", n)

desired_poles = [-4, .6, 0, 8000]  # Keuze van gewenste eigenwaarden voor de observer
P = place_poles(A.T, C.T, desired_poles)  # Pole placement voor observer
L = P.gain_matrix

class PID_controller():
  def __init__(self, target=0):
    self.integral1=0
    self.integral2=0
    self.K_P1=0
    self.K_I1=0
    self.K_D1=0
    self.K_P2=0
    self.K_I2=0
    self.K_D2=0

  def feedBack(self, observe):
    self.integral1+=observe[0]
    self.integral2+=observe[2]
    u=self.K_P1*observe[0]+\
      self.K_I1*self.integral1+\
      self.K_D1*observe[1]+\
      self.K_P2*observe[2]+\
      self.K_I2*self.integral2+\
      self.K_D2*observe[3]
    return u

class pp_controller():
  def __init__(self, target=0):
    self.matrix_gain=L

  def feedBack(self, observe):
    u= -self.matrix_gain @ observe
    return u

class controller():
  def __init__(self, target=0):
    pass

  def feedBack(self, observe):
    u=0
    return u


def main():
  model=systems.flywheel_inverted_pendulum(second_pendulum = False, high_kick=state0)
  control = pp_controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  #simulation.max_duration = 600 #seconde
  simulation.GIF_toggle = False #set to false to avoid frame and GIF creation

  while simulation.vis.Run():
      if simulation.time<simulation.max_duration:
        simulation.step()
        u = control.feedBack(simulation.observe())
        sys.stdout.write('\r'+ str(u[0]) + str(u[1]))
        sys.stdout.flush()
        simulation.control(u)
        # simulation.log()
        # simulation.refreshTime()
      else:
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()

  simulation.writeData()






if __name__ == "__main__":
  main()