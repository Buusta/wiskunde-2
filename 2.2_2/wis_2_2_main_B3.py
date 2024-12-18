import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np
from scipy.signal import place_poles
from numpy.linalg import matrix_rank

#set timestep
timestep = 2e-4


#params
M = 11.2
m = 0.168
l = 0.6
g = -9.81



A = np.array([[0, 1, 0, 0],
              [0, 0, (m*g) / (M-m), 0],
              [0, 0, 0, 1],
              [0, 0, (g*(M+m))/(l*M), 0]])

B = np.array([[0], 
              [1/(M+m) + (m*l)/(M*(M+m))],
              [0],
              [1/M]])


C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

D = [0, 0]

poles = [-1.6, 300, -1.21, -2.5] 


B = 1 * B #scale B


place_result = place_poles(A, B, poles)
K = place_result.gain_matrix



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
    self.matrix_gain=K
    
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
  score = 0
  balance_time = 0
  model=systems.cart_inverted_pendulum()
  control = pp_controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  simulation.max_duration = 3 #seconde
  simulation.GIF_toggle = False #set to false to avoid frame and GIF creation

  while simulation.vis.Run():
      if simulation.time<simulation.max_duration:
        simulation.step()
        u = control.feedBack(simulation.observe())
        simulation.control(u)
        # simulation.log()
        # simulation.refreshTime()
        score += u**2
        
        x_position = simulation.observe()[0]
        if abs(x_position) > 5e-3:
            balance_time = 0  # Reset balance time if the cart moves away from 0
        elif abs(x_position) < 5e-3 and simulation.time > .1 and balance_time == 0:
            balance_time = simulation.time
            print(simulation.observe())
        
      else:
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()
        
  print(round(balance_time, 3))
  print(round(int(score)))
  return score
  simulation.writeData()

if __name__ == "__main__":
    main()
