import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np
from time import sleep

#set timestep
timestep = 2e-3
score = 0




class PID_controller():
  def __init__(self, target=0):
    self.integral1=0
    self.integral2=0
    self.K_P1=-6.5
    self.K_I1=0
    self.K_D1=-4.5
    self.K_P2=-40
    self.K_I2=0
    self.K_D2=-10
    
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
    self.matrix_gain=np.array([[0, 0, 0, 0]])
    
  def feedBack(self, observe):
    u= -self.matrix_gain @ observe
    return u  
  
class controller():
  def __init__(self, target=0):
    pass
    
  def feedBack(self, observe):
    u=0
    return u


def main(kick):
  model=systems.rotation_inverted_pendulum(high_kick = kick)
  control = PID_controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  simulation.max_duration = 3 #seconde
  simulation.GIF_toggle = False #set to false to avoid frame and GIF creation
  
  # initial params
  score = 0
  i = 0
  t = 0

  while simulation.vis.Run():
      if simulation.time<simulation.max_duration:
        simulation.step()
        u = control.feedBack(simulation.observe())
        simulation.control(u)
        score += u**2     
        simulation.log()
        simulation.refreshTime()
        
        # check if the score has been low enouh times for the system to be stable
        if u**2 < 3e-8:
            if i == 12:
                t = simulation.time
                i += 1
            else:
                i += 1
                
      else:
        simulation.vis.GetDevice().closeDevice()
        
  simulation.writeData()
  
  # return time and score
  return round(t,2), round(score)
  
# make kick list and msts list
kicks = np.arange(0, 801, 20)
msts = []

# run main for each kick and add it to msts
for i in kicks:
    s = main(i)
    msts.append(s)
    
    # sleep to stop code from crashig
    sleep(1)

print(msts)


