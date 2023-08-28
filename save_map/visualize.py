#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import String, Int32
#from cost_map_msgs.msg import CostMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import tf
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseActionGoal

from tf import transformations as t

import rospkg 

import numpy as np
from PIL import Image

# Local costmap's frequency > global costmap.

class makeMapNode:
  def __init__(self):
    print('visualize map init')
    self.lcmapFile = None
    self.gcmapFile = None

    self.lcmap_info=None
    self.gcmap_info=None
    #x, y, x, y, z, w
    self.goal = []
    self.robot = None
    
    #costmap data
    self.lcmap_data = []
    self.gcmap_data = []
    
    #for i in range(2):
    #  self.getData(i)
    
    #self.getData(0)
    self.getData(0)

  def gridmap2world(self, x, y):
    fgx = x * self.gcmap_info[4]  + self.gcmap_info[2] 
    fgy = y * self.gcmap_info[4]  + self.gcmap_info[3]
  
    return fgx, fgy


  def world2gridmap(self, x, y):
    fx = float(x) - float(self.gcmap_info[2])  / float(self.gcmap_info[4])
    fy = float(y) - float(self.gcmap_info[3])  / float(self.gcmap_info[4])

    return int(fx), int(fy)


  def getData(self, num):
    try:
      self.gcmapFile = open("/home/ej/Desktop/test_planner/src/maps/map_gc_{0}.txt".format(num),"r")
      rospy.loginfo('GLOBAL file is opened')
    except:
      rospy.logerr("fail to open GLOBAL file..")
      return False
    
    list_file = self.gcmapFile.read().split('\n')
    
    idx = -1
    for line in list_file:
      idx += 1
      g_data = line.split(' ')
      
      if idx is 0:
        self.gcmap_info = np.array(g_data).copy()
        
      else:
        np_g_data = []
        for item in g_data:
          if item is '':
            break
          if int(item) is 0:
            np_g_data.append(254)
          
          elif int(item) is -1:
            np_g_data.append(0)
            
          else:
            np_g_data.append(int(item))
          
        #print np.array(np_g_data).astype(np.uint8)
        self.gcmap_data.append(np.array(np_g_data).astype(np.uint8))
        
    gcp_data = np.array(self.gcmap_data)
    
    lc = "/home/ej/Desktop/test_planner/src/maps/map_lc_{0}.txt".format(num)
    try:
      self.lcmapFile = open("/home/ej/Desktop/test_planner/src/maps/map_lc_{0}.txt".format(num),"r")
      rospy.loginfo('LOCAL file is opened')
    except:
      rospy.logerr("fail to open LOCAL file..")
      return False
    
    list_file = self.lcmapFile.read().split('\n')
    
    idx = -1
    for line in list_file:
      idx += 1
      l_data = line.split(' ')
      
      if idx > 2 and idx < 21:
        if idx is 16:
          self.robot = l_data
          print "robot "
          print self.robot
          
          rx, ry = self.world2gridmap(float(self.robot[0]), float(self.robot[1]))
          print rx, ry
        continue
      
      if idx is 0:
        self.lcmap_info = np.array(l_data).copy()
      
      elif idx is 1 or idx is 2:
        self.goal.append(l_data)
        
      else:
        np_l_data = []
        for item in l_data:
          if item is '':
            break
          if int(item) is 0:
            np_l_data.append(254)
          
          elif int(item) is -1:
            np_l_data.append(100)
            
          else:
            np_l_data.append(int(item))
          
        #print np.array(np_l_data).astype(np.uint8)
        self.lcmap_data.append(np.array(np_l_data).astype(np.uint8))
        
    lcp_data = np.array(self.lcmap_data)
    
    img = Image.fromarray(lcp_data)
    img.show()
    
    lx, ly = self.world2gridmap(self.lcmap_info[2], self.lcmap_info[3])
    print "origin "
    print lx, ly
    
    y_= 0
    for y_i in range(ly-(int(self.lcmap_info[1])/2), ly+(int(self.lcmap_info[1])/2)):
      x_= 0
      for x_i in range(lx-(int(self.lcmap_info[0])/2), lx+(int(self.lcmap_info[0])/2)):
        gcp_data[y_i][x_i] = lcp_data[y_][x_]
        x_+=1
        
      y_+=1
    
    img2 = Image.fromarray(gcp_data)
    img2.show()
    
      
if __name__ == '__main__':
  #rospy.init_node('make_map_node')
  node = makeMapNode()
  
  
 
