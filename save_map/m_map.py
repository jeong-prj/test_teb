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

import rospkg 

class makeMapNode:
  def __init__(self):
    print('make map init')
    #rospy.Subscriber("/gazebo/model_states", Int32, self.messageCallback, queue_size=10)
    #self.set_state = rospy.Publisher("gazebo/set_model_state", Modelstate, queue_size=10)
    self._occ_grid_metadata = None
    self._occ_cost_metadata = None
    self.grid = 0
    self.cost = 0 
    self.num = 0
    
    #x, y, x, y, z, w
    self.goal = None
    
    self._sub_goal = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.getGoal, queue_size=1)
      
#    while self._occ_grid_metadata is None and self._occ_cost_metadata is None and not rospy.is_shutdown():
    
    while self.goal is None:
      continue
      
    self._sub_gmap = rospy.Subscriber("/map", OccupancyGrid, self.writeGridMap, queue_size=1)
    self._sub_cmap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.writeCostMap, queue_size=1)

    while not rospy.is_shutdown():
      while self._occ_grid_metadata is None or self._occ_cost_metadata is None:
        rospy.sleep(1.0)    
      rospy.loginfo('%d map saved', self.num)
      self.num += 1
      self.grid = 0
      self.cost = 0
      self._occ_grid_metadata = None
      self._occ_cost_metadata = None

#    while self._occ_cost_metadata is None and not rospy.is_shutdown():
#      rospy.sleep(0.1)
        
  def getGoal(self, data):
    self.goal = [0, 0, 0, 0, 0, 0]
  
    self.goal[0] = data.goal.target_pose.pose.position.x
    self.goal[1] = data.goal.target_pose.pose.position.y
    
    self.goal[2] = data.goal.target_pose.pose.orientation.x
    self.goal[3] = data.goal.target_pose.pose.orientation.y
    self.goal[4] = data.goal.target_pose.pose.orientation.z
    self.goal[5] = data.goal.target_pose.pose.orientation.w
    
    print "set goal pose x: {0}, y: {1}".format(self.goal[0], self.goal[1])
    print "set goal pose x: {0}, y: {1}, z: {2}, w: {3}".format(self.goal[2], self.goal[3], self.goal[4], self.goal[5])
    
  
  def writeGridMap(self, data):
    rospy.loginfo("{0} Got a full grid OccupancyGrid update".format(self.num))
    if self.grid is 1:
      print("already done")
      return 0
    self.grid=1
    
    try:
      map_g = open("/home/ej/Desktop/test_planner/src/maps/map_g_{0}.txt".format(self.num),"w")
      rospy.loginfo('{0} file is opened'.format(self.num))
    except:
      rospy.logerr("fail to open file..")
      return False
      
    self._occ_grid_metadata = data
    
    wid = self._occ_grid_metadata.info.width
    hei = self._occ_grid_metadata.info.height
    ori_x=self._occ_grid_metadata.info.origin.position.x
    ori_y=self._occ_grid_metadata.info.origin.position.y
    res = self._occ_grid_metadata.info.resolution

    map_g.write("grid map {0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res))
    print "{0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res)
    
    map_g.write("{0} {1}\n".format(self.goal[0], self.goal[1]))
    map_g.write("{0} {1} {2} {3}\n".format(self.goal[2], self.goal[3], self.goal[4], self.goal[5]))
    
    tf_listener = tf.TransformListener()
    
    tf_matrix=None
    while tf_matrix is None:
      if tf_listener.canTransform(target_frame="/map", source_frame="/odom", time=rospy.Time(0)):
        tf_matrix = tf_listener.lookupTransform(target_frame="/map", source_frame="/odom", time=rospy.Time(0))
        print "{0} map to odom".format(self.num)
        print tf_matrix
    
    map_g.write("{0} {1}\n".format("map", "odom"))
    map_g.write("{0} {1}\n".format(tf_matrix[0][0], tf_matrix[0][1]))
    map_g.write("{0} {1} {2} {3}\n".format(tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2], tf_matrix[1][3]))
    
    tf_matrix=None
    while tf_matrix is None:
      if tf_listener.canTransform(target_frame="/odom", source_frame="/base_link", time=rospy.Time(0)):
        tf_matrix = tf_listener.lookupTransform(target_frame="/odom", source_frame="/base_link", time=rospy.Time(0))
        print "{0} odom to baselink".format(self.num)
        print tf_matrix
    
    map_g.write("{0} {1}\n".format("odom", "base_link"))
    map_g.write("{0} {1}\n".format(tf_matrix[0][0], tf_matrix[0][1]))
    map_g.write("{0} {1} {2} {3}\n".format(tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2], tf_matrix[1][3]))
    
    i = 0
    for x in self._occ_grid_metadata.data:   
      if i < wid:
        map_g.write("{0} ".format(x))
        i += 1
      else:
        map_g.write("\n{0} ".format(x))
        i = 1
    print "{0} gridmap done".format(self.num)
    
    map_g.close()
    
  def writeCostMap(self, data):
    rospy.loginfo("{0} Got a full cost OccupancyGrid update".format(self.num))
    if self.cost is 1:
      print("already done")
      return 0
    self.cost=1
     
    try:
      map_c = open("/home/ej/Desktop/test_planner/src/maps/map_c_{0}.txt".format(self.num),"w")
      rospy.loginfo('file is opened')
    except:
      rospy.logerr("fail to open file..")
      return False
      
    self._occ_cost_metadata = data
    
    wid = self._occ_cost_metadata.info.width
    hei = self._occ_cost_metadata.info.height
    ori_x=self._occ_cost_metadata.info.origin.position.x
    res = self._occ_cost_metadata.info.resolution
    ori_y=self._occ_cost_metadata.info.origin.position.y

    map_c.write("{0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res))
    print "cost map {0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res)
    
    i = 0
    for x in self._occ_cost_metadata.data:   
      if i < wid:
        map_c.write("{0} ".format(x))
        i += 1
      else:
        map_c.write("\n{0} ".format(x))
        i = 1
    print "{0} costmap done".format(self.num)
    
    map_c.close()
      
      
if __name__ == '__main__':
  rospy.init_node('make_map_node')
  node = makeMapNode()
  
  
 
