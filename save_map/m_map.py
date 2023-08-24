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

# Local costmap's frequency > global costmap.

class makeMapNode:
  def __init__(self):
    print('make map init')
    #rospy.Subscriber("/gazebo/model_states", Int32, self.messageCallback, queue_size=10)
    #self.set_state = rospy.Publisher("gazebo/set_model_state", Modelstate, queue_size=10)
    self._occ_lcost_metadata = None
    self._occ_gcost_metadata = None

    self.num = 0
    
    #x, y, x, y, z, w
    self.goal = None
    
    self._sub_goal = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.getGoal, queue_size=1)
    self._sub_gcmap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.writeGCostMap, queue_size=1)    
#    while self._occ_grid_metadata is None and self._occ_cost_metadata is None and not rospy.is_shutdown():
    
    while self.goal is None or self._occ_gcost_metadata is None:
      continue
      
#    self._sub_gmap = rospy.Subscriber("/map", OccupancyGrid, self.writeGridMap, queue_size=1)
    self._sub_lcmap = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.writeLCostMap, queue_size=1)

    while not rospy.is_shutdown():
      while self._occ_lcost_metadata is None:
        rospy.sleep(4.0)    
      rospy.loginfo('%d map saved', self.num)
      self.num += 1

      self._occ_lcost_metadata = None
        
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
    
  def writeGCostMap(self, data):
    rospy.loginfo("{0} Got a full GLOBAL cost OccupancyGrid update".format(self.num))
      
    self._occ_gcost_metadata = data
    
    
  def writeLCostMap(self, data):
    rospy.loginfo("{0} Got a full LOCAL cost OccupancyGrid update".format(self.num))
     
    try:
      map_lc = open("/home/ej/Desktop/test_planner/src/maps/map_lc_{0}.txt".format(self.num),"w")
      rospy.loginfo('LOCAL file is opened')
    except:
      rospy.logerr("fail to open LOCAL file..")
      return False
      
    self._occ_lcost_metadata = data
    
    wid = self._occ_lcost_metadata.info.width
    hei = self._occ_lcost_metadata.info.height
    ori_x=self._occ_lcost_metadata.info.origin.position.x
    res = self._occ_lcost_metadata.info.resolution
    ori_y=self._occ_lcost_metadata.info.origin.position.y

    map_lc.write("{0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res))
    print "LOCAL cost map {0} {1} {2} {3} {4}\n".format(wid, hei, ori_x, ori_y, res)
    
    map_lc.write("{0} {1}\n".format(self.goal[0], self.goal[1]))
    map_lc.write("{0} {1} {2} {3}\n".format(self.goal[2], self.goal[3], self.goal[4], self.goal[5]))
    
    tf_listener = tf.TransformListener()
    
    tf_matrix=None
    while tf_matrix is None:
      if tf_listener.canTransform(target_frame="/map", source_frame="/odom", time=rospy.Time(0)):
        tf_matrix = tf_listener.lookupTransform(target_frame="/map", source_frame="/odom", time=rospy.Time(0))
        print "{0} map to odom".format(self.num)
        print tf_matrix
    
    map_lc.write("{0} {1}\n".format("map", "odom"))
    map_lc.write("{0} {1}\n".format(tf_matrix[0][0], tf_matrix[0][1]))
    map_lc.write("{0} {1} {2} {3}\n".format(tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2], tf_matrix[1][3]))
    
    tf_matrix=None
    while tf_matrix is None:
      if tf_listener.canTransform(target_frame="/odom", source_frame="/base_link", time=rospy.Time(0)):
        tf_matrix = tf_listener.lookupTransform(target_frame="/odom", source_frame="/base_link", time=rospy.Time(0))
        print "{0} odom to baselink".format(self.num)
        print tf_matrix
    
    map_lc.write("{0} {1}\n".format("odom", "base_link"))
    map_lc.write("{0} {1}\n".format(tf_matrix[0][0], tf_matrix[0][1]))
    map_lc.write("{0} {1} {2} {3}\n".format(tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2], tf_matrix[1][3]))    
    
    i = 0
    for x in self._occ_lcost_metadata.data:   
      if i < wid:
        map_lc.write("{0} ".format(x))
        i += 1
      else:
        map_lc.write("\n{0} ".format(x))
        i = 1
    print "{0} LOCAL costmap done".format(self.num)
    
    map_lc.close()
    
###############################################################################################
         
    try:
      map_gc = open("/home/ej/Desktop/test_planner/src/maps/map_gc_{0}.txt".format(self.num),"w")
      rospy.loginfo('GLOBAL file is opened')
    except:
      rospy.logerr("fail to open GLOBAL file..")
      return False
    
    wid_g = self._occ_gcost_metadata.info.width
    hei_g = self._occ_gcost_metadata.info.height
    ori_x_g=self._occ_gcost_metadata.info.origin.position.x
    ori_y_g=self._occ_gcost_metadata.info.origin.position.y
    res_g = self._occ_gcost_metadata.info.resolution

    map_gc.write("{0} {1} {2} {3} {4}\n".format(wid_g, hei_g, ori_x_g, ori_y_g, res_g))
    print "GLOBAL cost map {0} {1} {2} {3} {4}\n".format(wid_g, hei_g, ori_x_g, ori_y_g, res_g)
    
    i = 0
    for x in self._occ_gcost_metadata.data:   
      if i < wid:
        map_gc.write("{0} ".format(x))
        i += 1
      else:
        map_gc.write("\n{0} ".format(x))
        i = 1
    print "{0} GLOBAL costmap done".format(self.num)
    
    map_gc.close()
    
      
if __name__ == '__main__':
  rospy.init_node('make_map_node')
  node = makeMapNode()
  
  
 
