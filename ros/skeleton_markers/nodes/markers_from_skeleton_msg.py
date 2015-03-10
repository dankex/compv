#!/usr/bin/env python

"""
    Publish a skeleton message as a list of visualization markers for RViz
        
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from skeleton_markers.msg import Skeleton
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SkeletonMarkers():
    def __init__(self):
        rospy.init_node('skeleton_markers')
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Skeleton Markers Node...")
        
        self.rate = rospy.get_param('~rate', 20)
        self.scale = rospy.get_param('~scale', 0.07)
        self.lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        self.ns = rospy.get_param('~ns', 'skeleton_markers')
        self.id = rospy.get_param('~id', 0)
        self.color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})

        rate = rospy.Rate(self.rate)
        
        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('skeleton_markers', Marker)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = self.ns
        self.markers.id = self.id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(self.lifetime)
        self.markers.scale.x = self.scale
        self.markers.scale.y = self.scale
        self.markers.color.r = self.color['r']
        self.markers.color.g = self.color['g']
        self.markers.color.b = self.color['b']
        self.markers.color.a = self.color['a']
                
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.markers)                        
            rate.sleep()
        
    def skeleton_handler(self, msg):
        self.markers.header.frame_id = msg.header.frame_id
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()
        for joint in msg.name:           
            p = Point()
            p = msg.position[msg.name.index(joint)]
            self.markers.points.append(p)

    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Marker Node.')
        
if __name__ == '__main__':
    try:
        SkeletonMarkers()
    except rospy.ROSInterruptException:
        pass
