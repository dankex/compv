#!/usr/bin/env python

"""
    Convert a skeleton transform tree to a list of visualization markers for RViz.
        
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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import re

class SkeletonMarkers():
    def __init__(self):
        rospy.init_node('markers_from_tf')
                
        rospy.loginfo("Initializing Skeleton Markers Node...")
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        tf_prefix = rospy.get_param('~tf_prefix', '')
        skeleton_frames = rospy.get_param('~skeleton_frames', '')
        self.fixed_frame = rospy.get_param('~fixed_frame', 'openni_depth_frame')
        
        # Normalize the tf_prefix variable with / at the beginning and end
        tf_prefix = '/' + tf_prefix + '/'
        tf_prefix = re.sub('/+', '/', tf_prefix)
        
        # Normalize the tf_prefix variable with / at the beginning and end
        self.fixed_frame = '/' + tf_prefix + '/' + self.fixed_frame
        self.fixed_frame = re.sub('/+', '/', self.fixed_frame)

        # Initialize tf listener
        tf_listener = tf.TransformListener()
        
        # Define a marker publisher
        marker_pub = rospy.Publisher('skeleton_markers', Marker)
        
        # Intialize the markers
        self.init_markers()
        
        # The openni_tracker identifies each skeleton frame with a user_id beginning
        # with 1 and incrementing as new users are tracked.
        user_id = None
        
        # Get the list of all skeleton frames from tf
        #skeleton_frames = [f for f in tf_listener.getFrameStrings() if f.startswith("/" + self.tf_prefix)]
        
               
        # Begin the main loop
        while not rospy.is_shutdown():
            # Get a list of all frames
            all_frames = [f for f in tf_listener.getFrameStrings()]
            
            # Test for the user ID to track
            try:
                test = all_frames.index(tf_prefix + 'torso')
                user_id = ""
            except:
                try:
                    test = all_frames.index(tf_prefix + 'torso_1')
                    user_id = "_1"
                except:
                    pass
            
            if user_id is None:
                r.sleep
                continue
            
            # Get all skeleton frames for the detected user user
            user_frames = list()
    
            for frame in skeleton_frames:
                qualified_frame = tf_prefix + frame + user_id
                try:
                    all_frames.index(qualified_frame)
                    user_frames.append(qualified_frame)
                except:
                    pass
                
            #tf_listener.waitForTransform(self.fixed_frame, user_frames[0], rospy.Time(), rospy.Duration(25.0))

            # Set the markers header
            self.markers.header.stamp = rospy.Time.now()
            
            # Clear the markers point list
            self.markers.points = list()
            
            # Loop through the skeleton frames
            for frame in user_frames:
                if frame == self.fixed_frame:
                    continue
                # Find the position of the frame's origin relative to the fixed frame.
                try:
                    position = Point()
                    
                    # Get the transformation from the fixed frame to the skeleton frame
                    (trans, rot)  = tf_listener.lookupTransform(self.fixed_frame, frame, rospy.Time(0))
                    position.x = trans[0]
                    position.y = trans[1]
                    position.z = trans[2]
                                        
                    # Set a marker at the origin of this frame
                    self.markers.points.append(position)
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    rospy.logerr("tf error when looking up " + frame + ' and ' + self.fixed_frame)
                    continue
            
            # Publish the set of markers
            marker_pub.publish(self.markers)                       
            r.sleep()
            
    def init_markers(self):
        # Set various parameters
        scale = rospy.get_param('~scale', 0.07)
        lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        ns = rospy.get_param('~ns', 'skeleton_markers')
        id = rospy.get_param('~id', 0)
        color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # Initialize the marker points list
        self.markers = Marker()
        self.markers.header.frame_id = self.fixed_frame
        self.markers.ns = ns
        self.markers.id = id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(lifetime)
        self.markers.scale.x = scale
        self.markers.scale.y = scale
        self.markers.color.r = color['r']
        self.markers.color.g = color['g']
        self.markers.color.b = color['b']
        self.markers.color.a = color['a']
        
if __name__ == '__main__':
    try:
        SkeletonMarkers()
    except rospy.ROSInterruptException:
        pass
