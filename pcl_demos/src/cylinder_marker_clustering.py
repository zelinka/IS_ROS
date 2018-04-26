#!/usr/bin/env python
from __future__ import print_function
 
import roslib
import sys
import rospy
from math import sqrt

from geometry_msgs.msg import PointStamped, Vector3, Pose

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def euclidian(tocka1, tocka2):
    return sqrt((tocka1.x - tocka2.x)**2 + (tocka1.y - tocka2.y)**2)

class C_tocka:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class C_Cilinder:
    def __init__(self):
        self.centroid = None
        self.tocke = []
        self.markirana = False

    def add(self, tocka):
        self.tocke.append(tocka)
        if(len(self.tocke) == 1):
            self.centroid = tocka
        else:
            c = C_tocka(0,0,0)
            for t in self.tocke:
                c.x += t.x
                c.y += t.y
                c.z += t.z
            l = len(self.tocke)
            c.x /= l
            c.y /= l
            c.z /= l
            self.centroid = c

def marker_callback(data):
    global marker_num
    global thresh
    tocka = C_tocka(data.pose.position.x, data.pose.position.y, data.pose.position.z)
    print("dobil point")
    if len(cylinder_array) == 0:
        tmp = C_Cilinder()
        tmp.add(tocka)
        cylinder_array.append(tmp)
    else:
        dodan = False
        novKrog = True
            
        for cylinder in cylinder_array:
            if(euclidian(tocka, cylinder.centroid) < thresh):
                novKrog = False
                print(euclidian(tocka, cylinder.centroid))

            if(len(cylinder.tocke) < cylinder_confirmation+1 and not dodan):
                cylinder.add(tocka)
                dodan = True

            if(len(cylinder.tocke) == cylinder_confirmation):
                cylinder.markirana = True
                print("dodan marker")
                # Create a Pose object with the same position
                center = cylinder.centroid

                pose = Pose()

                pose.position.x = center.x
                pose.position.y = center.y
                pose.position.z = center.z

                pose.orientation.w = 1

                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = "map"
                marker.pose = pose
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.id = marker_num
                marker_num += 1
                marker.scale = Vector3(0.1, 0.1, 0.1)
                marker.color = ColorRGBA(1, 0, 0, 1)
                marker_array.markers.append(marker)
                array_pub.publish(marker_array)
        if (novKrog):
            tmp = C_Cilinder()
            tmp.add(tocka)
            cylinder_array.append(tmp)


if __name__ == "__main__":
    rospy.init_node('cylinder_marker_clustering', anonymous=False)

    cylinder_array = []
    thresh = 0.5
    cylinder_confirmation = 2

    marker_array = MarkerArray()

    marker_num = 0

    marker_sub = rospy.Subscriber("detected_cylinder", Marker, marker_callback)

    array_pub = rospy.Publisher("confirmed_cylinders", MarkerArray, queue_size=100)

    rospy.spin()
