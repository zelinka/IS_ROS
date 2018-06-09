#!/usr/bin/env python
from __future__ import print_function
import tf
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import roslib
import sys
import rospy
from math import sqrt

from geometry_msgs.msg import PointStamped, Vector3, Pose

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def euclidian(tocka1, tocka2):
    #print("euclidian")
    #print('tocka x', tocka1.pose.position.x)
    #print('tocka y', tocka1.pose.position.y)
    return sqrt((tocka1.pose.position.x - tocka2.x)**2 + (tocka1.pose.position.y - tocka2.y)**2)

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
            self.centroid = C_tocka(tocka.pose.position.x,tocka.pose.position.y,tocka.pose.position.z)
        else:
            c = C_tocka(0,0,0)
            for t in self.tocke:
                c.x += t.pose.position.x
                c.y += t.pose.position.y
                c.z += t.pose.position.z
            l = len(self.tocke)
            c.x /= l
            c.y /= l
            c.z /= l
            self.centroid = c

def pose_length(pose):
        return sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)

def get_robot_pose(time_rec):
    print(rospy.Time.now(),"rospy.Time.now()")     
    print(time_rec,"time_rec")  
    print('tralala') 
    #trans = tf_buf.lookup_transform('map', 'base_link', time_rec) 
    while(True):
        try:
            trans = tf_buf.lookup_transform('map', 'base_link', time_rec)
        except:
            ##print("error")
            continue
        break
    print("po whilu get robot pose")

    pose = Pose()
    pose.position.x = trans.transform.translation.x
    pose.position.y = trans.transform.translation.y
    pose.position.z = trans.transform.translation.z
    return pose

def get_goal(pose_r, pose_c):
        pose_g = Pose()
        pose_g.position.x = pose_r.position.x - pose_c.position.x
        pose_g.position.y = pose_r.position.y - pose_c.position.y
        pose_g.position.z = pose_r.position.z - pose_c.position.z
        dist  = pose_length(pose_g)
        pose_g.position.x /= dist
        pose_g.position.y /= dist
        #pose_g.position.z /= dist
        param = 0.30
        pose_g.position.x *= param
        pose_g.position.y *= param
        #pose_g.position.z *= 0.6

        pose_final = Pose()
        pose_final.position.x = pose_c.position.x + pose_g.position.x
        pose_final.position.y = pose_c.position.y + pose_g.position.y
        #pose_final.position.z = pose_c.position.z + pose_g.position.z
        
        #izracun kota vektorja glede na x os (za rotacijo robota)
        pose_final.position.z = np.arctan(-pose_g.position.y / -pose_g.position.x)

        if (-pose_g.position.x < 0):
            pose_final.position.z += np.pi
        elif(-pose_g.position.x == 0 and -pose_g.position.y < 0):
            pose_final.position.z += np.pi
        kot = 88
        pose_final.position.z += (np.pi / 180) * kot

        print("Kot vektorja = ", (pose_final.position.z*180/np.pi), " x=",-pose_g.position.x, " y=", -pose_g.position.y)
    
        return pose_final
    
    
def marker_callback(data):
    global marker_num
    global thresh
    '''
    if(cylinder_confirmation == 1):

        for c in cylinder_array:

        print("cylinder confirmation je 1")
        pose_robot = get_robot_pose(data.header.stamp)

        pose = Pose()
        pose = data.pose

        pose_goal = get_goal(pose_robot, pose)
        markers2.publish(pose_goal)
        
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
        marker.color = ColorRGBA(255,0,0, 1)
        marker_array.markers.append(marker)
        array_pub.publish(marker_array)

        if len(cylinder_array) == 0:
            tmp = C_Cilinder()
            tmp.add(data)
            cylinder_array.append(tmp)
        else:
            

    
        markerG = Marker()
        markerG.header.stamp = rospy.Time(0)
        markerG.header.frame_id = "map"
        markerG.pose = pose_goal
        markerG.type = Marker.CYLINDER
        markerG.action = Marker.ADD
        markerG.frame_locked = False
        markerG.id = marker_num
        marker_num += 1
        markerG.scale = Vector3(0.1, 0.1, 0.1)
        markerG.color = ColorRGBA(1,1,1, 1)
        marker_array.markers.append(markerG)
        
        marker.pose = pose_goal
        marker.id = marker_num
        marker_num += 1
        marker_array.markers.append(marker)
        marker.color = ColorRGBA(0,255,0, 1)
        array_pub.publish(marker_array)
        
        print("\ndodan marker in pozicija goal-a\n")
    else:
    '''
    if(data.pose.position.x != None or data.pose.position.y != None or data.pose.position.z != None):
        #tocka = C_tocka(data.pose.position.x, data.pose.position.y, data.pose.position.z)
        tocka = data

        barva = data.color
        print('dobljena barva',barva)
        rdeca = barva.r
        zelena = barva.g
        modra = barva.b

        print('R: ',rdeca,'G: ',zelena,'B: ',modra)

        dodan = False
        novKrog = True
        print('stevilo skupin zaznav',len(cylinder_array))
        for cylinder in cylinder_array:
            if(euclidian(tocka, cylinder.centroid) < thresh):
                novKrog = False

        if (novKrog):
            tmp = C_Cilinder()
            tmp.add(tocka)
            cylinder_array.append(tmp)

            pose_robot = get_robot_pose(tocka.header.stamp)
            if(pose_robot.position.x != None or pose_robot.position.y != None or pose_robot.position.z != None):
                pose = Pose()
                pose.position.x = tocka.pose.position.x
                pose.position.y = tocka.pose.position.y
                pose.position.z = tocka.pose.position.z
                pose.orientation.w = 1

                pose_goal = get_goal(pose_robot, pose)
                # nastavi se barva na orientation.w
                if(pose_goal.position.x != None or pose_goal.position.y != None or pose_goal.position.z != None):
                    pose_barva = 0

                    if(data.color.r > 0):
                        if(data.color.g > 0):
                            pose_barva = 3
                        else:
                            pose_barva = 0
                    else:
                        if(data.color.g > 0):
                            pose_barva = 1
                        else:
                            pose_barva = 2

                    pose_goal.orientation.w = pose_barva

                    markers2.publish(pose_goal)
                    
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
                    marker.color = ColorRGBA(rdeca,zelena,modra, 1)
                    marker_array.markers.append(marker)
                    array_pub.publish(marker_array)

                    marker.pose = pose_goal
                    marker.id = marker_num
                    marker_num += 1
                    marker_array.markers.append(marker)
                    marker.color = ColorRGBA(0,255,0, 1)
                    array_pub.publish(marker_array)
                    
                    print("\ndodan marker in pozicija goal-a\n")


if __name__ == "__main__":

    print("zacetek cylinder marker filtering")
    rospy.init_node('cylinder_marker_clustering', anonymous=False)

    cylinder_array = []
    thresh = 0.5
    cylinder_confirmation = 2

    marker_array = MarkerArray()

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    
    marker_num = 0
    markers2 = rospy.Publisher("/markers_cylinder", Pose, queue_size=100)
    marker_sub = rospy.Subscriber("detected_cylinder", Marker, marker_callback)

    array_pub = rospy.Publisher("/markers2", MarkerArray, queue_size=100)

    rospy.spin()
