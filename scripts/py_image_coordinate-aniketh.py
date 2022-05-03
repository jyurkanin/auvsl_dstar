#!/usr/bin/env python
# Python Image Subscriber Node
# This node was modified with reference to imcmahon's reply on
# http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
# Basic idea is convert from ROS Image -> CvBridge Converter -> OpenCV
# Note you'd still need the CMakeList.txt and package.xml
# Reference: 
#  http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
#  http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import rospy                      # rospy
import numpy as np                # numpy
import cv2                        # OpenCV2
import matplotlib.pyplot as plt
import scipy.ndimage
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image # ROS Image message
from sensor_msgs.msg import Range
from uuv_sensor_ros_plugins_msgs.msg import ChemicalParticleConcentration #CPC sensor
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg

#Instantiate CV Bridge
bridge = CvBridge()
avgrho = 0
avgtheta = 0
order = 0
horizontal_error = 0
head_error = 0
outputtheta = 0
CPC = 0
x = 0
y = 0
z = 0
MAXCPC = 0
countMax = 0
leakx = 0
leaky = 0
leakz = 0
dist0 = 2000
dirw = 0
dirx = 0
diry = 0
dirz = 0
vehicle_orientation = 0
sonar0 = 0
sonar1 = 0
sonar2 = 0
sonar3 = 0
range0 = 0
range1 = 0
range2 = 0
range3 = 0
dh0x = 0
dh0y = 0
dh1x = 0
dh1y = 0
dh2x = 0
dh2y = 0
dh3x = 0
dh3y = 0
grid_map = np.zeros((400,400,3), np.uint8)
grid_map_sonar = np.zeros((400,400,3), np.uint8)
sonar_switch = 0
no_line  = 1 #line is present
obstacle_avoidance_sonar = np.zeros((1,768))
obstacle_front = np.zeros((1,128))
obstacle_right = np.zeros((1,384))
obstacle_left = np.zeros((1,384))
obstacle_decision = 0
linear_velocity_y = 0
angular_velocity_z = 0
linear_velocity_x = 0
obstacle_map = np.zeros([200,200])



def sonar0_callback(msg):
    global sonar0
    global range0
    global dh0x
    global dh0y
    
    sonar0 = msg.range 
    cthetay = np.cos(-0.13)
    sthetay = np.sin(-0.13)
    rotatey = np.matrix([[cthetay,sthetay,0],[-sthetay,cthetay,0],[0,0,1]])
    cthetap = np.cos(0.4-3.14159/2)
    sthetap = np.sin(0.4-3.14159/2)
    rotatep = np.matrix([[cthetap,0,-sthetap],[0 , 1, 0],[sthetap,0, cthetap,]])
    array0 = np.matrix([[sonar0],[0],[0]])
    array1 = (rotatey * array0)
    
    array2 = rotatep * array1
   
    dh0x = array2[2]
    dh0y = array2[1]

def sonar1_callback(msg):
    global sonar1
    global range1
    global dh1x
    global dh1y
    
    sonar1 = msg.range 
    cthetay = np.cos(0.13)
    sthetay = np.sin(0.13)
    rotatey = np.matrix([[cthetay,sthetay,0],[-sthetay,cthetay,0],[0,0,1]])
    cthetap = np.cos(0.4-3.14159/2)
    sthetap = np.sin(0.4-3.14159/2)
    rotatep = np.matrix([[cthetap,0,-sthetap],[0 , 1, 0],[sthetap,0, cthetap,]])
    array0 = np.matrix([[sonar1],[0],[0]])
   

    array1 = (rotatey*array0)
   
    array2 = (rotatep * array1)
    

    dh1x = array2[2]
    dh1y = array2[1]

def sonar2_callback(msg):
    global sonar2
    global range2
    global dh2x
    global dh2y
    
    sonar2 = msg.range 
    cthetay = np.cos(-0.1)
    sthetay = np.sin(-0.1)
    rotatey = np.matrix([[cthetay,sthetay,0],[-sthetay,cthetay,0],[0,0,1]])
    cthetap = np.cos(-0.4-3.14159/2)
    sthetap = np.sin(-0.4-3.14159/2)
    rotatep = np.matrix([[cthetap,0,-sthetap],[0 , 1, 0],[sthetap,0, cthetap,]])
    array0 = np.matrix([[sonar2],[0],[0]])
    array1 = (rotatey * array0)
    array2 = (rotatep * array1)

    dh2x = array2[2]
    dh2y = array2[1]

def sonar3_callback(msg):
    global sonar3
    global range3
    global dh3
    
    sonar3 = msg.range 
    cthetay = np.cos(0.1)
    sthetay = np.sin(0.1)
    rotatey = np.matrix([[cthetay,sthetay,0],[-sthetay,cthetay,0],[0,0,1]])
    cthetap = np.cos(-0.4-3.14159/2)
    sthetap = np.sin(-0.4-3.14159/2)
    rotatep = np.matrix([[cthetap,0,-sthetap],[0 , 1, 0],[sthetap,0, cthetap,]])
    array0 = np.matrix([[sonar3],[0],[0]])
    
    array1 = (rotatey * array0)
   
    array2 = (rotatep * array1)

    dh3x = array2[2]
    dh3y = array2[1]


def euler_from_quaternion(x, y, z, w):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return yaw_z # in radians



def obstacle_avoidance_sonar_callback(msg):
    global obstacle_avoidance_sonar
    global obstacle_front
    global obstacle_left
    global obstacle_right
    global obstacle_decision
    global linear_velocity_x
    global linear_velocity_y
    global angular_velocity_z
    global obstacle_distance_from_vehicle
    global obstacle_map

    

    obstacle_map = np.zeros([200,200])



    obstacle_avoidance_sonar = msg.ranges
    #print(obstacle_avoidance_sonar)
    obstacle_right = np.array((obstacle_avoidance_sonar[0:383]))
    obstacle_left = np.array((obstacle_avoidance_sonar[384:767]))


    for i in range(len(obstacle_avoidance_sonar)):
        if obstacle_avoidance_sonar[i] != np.inf:
            distance = obstacle_avoidance_sonar[i]
            print("distance = ", distance)
            print("index = ", i)
            angle = (i-383)*0.1692708333*0.01745329252
            print("angle = ", angle)
            x_vehicle = (distance*np.cos(angle))
            print("x_v = ", x_vehicle)
            y_vehicle = (distance*np.sin(angle))
            print("y_v = ", y_vehicle)
            world = coordinate_vehicletoworld(x_vehicle,y_vehicle,x,y,vehicle_orientation)
            x_world = round(world[0])
            print("x world = ", x_world)
            y_world = round(world[1])
            print("y world = ", y_world)
            x_index_in_obstacle_map = x_world+100
            y_index_in_obstacle_map = y_world+100
            if y_index_in_obstacle_map <= 200:
                if x_index_in_obstacle_map <= 200:
                    obstacle_map[x_index_in_obstacle_map,y_index_in_obstacle_map] = 1
                    print("obstacle_map = ", obstacle_map)
            else:
                break
        





    
    
    
            
    


    #appended_obstacle_right = obstacle_right[obstacle_right != np.inf]
    #appended_obstacle_left = obstacle_left[obstacle_left != np.inf]
    
    #left_most_point = max(appended_obstacle_left)
    #right_most_point = max(appended_obstacle_right)

   # left_most_index = np.where(obstacle_left == left_most_point)
    #right_most_index = np.where(obstacle_right == right_most_point)

    #print("The right most point is:", right_most_index)
    #print("The left most point is: ",left_most_index)

    #for i in appended_obstacle_left:
        #theta_left = angle*


    #obstacle_distance_from_vehicle = obstacle_avoidance_sonar[384]
    #angle = 0.1796875*np.pi/180
    #theta_left = angle*left_most_index[0]
    #theta_right = angle*right_most_index[0]
    #obstacle_left_vehicle_coordinates = np.array([ left_most_point*np.sin(theta_left), left_most_point*np.cos(theta_left)])
    #obstacle_right_vehicle_coordinates = np.array([right_most_point*np.sin(theta_right),right_most_point*np.cos(theta_right)])

    #x_l_v =-np.absolute(obstacle_left_vehicle_coordinates[0])
    #y_l_v = np.absolute(obstacle_left_vehicle_coordinates[1])

    #x_r_v  = np.absolute(obstacle_right_vehicle_coordinates[0])
    #y_r_v = np.absolute(obstacle_right_vehicle_coordinates[1])



    #left_edge_in_global = coordinate_vehicletoworld(x_l_v,y_l_v,x,y,vehicle_orientation)
    #right_edge_in_global = coordinate_vehicletoworld(x_r_v,y_r_v,x,y,vehicle_orientation)







    







    #print(obstacle_avoidance_sonar)
    #x = len(obstacle_avoidance_sonar)
    #print (x)
    #obstacle_front = min(obstacle_avoidance_sonar[320:448])
    #print (obstacle_front)
    #obstacle_right = min(obstacle_avoidance_sonar[0:319])
    #obstacle_left  = min(obstacle_avoidance_sonar[449:767])



    



    # if obstacle_right < obstacle_left: #this means turn towards the right
    #     obstacle_decision = 1
    # elif obstacle_left < obstacle_right: # turn towards the left
    #     obstacle_decision = 2
    # else:
    #     obstacle_decision = 3 #doesnt matter which direction

    # if  obstacle_front > 10:
    #     linear_velocity_x = 0.5
    # else:
    #     linear_velocity_x = 0

    #time.sleep(5)


    #linear_velocity_x = 0.5

    #time.sleep(2)

    # if obstacle_right <= obstacle_left:
    #if obstacle_front < 7 or 1 < obstacle_left < 7:
        #linear_velocity_x = 0
        #angular_velocity_z = -0.05
    #else:
    #    angular_velocity_z = 0
    #    linear_velocity_x = 0.5
        # if obstacle_left != 0:
        #     linear_velocity_x = 0.5
        # else:
        #     linear_velocity_x = 0
    # elif obstacle_left < obstacle_right:
    #     if obstacle_front < 5:
    #         linear_velocity_x = 0
    #         angular_velocity_z = 0.05
    #     else:
    #         angular_velocity_z = 0
        # if obstacle_right != 0:
        #     linear_velocity_x = 0.5
        # else:
        #     linear_velocity_x = 0


#    if obstacle_left == 0 and obstacle_front > 20:
    #    linear_velocity_x = 0.5
    #    angular_velocity_z = 0.05




    #rint ("obstacle_left = ", obstacle_left)
    #print ("obstacle_right = ", obstacle_right)
    #print("left_most_point =", left_most_point)
    #print("right_most_point = ", right_most_point)
    #print("left_most_point = ", left_most_point)
    #print("right_most_point = ", right_most_point)

    #print ("obstacle_decision = ", obstacle_decision)
    #print ("obstacle_front = ", obstacle_front)
    #print ("linear_velocity_x = ", linear_velocity_x)
    #print ("angular_velocity_z = ", angular_velocity_z)


    # for i in range(0, 319):
    #     if obstacle_right[i] > 100:
    #         obstacle_right = obstacle_right[i+1]
    #     if np.all(obstacle_right == obstacle_right[0]):
    #         obstacle_right = 0
    #
    # #obstacle_left = obstacle_sonar[449:767]
    # for i in range(449, 767):
    #     if obstacle_left[i] > 100:
    #         obstacle_right = obstacle_right[i-1]
    #     if np.all(obstacle_left == obstacle_left[0]):
    #         obstacle_left = 0

    #print("x = " ,obstacle_left)
    #print("y = " ,obstacle_right)



def maxdetector(msg,posx,posy,posz):
    global MAXCPC
    global countMax
    global leakx
    global leaky
    global leakz
    a = 0
    if msg > MAXCPC:
        MAXCPC = msg
        #print("Maxcpc =", MAXCPC)
        leakx = posx
        leaky = posy
        leakz = posz
        countMax = 0
    if msg < 0.9*MAXCPC:
        countMax = countMax +1
        print("!!!!cpc<max!!!!")
        print("Count Max =",countMax)
        if countMax is 10:
            a = 1
            countMax = 0
            print(a)

    return a

def odom_callback(msg):
    global x
    global y
    global z
    global dist0
    global dirw
    global dirx
    global diry
    global dirz
    global vehicle_orientation
    x = msg.pose.pose.position.x 
    y = msg.pose.pose.position.y 
    z = msg.pose.pose.position.z
    dirw = msg.pose.pose.orientation.w
    dirx = msg.pose.pose.orientation.x
    diry = msg.pose.pose.orientation.y
    dirz = msg.pose.pose.orientation.z
    vehicle_orientation = euler_from_quaternion(dirx, diry, dirz, dirw)
    dist = np.square(x)+ np.square(y)
    if dist<dist0:
        dist0 = dist
        #print("distance decrease, x=",x, "y=",y)
    #print("position = (", x , y ,z, ")")
    alpha = np.arccos(dirw)


def chemical_callback(msg):
    global CPC
    CPC = msg.concentration 
    #print("CPC=", CPC)

def image_callback(msg):
    global avgrho
    global avgtheta
    global horizontal_error
    global head_error
    global outputtheta
    global grid_map
    global sonar_switch
    global no_line 
    #print("PyImageSubscriber node  Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "mono8")
    except CvBridgeError, e:
        print(e)
    else:
        #saving images---------------------------------
        #order = order + 1
        #filename = "img{}.jpg".format(order)
        #cv2.imwrite(filename, cv2_img)
        #----------------------------------------------
        size = cv2_img.shape
        threshold = 10
        width = size[1]
        height = size[0]
        pdf = np.zeros(256)

        #for k in range(255):            
        hist = cv2.calcHist([cv2_img],[0],None,[256],[0,256])
        for k in range(255):
            if hist[k] >threshold:
                hist[k] = threshold

        cdf = np.cumsum(hist)/255
        for i in range(width):
            for j in range(height):
                a = cv2_img[j,i]
                cv2_img[j,i] = round(a*cdf[a])
        #----------------------calculate entropy--------------------------------
        #hist2 = np.array(cv2.calcHist([cv2_img],[0],None,[256],[0,256]))
        #entropy = 0
        #for i in range(255):
        #    if hist2[i] != 0:
        #        #print(i)
        #        probability = hist2[i]/height/width
        #        entropy = entropy + probability*np.log2(1/probability)
        #print("entropy = ", entropy)     
        #----------------------------------------------------------------------
        #dr = (np.amax(cv2_img)+1)/(np.amin(cv2_img)+1)
        #print(np.log10(dr))
        cv2_img = cv2.medianBlur(cv2_img, 3)        
        cv2.imshow("median", cv2_img)
        #------------Canny----------------------------------
        #gray = cv2.GaussianBlur(cv2_img, (5,5), 0)
        #masked_edge = cv2.Canny(gray, 30,80)
        #---------------------------------------------------
        #------------Sobel----------------------------------
        #gray = cv2.GaussianBlur(cv2_img, (3,3), 0)
        gray = cv2_img
        kernel = np.array([[1.0/8.0, 2.0/8.0, 1.0/8.0], [0, 0, 0], [-1.0/8.0, -2.0/8.0, -1.0/8.0]])
        sobelx = scipy.ndimage.convolve(gray, kernel, mode='nearest')
        sobely = scipy.ndimage.convolve(gray, np.transpose(kernel) , mode='nearest')
        squareMatrix = np.uint8(0.5*np.square(sobelx) + 1*np.square(sobely))
        cutoff = 5*np.average(squareMatrix) #6 20210711
        masked_edge = cv2.threshold(squareMatrix, cutoff, 255, cv2.THRESH_BINARY)
        #cv2.imshow("Sobel", masked_edge)
        #print(cutoff)
        #print(masked_edge[1])
        #---------------------------------------------------
        line_image = np.zeros((height,width,3), np.uint8)
        #lines = cv2.HoughLines(masked_edge, 1, np.pi/180, 150)
        lines = cv2.HoughLines(masked_edge[1], 1, np.pi/180, 60) #120 #100 #80 20210711
        sumrho = 0
        sumtheta = 0
        cnt = 0
        #print(len(lines))
        #------------average angle--------------------------
        if lines is not None:
            sonar_switch = 0
            if len(lines) < 10:
                draw_line_number = len(lines)
            else:
                draw_line_number = 10 
            maxtheta = lines[0][0][1]
            maxrho = lines[0][0][0]
            if maxtheta >1.5708:
                maxtheta = maxtheta - 3.14159
                maxrho = -maxrho            
            mintheta = lines[0][0][1]
            minrho = lines[0][0][0]
            if mintheta >1.5708:
                mintheta = mintheta - 3.14159
                minrho = -minrho
            for i in (range(draw_line_number)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if theta > 1.5708:
                    theta = theta-3.14159
                    rho = -rho
                sumrho = sumrho+rho
                sumtheta = sumtheta + theta
                if theta>maxtheta:
                    maxtheta = theta
                    maxrho = rho
                if theta<mintheta:
                    mintheta = theta
                    minrho = rho
                
                cnt = cnt + 1
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(line_image, pt1, pt2, (0,0,255), 1)
                #print("rho=",rho)
                #print("theta=",theta)
            #print("maxtheta=", maxtheta)
            #print("mintheta=", mintheta)
            #print("difference=", maxtheta-mintheta)
            difference = maxtheta-mintheta
            #if difference > 0.4:
            #    print("Difference Too Large")
            #    sonar_switch = 1
            #x1 = (maxrho-np.sin(maxtheta)*height)/np.cos(maxtheta)
            #x2 = (minrho-np.sin(mintheta)*height)/np.cos(mintheta)
            #tubewidth1 = x2-x1
            #y1 = maxrho/np.sin(maxtheta)
            #y2 = minrho/np.sin(mintheta)
            #tubewidth2 = y2-y1
            #tubewidth = min(tubewidth1,tubewidth2)
            #print("Tube width", tubewidth)
        else:
            print("No line")
            no_line = 0 #sensors cannnot detect any line
            #sonar_switch = 1
            #msg = Twist()
            #msg.angular.z = -0.01     
        if cnt is 0:
            avgrho = avgrho
            avgtheta = avgtheta
        else:           
            avgrho = sumrho/(cnt+0.001)
            avgtheta = sumtheta/(cnt+0.001)
        #print("avgrho=",avgrho)
        #print("avgtheta=", avgtheta)
        a1 = np.cos(avgtheta)
        b1 = np.sin(avgtheta)
        x01 = a1*avgrho
        y01 = b1*avgrho
        pt11 = (int(x01 + 1000*(-b1)), int(y01 + 1000*(a1)))
        pt21 = (int(x01 - 1000*(-b1)), int(y01 - 1000*(a1)))
        cv2.line(line_image, pt11, pt21, (0,255,255), 1)
        centerpoint = (avgrho-height*b1)/a1
        xe = (avgrho-np.sin(avgtheta)*0.5*height)/np.cos(avgtheta)
        route_coor = coordinate_transform(xe, 0.5*height)
        near_coor = coordinate_transform(centerpoint,height)
        head_error = np.arctan(route_coor[1]/route_coor[0])
        pipeline = route_coor-near_coor
        outputtheta = np.arctan(pipeline[1]/pipeline[0])
        horizontal_error = -near_coor[1]
        #head_error = np.arctan((0.5*width-avgrho/np.cos(avgtheta))/(0.5*height))
        far_pointx = (avgrho)/a1
        far_point_coor = coordinate_transform(far_pointx, 0)
        #---------------image grid map---------------------------------------------------
        grid1 = coordinate_vehicletoworld(near_coor[0],near_coor[1],x,y,vehicle_orientation)
        grid1_drawx = int((-grid1[1]))+200
        grid1_drawy = int((grid1[0]))+200
        grid2 = coordinate_vehicletoworld(route_coor[0],route_coor[1],x,y,vehicle_orientation)
        grid2_drawx = int((-grid2[1]))+200
        grid2_drawy = int((grid2[0]))+200
        #grid3 = coordinate_vehicletoworld(far_point_coor[0], far_point_coor[1],x,y,vehicle_orientation)
        #grid3_drawx = np.int(-grid3[1])+200
        #grid3_drawy = np.int(grid3[0])+200
        if (grid1_drawx<=400) and (grid1_drawy<=400) and (grid1_drawx>=0) and (grid1_drawy>=0):
        #if (500 <=400)
            grid_map[grid1_drawx][grid1_drawy] = (255,0,0)
        if (grid2_drawx<=400) and (grid2_drawy<=400) and (grid2_drawx>=0) and (grid2_drawy>=0):    
            grid_map[grid2_drawx][grid2_drawy] = (0,255,0)
        #if (grid3_drawx<=400) and (grid3_drawy<=400) and (grid3_drawx>=0) and (grid3_drawy>=0):    
        #    grid_map[grid3_drawx][grid3_drawy] = (0,0,255)
        cv2.imshow("Grid", grid_map)
        #---------------------------------------------------------------------------------------
        #----------------Sonar/ Image compensation---------------------------------------------
        #hori_s, head_s, ori_s = sonar_error(sonar0,sonar1,sonar2,sonar3)
        #if np.absolute(outputtheta) > np.absolute(ori_s):
        #    outputtheta = ori_s
        #if np.absolute(horizontal_error) > np.absolute(hori_s):
        #    horizontal_error = hori_s
        #if np.absolute(head_error) > np.absolute(head_s):
        #    head_error = head_s
        #---------------sonar grid map---------------------------------------------------------
        if sonar0 < 9:
            #sonar_in_vehicle_x = dh0
            #sonar_in_vehicle_y = 0
            grids0 = coordinate_vehicletoworld(dh0x, dh0y,x,y,vehicle_orientation)
            grids0_drawx = int((-grids0[1]))+200
            grids0_drawy = int((grids0[0]))+200
            grid_map_sonar[grids0_drawx][grids0_drawy] = (255,0,0)
        if sonar1 < 9:
            #sonar_in_vehicle_x = 0
            #sonar_in_vehicle_y = dh1
            grids1 = coordinate_vehicletoworld(dh1x, dh1y,x,y,vehicle_orientation)
            grids1_drawx = int((-grids1[1]))+200
            grids1_drawy = int((grids1[0]))+200
            grid_map_sonar[grids1_drawx][grids1_drawy] = (0,255,0)
        if sonar2 < 9:
            #sonar_in_vehicle_x = 0
            #sonar_in_vehicle_y = 0
            grids2 = coordinate_vehicletoworld(dh2x, dh2y,x,y,vehicle_orientation)
            grids2_drawx = int((-grids2[1]))+200
            grids2_drawy = int((grids2[0]))+200
            grid_map_sonar[grids2_drawx][grids2_drawy] = (0,0,255)    
        if sonar3 < 9:
            #sonar_in_vehicle_x = 0
            #sonar_in_vehicle_y = -dh3
            grids3 = coordinate_vehicletoworld(dh3x, dh3y,x,y,vehicle_orientation)
            grids3_drawx = int((-grids3[1]))+200
            grids3_drawy = int((grids3[0]))+200
            grid_map_sonar[grids3_drawx][grids3_drawy] = (0,0,255)        

        cv2.imshow("Grid_sonar", grid_map_sonar)    

        #----------------------------------------------------
        color_edge = np.dstack((masked_edge[1], masked_edge[1], masked_edge[1]))
        #cv2.imshow("Sobel", color_edge)
        #gray_edge = cv2.cvtColor(color_edge, cv2.COLOR_BGR2GRAY)
        combo = cv2.addWeighted(color_edge, 0.8, line_image, 1, 0)  
        cv2.imshow("Combo", combo)

        #cv2.imshow("Image Display", cv2_img)
        
        #take_action()
        # Wait 30 ms to allow image to be drawn.
        # Image won't display properly without this cv2.waitkey
        cv2.waitKey(30) 
        # Save your OpenCV2 image as a jpeg 
        #cv2.imwrite('depth92.jpeg', combo)

def coordinate_vehicletoworld(x_v,y_v,x,y,orientation):
    #x_v, y_v are the coordinate of target observed by the vehicle
    #x, y are the coordinate of vehicle itself
    ctheta = np.cos(orientation)
    stheta = np.sin(orientation)
    rotate = np.matrix([[ctheta,-stheta,0],[stheta,ctheta,0],[0,0,1]])
    array0 = np.matrix([[x_v],[y_v],[-90]])
    print (array0)
    

    array1 = rotate * array0
    print(array1)
    
    x2 = array1[0]
    y2 = array1[1]
    z2 = array1[2]
    x3 = (x2+x)
    x3.astype(int)
    y3 = (y2+y)
    y3.astype(int)
    z3 = (z2)
    z3.astype(int)
    array3 = np.array([[x3],[y3],[z3]])

    return array3

#def image_listener():
        
def coordinate_transform(pix,piy):
    xd = pix-384.5
    yd = piy-246.5
    h = 9.9
    stheta = np.sin(0.6)
    ctheta = np.cos(0.6)
    z0 = h/(stheta+yd/407*ctheta)
    x0 = xd*z0/407
    y0 = yd*z0/407
    array0 = np.matrix([[x0],[y0],[z0]])
    rotate = np.matrix([[1,0,0],[0,ctheta,stheta],[0,-stheta,ctheta]])
   

    array1 = rotate * array0
    

    change = np.matrix([[0,0,1],[-1,0,0],[0,-1,0]])

    array2 = change * array1
    
    x2 = array2[0]
    y2 = array2[1]
    z2 = array2[2]
    x3 = x2+1.15
    y3 = y2
    z3 = z2+0.4
    array3 = np.array([[x3],[y3],[z3]])

    return array3

def sonar_error(sonard0,sonard1,sonard2,sonard3):
    if sonard0 < 9:
        d0 = 1
    else:
        d0 = 0    
    if sonard1 < 9:
        d1 = 1
    else:
        d1 = 0    
    if sonard2 < 9:
        d2 = 1
    else:
        d2 = 0
    if sonard3 < 9:
        d3 = 1
    else:
        d3 = 0        
    if (d0+d1+d2+d3) is 0:
        sonar_orientation = 3.14159
        sonar_horizontal = 300

    if (d0+d1) is 2 and (d2+d3) is 0:
        sonar_orientation = 3.14159
        sonar_horizontal = 300

    if (d1+d3) is 2 and (d0+d2) is 0:
        sonar_orientation = 3.14159
        sonar_horizontal = 300

    if (d2+d3) is 2 and (d0+d1) is 0:
        sonar_orientation = 3.14159
        sonar_horizontal = 300

    if (d0+d2) is 2 and (d1+d3) is 0:
        sonar_orientation = 3.14159
        sonar_horizontal = 300                

    if (d0+d1+d2+d3) is 4:
        sonar_horizontal = 0
        sonar_orientation = 0

    if (d0+d1+d2+d3) is 3:
        if d0 is 0:
            sonar_horizontal = -0.45
            sonar_orientation = -0.18
        if d1 is 0:
            sonar_horizontal = 0.45
            sonar_orientation = 0.18
        if d2 is 0:
            sonar_horizontal = -0.45
            sonar_orientation = 0.18
        if d3 is 0:
            sonar_horizontal = 0.45
            sonar_orientation = -0.18

    if (d0+d1+d2+d3) is 1:
        if d0 is 1:
            sonar_horizontal = 1.45
            sonar_orientation = -0.18
        if d1 is 1:
            sonar_horizontal = -1.45
            sonar_orientation = 0.18
        if d2 is 1:
            sonar_horizontal = 1.45
            sonar_orientation = 0.18
        if d3 is 1:
            sonar_horizontal = -1.45
            sonar_orientation = -0.18

    if (d0+d3) is 2 and (d1+d2) is 0:
        sonar_horizontal = 0
        sonar_orientation = 0.18

    if (d0+d3) is 0 and (d1+d2) is 1:
        sonar_horizontal = 0
        sonar_orientation = -0.18

    sonar_heading = sonar_orientation

    return sonar_horizontal, sonar_heading, sonar_orientation        


def main():
    # Initiate the node
    rospy.init_node('py_image_listener')
    rate = rospy.Rate(3)
    # Setupt the subscription, camera/rb/image_raw is used in turtlebot_gazebo example
    sub = rospy.Subscriber("rexrov/rexrov/camera/camera_image", Image, image_callback)
    sub2 = rospy.Subscriber("rexrov/particle_concentration", ChemicalParticleConcentration, chemical_callback)
    sub3 = rospy.Subscriber("rexrov/pose_gt", Odometry, odom_callback)
    sub4 = rospy.Subscriber("rexrov/dvl_sonar0", Range, sonar0_callback)
    sub5 = rospy.Subscriber("rexrov/dvl_sonar1", Range, sonar1_callback)
    sub6 = rospy.Subscriber("rexrov/dvl_sonar2", Range, sonar2_callback)
    sub7 = rospy.Subscriber("rexrov/dvl_sonar3", Range, sonar3_callback)
    obstacle_avoidance = rospy.Subscriber("rexrov/sonar", LaserScan, obstacle_avoidance_sonar_callback, queue_size = 10)

    front_angle_error = rospy.Publisher('front_angle_error', Float32, queue_size = 10)
    orientation_error = rospy.Publisher('orientation_error', Float32, queue_size = 10)
    position_error = rospy.Publisher('/distance_error', Float32, queue_size = 10)
    cpc_output = rospy.Publisher('/CPCoutput',Float32, queue_size = 10)
    switch_output = rospy.Publisher('/Sonar_switch', Int8, queue_size = 10)
    lost_search = rospy.Publisher('lost_search', Int8, queue_size = 10)
    
    obstacle_decision_ros = rospy.Publisher('obstacle_decision_ros', Int8, queue_size = 10)
    obstacle_front_ros = rospy.Publisher('obstacle_front_ros', Float32, queue_size = 10)
    obstacle_right_ros = rospy.Publisher('obstacle_right_ros', Float32, queue_size = 10)
    obstacle_left_ros = rospy.Publisher('obstacle_left_ros', Float32, queue_size = 10)
    linear_velocity_x_ros = rospy.Publisher('linear_velocity_x_ros', Float32, queue_size =10)
    linear_velocity_y_ros = rospy.Publisher('linear_velocity_y_ros', Float32, queue_size =10)
    angular_velocity_z_ros = rospy.Publisher('angular_velocity_z_ros', Float32, queue_size =10)
    obstacle_map_ros = rospy.Publisher('obstacle_map_ros', numpy_msg(Float32), queue_size = 10)

    while not rospy.is_shutdown():
        #msg = Twist()
        #print(msg)
        #pub.publish(msg)
        #print(horizontal_error)
        #rospy.loginfo(hello_str)

        orientation_error.publish(outputtheta)
        position_error.publish(horizontal_error)
        front_angle_error.publish(head_error)
        cpc_output.publish(CPC)
        switch_output.publish(sonar_switch)
        lost_search.publish(no_line)
        leak_outpout = maxdetector(CPC,x,y,z)
        obstacle_decision_ros.publish(obstacle_decision)
        obstacle_front_ros.publish(obstacle_front)
        obstacle_right_ros.publish(obstacle_right)
        obstacle_left_ros.publish(obstacle_left)
        linear_velocity_x_ros.publish(linear_velocity_x)
        linear_velocity_y_ros.publish(linear_velocity_y)
        angular_velocity_z_ros.publish(angular_velocity_z)
        obstacle_map_ros.publish(obstacle_map)

        if leak_outpout is 1:
            print("There's a leak at", leakx, leaky, leakz)
        rate.sleep()
        #pub.publish(generate())        
    # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
    #cv2.destroyWindow("Image Display")
    #msg.linear.x = 0
    #msg.angular.z = 0
    #pub.publish(msg)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    #image_listener()
    main()
