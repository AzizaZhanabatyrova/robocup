import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from aruco_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys
from std_msgs.msg import String

###############################################################################################################
# Some variables


# Callback functions from subscribers
def callback_odom(data):
    global x
    global y
    global theta
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    x = round(position.x, 3)
    y = round(position.y, 3)
    theta = round(2*math.atan2(orientation.z,orientation.w), 3)

def callback_array(msg):
    global marker_array
    marker_array = msg.markers
  
def callback_color(msg):
    global color_img
    global allow_processing
    color_img = (bridge.imgmsg_to_cv2(msg, "rgb8")) [240:480,:]
    allow_processing = 1

def callback_depth(msg):
    global depth_img
    depth_img = (bridge.imgmsg_to_cv2(msg, "passthrough")) [240:480,:]

def callback_chatter(data):
	global partners_distance_ball
	global partners_side
	global partners_beta
	global partner_sent_info
	global permission_to_continue
	global robot
	info = data.data
	if robot == 'robot2':
		if info[:2] == 'd1':
			partners_distance_ball = float(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 's1':
			partners_side = int(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 'b1':
			partners_beta = float(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 'p1':
			permission_to_continue = int(info[2:])

	elif robot == 'robot1':
		if info[:2] == 'd2':
			partners_distance_ball = float(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 's2':
			partners_side = int(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 'b2':
			partners_beta = float(info[2:])
			partner_sent_info = partner_sent_info + 1
		if info[:2] == 'p2':
			permission_to_continue = int(info[2:])
rospy.sleep(1)

# Functions
def stop_moving():    
    twist.linear.x = 0.0 
    twist.angular.z = 0.0
    publisher.publish(twist)
    rospy.sleep(0.1)
    return;

def rotate_to_goal_center(): 
    m = marker_array[0]
    error = m.pose.pose.position.x
    tolerance = 0.08
    print("Aligning with the center of marker")
    while abs(error) > tolerance:
        m = marker_array[0]
        twist.linear.x = 0.0
        speed = -1*error
        
        if abs(speed) < 0.1:
            speed = 0.1*speed/abs(speed)
            
        twist.angular.z = speed
        publisher.publish(twist)
        rospy.sleep(0.1)
        error = m.pose.pose.position.x
    return m;

def process_image():
    global cnt
    global xr
    global yr
    global wr
    global hr
    global center_rectangle_x    
    global center_rectangle_y
    global allow_processing
    
    hasBothColors = 0
    foundBall = 0
    hasBlue = 0
    hasRed = 0
    lower_blue = (0,0,120)
    upper_blue = (100,100,255)
    lower_red = (200,0,0)
    upper_red = (255,100,100)
        
    mask_blue = cv2.inRange(color_img, lower_blue, upper_blue)
    mask_red = cv2.inRange(color_img, lower_red, upper_red)

    kernel = np.ones((5,5),np.uint8)
    erosion_blue = cv2.erode(mask_blue,kernel,iterations = 1)
    dilation_blue = cv2.dilate(erosion_blue,kernel,iterations = 3)
    erosion_red = cv2.erode(mask_red,kernel,iterations = 1)
    dilation_red = cv2.dilate(erosion_red,kernel,iterations = 3)
    mask = cv2.bitwise_or(dilation_blue, dilation_red)

    cnt = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    if (len(cnt)>0):
        # Something's found
        cnt.sort(key=cv2.contourArea, reverse=True)
        xr, yr, wr, hr = cv2.boundingRect( cnt[0] )
        
        center_rectangle_x = xr + wr/2
        center_rectangle_y = yr + hr/2

        # Check if rectangle has both red and blue
        new_img = color_img[yr:yr+hr,xr:xr+wr]
        new_mask_blue = cv2.inRange(new_img, lower_blue, upper_blue)
        new_mask_red = cv2.inRange(new_img, lower_red, upper_red)
        hasBlue = sum(sum(new_mask_blue))
        hasRed = sum(sum(new_mask_red))
        hasBothColors = hasBlue and hasRed
        distance = np.nanmin(depth_img[yr:yr+hr, xr:xr+wr])
        if (hasBothColors) and (distance < 3):
            foundBall = 1
        else:
            foundBall = 0

    if cnt is None:
        cnt = []    
    
    allow_processing = 0

    return (foundBall)

def align_and_obtain_ball_info():
    global center_rectangle_x
    global xr
    global yr
    global wr
    global hr
    
    angle_to_ball = 0
    distance_to_ball = 0
    # Make sure the robot is stopped
    stop_moving()
    # Wait until everything is settled down
    rospy.sleep(0.5)
    
    process_image()
    error_alignment = 320 - center_rectangle_x
    
    angle_correction = 0.001917*error_alignment
    angle_to_ball = theta + angle_correction
    
    distance_correction = -0.00006*error_alignment
    distance_to_ball = np.nanmin(depth_img[yr:yr+hr, xr:xr+wr]) + distance_correction
    
    
    while (abs(error_alignment) > 12):
    
    
        angle_correction = 0.001917*error_alignment
        angle_to_ball = theta + angle_correction

        if (angle_to_ball > 3.1415926):
            angle_to_ball = angle_to_ball - 6.2831853
        if (angle_to_ball < -3.1415926):
            angle_to_ball = angle_to_ball + 6.2831853

        distance_correction = -0.00006*error_alignment
        distance_to_ball = np.nanmin(depth_img[yr:yr+hr, xr:xr+wr]) + distance_correction
        
        if  abs(angle_correction) <= 0.03:
            break
        
        rotate_to_desired_position(angle_to_ball)
        stop_moving()
        rospy.sleep(0.2)
        process_image()
        error_alignment = 320 - center_rectangle_x
    
    return angle_to_ball, distance_to_ball
    
def should_rotate_clockwise(from_theta, to_theta):

    rotate_clockwise = 0
    
    if (from_theta < 0):
        from_theta = from_theta + 6.28
    if (to_theta < 0):
        to_theta = to_theta + 6.28

    if (to_theta > from_theta):
        if (to_theta - from_theta > 3.14):
            rotate_clockwise = -1 # Rotate anticlockwise
        else:
            rotate_clockwise = 1 # Rotate clockwise
    else:
        if (from_theta - to_theta > 3.14):
            rotate_clockwise = 1 # Rotate clockwise
        else:
            rotate_clockwise = -1 # Rotate anticlockwise
            
    return rotate_clockwise

# Rotate until you are looking to the desired position. At this point, the vehicle is supposed to be looking to the ball
def rotate_to_desired_position(desired_theta):
    
    if (desired_theta < 0):
        pos_desired_theta = desired_theta + 6.28
    else:
        pos_desired_theta = desired_theta

    if (theta < 0):
        pos_theta = theta + 6.28
    else:
        pos_theta = theta
    
    while (abs(pos_desired_theta - pos_theta) > 0.03):

        error_diff_angle = pos_desired_theta - pos_theta
        if (error_diff_angle > 3.14):
            error_diff_angle = error_diff_angle - 6.28

	if (error_diff_angle < -3.14):
	    error_diff_angle = error_diff_angle + 6.28

        speed = 3.5*error_diff_angle
        if abs(speed) > 0.8:
            speed = 0.8*speed/abs(speed)
        twist.angular.z = speed
        publisher.publish(twist)
        rospy.sleep(0.1)
        
        if (theta < 0):
            pos_theta = theta + 6.28
        else:
            pos_theta = theta
    
    rospy.sleep(0.25)

def find_beta_alpha(S, distance_to_goal,angle_to_goal, angle_to_ball):
    R = 0.7
    P = distance_to_goal
    
    # Make all angles positive
    if (angle_to_ball < 0):
        pos_angle_to_ball = angle_to_ball + 6.28
    else:
        pos_angle_to_ball = angle_to_ball
        
    if (angle_to_goal < 0):
        pos_angle_to_goal = angle_to_goal + 6.28
    else:
        pos_angle_to_goal = angle_to_goal
    
    alpha = abs(pos_angle_to_ball - pos_angle_to_goal)

    if (alpha > 3.14):
        alpha = 6.28 - alpha

    L = math.sqrt(math.pow(P,2) + math.pow(S,2) - 2*P*S*math.cos(alpha))
    beta = math.acos((math.pow(S,2) + math.pow(L,2) -  math.pow(P,2))/(2*S*L))
    return beta,alpha;

# Find partner
# robots on opposite side
def find_partner(alpha, beta, distance_to_ball):
    
    my_side = should_rotate_clockwise(angle_to_goal,angle_to_ball)
    
    
    if (my_side != partners_side):
        # Robots on opposite side
        psi = 2*3.14 - beta - partners_beta
    else:
        # robots on same side
        psi = abs(partners_beta - beta)

    distance_to_partner = math.sqrt(math.pow(distance_to_ball,2) + math.pow(partners_distance_ball,2) - 2*distance_to_ball*partners_distance_ball*math.cos(psi))
    phi = abs(math.asin(partners_distance_ball*math.sin(psi)/distance_to_partner))
    
    if (my_side != partners_side):
        # Robots on opposite side
        if my_side == 1: # If I am on the right side
            if (partners_beta > alpha): # Rotate clockwise to see the partner
                angle_to_partner = angle_to_ball + phi
            else:
                angle_to_partner = angle_to_ball - phi
        else:
            if (partners_beta > alpha): # Rotate anticlockwise to see the partner
                angle_to_partner = angle_to_ball - phi
            else:
                angle_to_partner = angle_to_ball + phi
        
    if (my_side == 1):
        angle_to_partner = angle_to_partner - 0.9
    else:
        angle_to_partner = angle_to_partner + 0.9
        
    if (angle_to_partner < -3.14):
        angle_to_partner = angle_to_partner + 6.28
    if (angle_to_partner > 3.14):
        angle_to_partner = angle_to_partner - 6.28

    print(distance_to_partner)
    print(phi)
    return distance_to_partner, angle_to_partner;


####################################################################################################################

if __name__ == "__main__":
    global twist
    global publisher
    robot = str(sys.argv[1])
    rospy.init_node(robot)
    rospy.sleep(1)
    twist = Twist()
    bridge = CvBridge()

    # Globals from callbacks
    x = 0
    y = 0 
    theta = 0
    marker_array = []
    allow_processing = 0
    partners_distance_ball = 0
    partners_gamma = 0
    permission_to_continue = 0
    partner_sent_info = 0
    
    distance_to_goal = 0
    distance_to_ball = 0
    angle_to_goal = 0
    angle_to_ball = 0
    detected_marker = 0 
    detected_ball = 0
    r = rospy.Rate(100)
    cnt = []
    hasSeenBall = 0
    rospy.sleep(1)
    
    # Subscribers and publisher
    if robot == "robot1":
        subscriber = rospy.Subscriber("/odom", Odometry, callback_odom)
        subscriber_array = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callback_array)
        subscriber_color = rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_color)
        subscriber_depth = rospy.Subscriber("/camera/depth/image_rect", Image, callback_depth)

        publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        my_distance = 'd1'
        my_side = 's1'
        my_beta = 'b1'
        my_permission = 'p1'

    elif robot == "robot2":
        subscriber = rospy.Subscriber("/bestRobot/odom", Odometry, callback_odom)
        subscriber_array = rospy.Subscriber("/bestRobot/aruco_marker_publisher/markers", MarkerArray, callback_array)
        subscriber_color = rospy.Subscriber("/bestRobot/camera/rgb/image_rect_color", Image, callback_color)
        subscriber_depth = rospy.Subscriber("/bestRobot/camera/depth/image_rect", Image, callback_depth)

        publisher = rospy.Publisher('/bestRobot/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        my_distance = 'd2'
        my_side = 's2'
        my_beta = 'b2'
        my_permission = 'p2'

    # Common
    rospy.Subscriber("chatter", String, callback_chatter)
    chat = rospy.Publisher('chatter', String, queue_size=10)
    rospy.sleep(1)

    # Rotate until detect something
    print("Rotating to find marker or ball.")
    while True: 
        if allow_processing == 1:
            hasSeenBall = process_image()

        if (len(cnt) > 0) and (detected_ball == 0) and (hasSeenBall): # found ball?
            stop_moving()     
            print("Ball was found. Obtaining its infomation.")

            angle_to_ball, distance_to_ball = align_and_obtain_ball_info()
            print("Got information of the ball")

            detected_ball = detected_ball + 1
            if detected_marker > 0:
                break

        if len(marker_array) > 0 and detected_marker == 0: # found a marker and it is not in the center?
            print("Marker was found")
            stop_moving()
            m = rotate_to_goal_center()
            # Save the angle and distance to the marker
            distance_to_goal = math.sqrt(math.pow(m.pose.pose.position.x,2) + math.pow(m.pose.pose.position.z,2))
            angle_to_goal = theta # from initial position
            stop_moving()
            print("Got information of the goal")
            detected_marker = detected_marker + 1
            hasSeenBall = 0
            if detected_ball > 0:
                break

        twist.linear.x = 0.0
        twist.angular.z = -0.35
        publisher.publish(twist)
        rospy.sleep(0.1)

    rospy.sleep(0.25)

    

    # Send info to the partner

    S = distance_to_ball

    beta, alpha = find_beta_alpha(S, distance_to_goal,angle_to_goal, angle_to_ball)
    rospy.sleep(0.1)
    chat.publish(my_distance + str(distance_to_ball))
    rospy.sleep(0.1)
    chat.publish(my_beta + str(beta))
    rospy.sleep(0.1)
    chat.publish(my_side + str(should_rotate_clockwise(angle_to_goal,angle_to_ball)))

    #Communicate about who is closer to the ball
    while partner_sent_info < 3:
        rospy.sleep(0.1)
    print('partners_distance_ball')
    print(partners_distance_ball)
    print('partners_side')
    print(partners_side)
    print('partners_beta')
    print(partners_beta)


    if partners_distance_ball > distance_to_ball: # If I am closer
        distance_to_goal, angle_to_goal = find_partner(alpha, beta, distance_to_ball)
    else: 
        while not permission_to_continue == 1:
            rospy.sleep(0.1)

        # Rotate until detect something
        print("Rotating to find ball again")
        detected_ball = 0
        hasSeenBall = 0
        while True: 
            if allow_processing == 1:
                hasSeenBall = process_image()

            if (len(cnt) > 0) and (detected_ball == 0) and (hasSeenBall): # found ball?
                stop_moving()     
                print("Ball was found. Obtaining its infomation.")

                angle_to_ball, distance_to_ball = align_and_obtain_ball_info()
                print("Got information of the ball")

                detected_ball = detected_ball + 1
                break

            twist.linear.x = 0.0
            twist.angular.z = -0.2
            publisher.publish(twist)
            rospy.sleep(0.1)


        rospy.sleep(0.25)

    S = distance_to_ball
    # Find distance and angle to desired position
    S = distance_to_ball
    beta, alpha = find_beta_alpha(S, distance_to_goal,angle_to_goal, angle_to_ball)
    gamma = 3.14 - beta
    R = 0.7

    K = math.sqrt(math.pow(S,2) + math.pow(R,2) - 2*S*R*math.cos(gamma))
    P = distance_to_goal
    print(distance_to_ball)
    print(angle_to_ball)

    M = math.acos((math.pow(K,2) + math.pow(S,2) - math.pow(R,2))/(2*K*S))
    # Calculate sign of M
    if (abs(angle_to_ball - angle_to_goal) > 3.14):
        diff_angle = abs(angle_to_ball - angle_to_goal) - 3.14
    else:
        diff_angle = abs(angle_to_ball - angle_to_goal)

    if (abs(angle_to_ball + M - angle_to_goal)> diff_angle):
        print("Turning clockwise")
    else:
        print("Turning anticlockwise")
        M = -M

    desired_theta = angle_to_ball + M

    if (desired_theta < -3.14):
        desired_theta = desired_theta + 6.28
    if (desired_theta > 3.14):
        desired_theta = desired_theta - 6.28

    rospy.sleep(1)

    rotate_to_desired_position(desired_theta)

    # Walk to the desired position
    starting_position_x = x
    starting_position_y = y
    travelled = 0

    while abs(K - travelled) > 0.065:
        error = K - travelled
        speed = 0.5*error
        if (abs(speed) > 1.5):
            speed = 1.5*speed/abs(speed)
        twist.linear.x = speed
        twist.angular.z = 0.0
        publisher.publish(twist)
        rospy.sleep(0.1)
        travelled = math.sqrt(math.pow(starting_position_x - x,2) + math.pow(starting_position_y - y,2))

    twist.linear.x = 0.0 
    twist.angular.z = 0.0
    publisher.publish(twist)
    rospy.sleep(1)

    # Rotate to ball
    cnt = []
    hasSeenBall = 0

    #Sign to rotate back to the ball
    sign = -M/abs(M)

    # Rotate until detect ball
    print("Rotating to find ball again.")
    while True: 
        twist.linear.x = 0.0
        twist.angular.z = sign*0.2
        publisher.publish(twist)
        rospy.sleep(0.1)

        hasSeenBall = process_image()

        if (len(cnt) > 0) and (hasSeenBall): # found ball?
            print("Ball was found. Aligning with the center of it")
            stop_moving()
            rospy.sleep(0.5)

            # Ball is found
            cnt.sort(key=cv2.contourArea, reverse=True)
            img = color_img.copy()
            cv2.drawContours(img, cnt, -1, (0,255,0), 3)
            xr, yr, wr, hr = cv2.boundingRect(cnt[0])
            rospy.sleep(0.5)

            center_rectangle_x = xr + wr/2
            center_rectangle_y = yr + hr/2 

            angle_to_ball, distance_to_ball = align_and_obtain_ball_info()


            print("Got information of the ball")
            break
    rospy.sleep(0.1)


    # Kick
    elapsed_time = 0
    starting_position_x = x
    starting_position_y = y
    distance_to_goal = math.sqrt(math.pow(m.pose.pose.position.x,2) + math.pow(m.pose.pose.position.z,2))
    travelled = 0

    if partners_distance_ball > distance_to_ball:
        distance_to_travel = 0.1*distance_to_goal + 0.25*distance_to_ball
    else:
        distance_to_travel = 0.1*distance_to_goal + 0.85

    while (travelled < distance_to_travel):
        twist.linear.x = 1.2
        publisher.publish(twist)
        rospy.sleep(0.1)
        travelled = math.sqrt(math.pow(starting_position_x - x,2) + math.pow(starting_position_y - y,2))
        publisher.publish(twist)
    twist.linear.x = 0.0 
    twist.angular.z = 0.0
    publisher.publish(twist)

    if partners_distance_ball > distance_to_ball:
        rospy.sleep(3)	
        chat.publish(my_permission + str(1))
        
    rospy.spin()
