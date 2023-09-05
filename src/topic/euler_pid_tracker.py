#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Global variables
current_path = None
target_index = 10


goal_x, goal_y = None, None

linear_errors = []
angular_errors = []
time_stamps = []

fig, (ax1, ax2) = plt.subplots(2, 1)


# 현재, 목표 위치
def goal_callback(msg, path_pub):

    global current_path, tf_buffer, goal_x, goal_y
    global points

    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y

    try:
        transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.header.frame_id = "base_link"

        transformed_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)
        path = interpolate_path(transformed_pose, msg)
        path_pub.publish(path)
        current_path = path

        
    except Exception as e:
        rospy.loginfo("Exception in goal_callback: %s", str(e))



# 보간법
def interpolate_path(start, end, resolution=0.05):
    global points
    path = Path()
    path.header.frame_id = "map"
    points = int(((end.pose.position.x - start.pose.position.x)**2 + (end.pose.position.y - start.pose.position.y)**2)**0.5 / resolution)
    rospy.loginfo(points)
    target_angle = math.atan2(end.pose.position.y - start.pose.position.y, 
                               end.pose.position.x - start.pose.position.x)
    quat = quaternion_from_euler(0,0,target_angle)
    for i in range(points + 1):
        interpolated_pose = PoseStamped()
        interpolated_pose.header.frame_id = "map" 
        interpolated_pose.header.stamp = rospy.Time.now() 
        alpha = float(i) / points
        interpolated_pose.pose.position.x = start.pose.position.x * (1.0 - alpha) + end.pose.position.x * alpha
        interpolated_pose.pose.position.y = start.pose.position.y * (1.0 - alpha) + end.pose.position.y * alpha
        interpolated_pose.pose.orientation.x = quat[0]
        interpolated_pose.pose.orientation.y = quat[1]
        interpolated_pose.pose.orientation.z = quat[2]
        interpolated_pose.pose.orientation.w = quat[3]
        
        path.poses.append(interpolated_pose)
    
    return path


#강종
def shutdown_hook():
    global cmd_vel_pub 
    rospy.loginfo("Shutting down")
    plt.close()
    twist = Twist()
    cmd_vel_pub.publish(twist)
    rospy.sleep(2) 
    

    rospy.wait_for_service('/gazebo/set_model_state')

    #가제보에서 로봇 원위치
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        model_state = ModelState()
        model_state.model_name = 'turtlebot3_burger'  # 로봇 모델의 이름
        model_state.pose.position.x = -1.72
        model_state.pose.position.y = -0.58
        model_state.pose.position.z = 0
        model_state.pose.orientation.z = 0 

        resp = set_state(model_state)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)



#pid
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        rospy.loginfo("Kp : %s, Ki : %s, Kd : %s", Kp,Ki,Kd )

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        # rospy.loginfo(output)
        return output


#그래프
def update_graph(i):
    ax1.clear()
    ax2.clear()
    
    ax1.plot(time_stamps, linear_errors, label='Linear Error')
    ax1.set_title('Linear')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Linear Error')
    
    ax2.plot(time_stamps, angular_errors, label='Angular Error')
    ax2.set_title('Angular')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Angular Error')





def main():
    global tf_buffer, cmd_vel_pub  
    global goal_x, goal_y 
    global target_index 

    rospy.init_node('interpolation_tracker')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    path_pub = rospy.Publisher('interpolated_plan', Path, queue_size=1)
    goal_sub = rospy.Subscriber('topic/goal', PoseStamped, lambda msg: goal_callback(msg, path_pub))
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    lh_pub = rospy.Publisher('lookahead', PoseStamped, queue_size=1)

    rate = rospy.Rate(20)
    
    control_angular = 0.0
    control_linear = 0.0

    max_linear = 0.3
    max_angular = 0.5

    pid_linear = PID(5.15, 1.8, 2)
    pid_angular = PID(9, 1.5, 1.8)


    global points
    points = 0
    current_x = 0.0
    current_y = 0.0
    goal_x = 0.0
    goal_y = 0.0

    ani = animation.FuncAnimation(fig, update_graph, interval=1000)
    start_time = rospy.get_time()

    rospy.on_shutdown(shutdown_hook)
    

    while not rospy.is_shutdown():
        try:
            # 현재 로봇의 theta 값
            transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
        except Exception as e:
            rospy.loginfo("fail to transform")
            rospy.sleep(1)
            continue

        goal_error = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)  

         #도착 했을 때
        if goal_error < 0.1: # and abs(angle_error) < 0.3:
            rospy.loginfo("완료")
            twist = Twist()  
            cmd_vel_pub.publish(twist)
            rospy.sleep(10000000000000000)
            
            break

        
        if current_path and target_index < len(current_path.poses):
            lh = PoseStamped()
            lh = current_path.poses[target_index]
            lh_pub.publish(lh)
            try:
                # 현재 로봇의 theta 값
                transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y
                

                #오일러
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                (_, _, current_theta) = euler_from_quaternion(quaternion)  

                # 목표의 theta 값
                target_x = current_path.poses[target_index].pose.position.x
                target_y = current_path.poses[target_index].pose.position.y
                
                # quaternion = (
                #     current_path.poses[target_index].pose.orientation.x,
                #     current_path.poses[target_index].pose.orientation.y,
                #     current_path.poses[target_index].pose.orientation.z,
                #     current_path.poses[target_index].pose.orientation.w
                # )
                # (_, _, target_theta) = euler_from_quaternion(quaternion)
                 

                # 현재 로봇 위치와 target_index에 해당하는 위치 사이의 직선 경로에 기반한 각도
                target_theta = math.atan2(target_y - current_y, target_x - current_x)

                
                # 에러 계산
                linear_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                angle_error = target_theta - current_theta 

                goal_error = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)  

                rospy.loginfo("linear_error : %s , angle_error : %s", linear_error, angle_error)
                # rospy.loginfo("goal error : %s", goal_error)
                rospy.loginfo("goal error %s", goal_error)

               

                
                # PID control
                control_linear = pid_linear.compute(linear_error)
                control_angular = pid_angular.compute(angle_error)

                control_linear = max(min(control_linear, max_linear), -max_linear)
                control_angular = max(min(control_angular, max_angular), -max_angular)
                
                twist = Twist()
                twist.linear.x = control_linear
                twist.angular.z = control_angular
                
                cmd_vel_pub.publish(twist)

                current_time = rospy.get_time() - start_time
                linear_errors.append(linear_error)
                angular_errors.append(angle_error)
                time_stamps.append(current_time)

                if linear_error < 0.3 and target_index <= points:
                    target_index += 1
                    # rospy.loginfo(target_index)
                # for i in range(len(current_path.poses)):
                #     px = current_path.poses[i].x
                #     py = current_path.poses[i].y

                #     dist = math.hypot(px - current_x, py - current_y)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        
        plt.pause(0.01) 
        rate.sleep()

    plt.show()

if __name__ == '__main__':
    main()

