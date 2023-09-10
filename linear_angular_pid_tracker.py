#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
import math
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import matplotlib.pyplot as plt
import matplotlib.animation as animation


current_path = None
tf_buffer = None

linear_errors = []
time_stamps = []

fig, (ax1) = plt.subplots(1, 1)



# 현재, 목표 위치
def goal_callback(msg, path_pub):
    global current_path, tf_buffer, goal_x, goal_y

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
    global target_theta
    global slope
    global y_intercept
    global path_vector_x, path_vector_y
    global begin

    begin = 1
    path = Path()
    path.header.frame_id = "map"
    points = int(((end.pose.position.x - start.pose.position.x)**2 + (end.pose.position.y - start.pose.position.y)**2)**0.5 / resolution)
    target_theta = math.atan2(end.pose.position.y - start.pose.position.y, 
                               end.pose.position.x - start.pose.position.x)
                               
    
    #시작점 끝점 벡터
    path_vector_x = end.pose.position.x - start.pose.position.x
    path_vector_y = end.pose.position.y - start.pose.position.y

    #시작점 끝점 이은 함수
    if(end.pose.position.x != start.pose.position.x and end.pose.position.y != start.pose.position.y):
       if (end.pose.position.x == start.pose.position.x):
        slope = None 
        y_intercept = None 
       elif (end.pose.position.y == start.pose.position.y):
        slope = None 
        y_intercept = None 
       else:
        #기울기
        slope = (end.pose.position.y - start.pose.position.y) / (end.pose.position.x - start.pose.position.x)
        #y 절편
        y_intercept = start.pose.position.y - slope * start.pose.position.x

    # rospy.loginfo("slope = %s, y_intercept = %s", slope, y_intercept)
    

    
    quat = quaternion_from_euler(0,0,target_theta)

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
    global begin
    begin = 0
    rospy.loginfo("Shutting down")
    plt.close('all')
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



#그래프
def update_graph(i):
    ax1.clear()
    
    ax1.plot(time_stamps, linear_errors, label='linear Error')
    ax1.set_title('linear_Error')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Linear Error')
    

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



def main():
    global current_path, tf_buffer, goal_x, goal_y, cmd_vel_pub  
    global points
    global target_theta
    global slope
    global y_intercept
    global path_vector_x, path_vector_y
    global begin

    #초기화
    begin = 0

    MAX_ANGLE = 0.5
    MAX_SPEED = 0.4
    
    points = 0

    current_x = 0.0
    current_y = 0.0
    current_theta = 0.0

    target_x = 0.0
    target_y = 0.0
    target_theta = 0.0

    goal_x = 0.0
    goal_y = 0.0
    goal_distance = 0.0
    
    path_vector_x, path_vector_y = 0.0, 0.0
    robot_vector_x, robot_vector_y = 0.0, 0.0
    robot_position = 0.0
    c = 0
    
    linear_error = 0.0
    angular_error = 0.0
     
    control_linear = 0.0
    control_angular = 0.0
    control_robot = 0.0

    slope = 0.0 
    y_intercept = 0.0

    rospy.init_node('interpolation_tracker')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    path_pub = rospy.Publisher('interpolated_plan', Path, queue_size=1)
    goal_sub = rospy.Subscriber('topic/goal', PoseStamped, lambda msg: goal_callback(msg, path_pub))
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    pid_linear = PID(5, 0, 0)
    pid_angular = PID(5, 0, 0)

    rate = rospy.Rate(10)

    ani = animation.FuncAnimation(fig, update_graph, interval=1000)
    start_time = rospy.get_time()

    rospy.on_shutdown(shutdown_hook)
    
    rospy.loginfo(begin)

    
    while not rospy.is_shutdown():
        if  begin == 1:    
            #계산
            try:
                # 현재 로봇의 위치값
                transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y


                #오일러
                #현재 로봇의 theata 값
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                (_, _, current_theta) = euler_from_quaternion(quaternion)
                # rospy.loginfo("target_theta = %s, current_theta = %s", target_theta, current_theta)  

                
                #target 위치 값
                target_x = (current_x + slope * current_y - slope * y_intercept)/(slope ** 2 + 1)
                target_y = (slope ** 2*current_y + slope * current_x + y_intercept)/(slope ** 2 + 1)
                # rospy.loginfo("current_position = (%s,%s)", current_x,current_y)
                # rospy.loginfo("target_position = (%s,%s)", target_x,target_y)

                #로봇 vector 계산
                robot_vector_x = current_x - target_x
                robot_vector_y = current_y - target_y
                # rospy.loginfo("path_vertor = (%s,%s)", path_vector_x,path_vector_y)
                # rospy.loginfo("robot_vertor = (%s,%s)", robot_vector_x,robot_vector_y)

                #외적 계산
                robot_position = (path_vector_x)*(robot_vector_y) - (path_vector_y)*(robot_vector_x)
                # rospy.loginfo("robot_position = %s", robot_position)

                if robot_position < 0.0:  # 경로 오른쪽
                    c = 1
                elif robot_position > 0.0:  # 경로 왼쪽
                    c = -1
                else:
                    c = 0
                # rospy.loginfo("c = %s", c)

                # 에러 계산
                linear_error = abs(slope * current_x - current_y + y_intercept ) / math.sqrt(slope**2 + 1)
                angular_error = target_theta - current_theta
                goal_distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
                rospy.loginfo("linear_error = %s, angular_error = %s", linear_error, angular_error)  
                # rospy.loginfo("goal error %s", goal_error)
                
                
                # PID control
                control_linear = pid_linear.compute(linear_error )
                control_angular = pid_angular.compute(angular_error)
                rospy.loginfo("conrol_linear = %s, control_angular = %s", control_linear, control_angular)
                
                #결합
                control_robot = control_linear * c + control_angular
                rospy.loginfo("control_robot = %s", control_robot)
                if abs(control_linear) > abs(control_angular):
                    control_robot = abs(max(min(control_robot, MAX_ANGLE), -MAX_ANGLE)) * c
                else:
                    control_robot = max(min(control_robot, MAX_ANGLE), -MAX_ANGLE)
                rospy.loginfo("control_robot = %s", control_robot)
                rospy.loginfo("")
                
                

            except Exception as e:
                rospy.loginfo("fail to transform")
                rospy.sleep(1)
                continue
            

                #도착 했을 때
            if goal_distance < 0.1:
                rospy.loginfo("완료")
                twist = Twist()  
                cmd_vel_pub.publish(twist)
                break

                            
            current_time = rospy.get_time() - start_time
            linear_errors.append(linear_error)
            time_stamps.append(current_time)

            try:
                twist = Twist()
                twist.angular.z = control_robot
                twist.linear.x = MAX_SPEED
                
                cmd_vel_pub.publish(twist)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        
        else:
            rospy.loginfo("no start")
            rospy.sleep(1)

        plt.pause(0.01) 
        rate.sleep()       

    plt.show()


if __name__ == '__main__':
    main()

