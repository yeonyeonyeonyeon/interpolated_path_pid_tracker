import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs

#goal의 위치 받기
def goal_callback(msg):
    global tf_buffer  #트랜스폼 정보 가져오기

    transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time()) # map에서 base까지 트랜스폼 찾기
    robot_pose = PoseStamped()  #base프레임
    robot_pose.header.stamp = rospy.Time.now() 
    robot_pose.header.frame_id = "base_link"

    transformed_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)   #현재위치 알기
        
    path = interpolate_path(transformed_pose, msg) #보간법
    path_pub.publish(path) #토픽



#선형 보간법
def interpolate_path(start, end, resolution=0.1):
    path = Path()
    path.header.frame_id = "map"  
    
    #직선거리를 resolutiond으로 나누어 점의 개수 계산 = num_point
    points = int(((end.pose.position.x - start.pose.position.x) ** 2 + 
                      (end.pose.position.y - start.pose.position.y) ** 2) ** 0.5 / resolution)
    
    # start와 end 점 사이에 위에 개수개의 보간된 점 꼐산
    for i in range(points + 1):
        interpolated_pose = PoseStamped()  #보간된 점의 위치 저장
        alpha = float(i) / points      #alpha 값 보간된 점의 위치 찾기 위해 쓰이는 변수
        interpolated_pose.pose.position.x = start.pose.position.x * (1.0 - alpha) + end.pose.position.x * alpha
        interpolated_pose.pose.position.y = start.pose.position.y * (1.0 - alpha) + end.pose.position.y * alpha
        path.poses.append(interpolated_pose) #보간된 점의 위치를 path 객체에 poses리스트에 추가

    return path




if __name__ == '__main__':
    rospy.init_node('global_path_planner')  

    path_pub = rospy.Publisher('interpolated_plan', Path,queue_size=10)  # 보간법 경로 토픽
    goal_sub = rospy.Subscriber('topic/goal', PoseStamped, goal_callback)  #목표위치 받기

    tf_buffer = tf2_ros.Buffer()  #트랜스폼(상대적 위치) 저장
    tf_listener = tf2_ros.TransformListener(tf_buffer)  #새로운 트랜스폼 저장 리스너

    rospy.spin()