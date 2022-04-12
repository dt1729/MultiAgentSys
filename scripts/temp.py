import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import tf
import time
rospy.init_node("temp")
a = rospy.wait_for_message("/gazebo/model_states",ModelStates)

# wamv2 pos
print(a.pose[len(a.pose)-1])
print(tf.transformations.euler_from_quaternion([a.pose[len(a.pose)-1].orientation.x,a.pose[len(a.pose)-1].orientation.y,a.pose[len(a.pose)-1].orientation.z,a.pose[len(a.pose)-1].orientation.w]))
#wamv pos
print(a.pose[len(a.pose)-2])

pub = rospy.Publisher('/wamv2/thrusters/left_thrust_cmd',Float32,queue_size=10)
pub1 = rospy.Publisher('/wamv2/thrusters/right_thrust_cmd',Float32,queue_size=10)

prev_time = time.time()
orient = []
pos_x = []
pos_y = []
time_s = []
while time.time() - prev_time < 10:
    pub.publish(Float32(10))
    pub1.publish(Float32(10))
    a = rospy.wait_for_message("/gazebo/model_states",ModelStates)
    # wamv2 pos
    wamv2_pose = a.pose[len(a.pose)-1]
    time_s.append(time.time() - prev_time)
    pos_x.append(wamv2_pose.position.x)
    pos_y.append(wamv2_pose.position.y)
    wamv2_orientation = tf.transformations.euler_from_quaternion([a.pose[len(a.pose)-1].orientation.x,a.pose[len(a.pose)-1].orientation.y,a.pose[len(a.pose)-1].orientation.z,a.pose[len(a.pose)-1].orientation.w])[2]
    orient.append(wamv2_orientation)

plt.plot(orient,time_s)
plt.xlabel("time(s)")
plt.ylabel("radians")
plt.show()
plt.plot(pos_x,pos_y)
plt.xlabel("X co-ordinate(m)")
plt.ylabel("Y co-ordinate(m)")
plt.show()
