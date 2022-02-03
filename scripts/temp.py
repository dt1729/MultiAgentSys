import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

rospy.init_node("temp")
a = rospy.wait_for_message("/gazebo/model_states",ModelStates)

# wamv2 pos
print(a.pose[len(a.pose)-1])

#wamv pos
print(a.pose[len(a.pose)-2])

pub = rospy.Publisher('/wamv2/thrusters/left_thrust_cmd',Float32,queue_size=10)


while True:
    pub.publish(Float32(10))
