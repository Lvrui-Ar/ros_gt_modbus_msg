import rospy
from ros_gt_modbus_msg.msg import Control, CtrlDetail

if __name__ == '__main__':
    rospy.init_node('node_pub')
    pub = rospy.Publisher('sub_move_wp', Control, queue_size=10)