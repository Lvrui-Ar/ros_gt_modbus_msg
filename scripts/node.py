import rospy
from ros_gt_modbus_msg.msg import Control
from ros_gt_modbus_msg.msg import Ctrl_detail
from ros_gt_modbus_msg.msg import Ctrl_np


def callback_axis(msg):
    pass


def callback_np(msg):
    pass


if __name__ == '__main__':
    rospy.init_node('node_modbus')

    client = PLC_ModbusClient()

    sub_wp = rospy.Subscriber('sub_move_wp', Control, callback_axis)
    sub_np = rospy.Subscriber('sub_move_np', Ctrl_np, callback_np)
    rospy.spin()

