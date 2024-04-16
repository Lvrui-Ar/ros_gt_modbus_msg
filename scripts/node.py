import rospy
from ros_gt_modbus_msg.msg import Control
from ros_gt_modbus_msg.msg import Ctrl_detail
from ros_gt_modbus_msg.msg import Ctrl_np
from wrapper_modbus.PLC_ModbusClient import PLC_ModbusClient


def callback_axis(msg):
    # mode = msg.mode
    # x_pol = msg.x_axis.pol
    # x_vel = msg.x_axis.val
    # x_acc = msg.x_axis.acc
    # y_pol = msg.y_axis.pol
    # y_vel = msg.y_axis.val
    # y_acc = msg.y_axis.acc
    # value = [[x_pol, x_vel, x_acc], [y_pol, y_vel, y_acc]]

    mode = msg.mode
    x_axis = msg.x_axis
    y_axis = msg.y_axis
    value = [[x_axis.pol, x_axis.vel, x_axis.acc], [y_axis.pol, y_axis.vel, y_axis.acc]]

    client.command_anaylsis(mode, value)

def callback_np(msg):
    mode = msg.mode
    strs = msg.strs
    value = [[int(strs[0]),int(strs[1])]]

    client.command_anaylsis(mode, value)


if __name__ == '__main__':
    rospy.init_node('node_modbus')

    path = ""
    client = PLC_ModbusClient(path)

    sub_wp = rospy.Subscriber('sub_move_wp', Control, callback_axis)
    sub_np = rospy.Subscriber('sub_move_np', Ctrl_np, callback_np)
    rospy.spin()

