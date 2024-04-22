import rospy
from ros_gt_modbus_msg.msg import Control,CtrlDetail
from ros_gt_modbus_msg.msg import CtrlNoParam
from wrapper_modbus.PLC_ModbusClient import PLC_ModbusClient


def callback_axis(msg):
    mode = msg.mode
    x_pos = msg.X_axis.pos
    x_vel = msg.X_axis.vel
    x_acc = msg.X_axis.acc
    y_pos = msg.Y_axis.pos
    y_vel = msg.Y_axis.vel
    y_acc = msg.Y_axis.acc
    value = [[x_pos, x_vel, x_acc], [y_pos, y_vel, y_acc]]

    # mode = msg.mode
    # x_axis = msg.x_axis
    # y_axis = msg.y_axis
    # value = [[x_axis.pos, x_axis.vel, x_axis.acc], [y_axis.pos, y_axis.vel, y_axis.acc]]
    rospy.loginfo(f"mode:{mode},value:{value}")

    client.command_anaylsis(mode, value)

def callback_np(msg):
    mode = msg.mode
    strs = msg.strs
    value = [[int(strs[0])],[int(strs[1])]]
    rospy.loginfo(f"mode:{mode},value:{value}")


    client.command_anaylsis(mode, value)


if __name__ == '__main__':
    rospy.init_node('node_modbus')

    client = PLC_ModbusClient()
    rospy.loginfo(" node_modbus start!")

    sub_wp = rospy.Subscriber('sub_move_wp', Control, callback_axis)
    sub_np = rospy.Subscriber('sub_move_np', CtrlNoParam, callback_np)
    rospy.spin()

