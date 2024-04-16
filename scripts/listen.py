import rospy
from ros_gt_modbus_msg.msg import Slide_table_status
from wrapper_modbus.PLC_ModbusClient import PLC_ModbusClient




if __name__ == '__main__':
    rospy.init_node('node_modbus')

    client = PLC_ModbusClient()

    

    
    pub = rospy.Publisher('Listener_modbus', Slide_table_status, queue_size=10)
    while not rospy.is_shutdown():
        pass
    rospy.spin()


