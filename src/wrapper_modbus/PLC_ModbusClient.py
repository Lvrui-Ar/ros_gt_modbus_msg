import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from wrapper_modbus.read_mapping import ReadMapping
import os
import time
# from icecream import ic


class PLC_ModbusClient:
    def __init__(self):
        """
        初始化PLC_ModbusClient类。
        
        :param path1: 读取映射文件的路径。
        """
        self.remapping = ReadMapping()
        self.client = BaseModbusClient(self.remapping.host, self.remapping.port)
        
        # 单轴是否移动控制 参数：
        self.move = 1
        self.max_wait_time = 60  # 最大等待时间，单位为秒
        self.waited_time = 0  # 已等待的时间
    
    def _data_validation(self,data,lower,upper):
        try:
            ic(data,lower,upper)
            if  lower<= data and data <= upper:
                rospy.loginfo("位置参数 验证通过")
            else:
                raise ValueError(f"{data} 不在 [{lower}, {upper}] 范围内")          
        except ValueError as e1:
            rospy.logerr("位置参数 验证失败")
            raise e1

    def command_anaylsis(self, mode, value):
        """
        根据给定的模式和值进行命令分析与执行。
        
        :param mode: 指定的模式，用于确定要操作的寄存器或配置。
        :param value: 要写入寄存器或配置的值，可以是序列。
        """
        # 根据模式从映射中获取操作列表
        mode_list = self.remapping.opera["write_register"][str(mode)]

        self._data_validation(value[0][0],
                              self.remapping.itinerary_setting_x[0],
                              self.remapping.itinerary_setting_x[1])
        self._data_validation(value[1][0],
                              self.remapping.itinerary_setting_y[0],
                              self.remapping.itinerary_setting_y[1])

        value = self._norm_command(value)

        # 遍历模式列表，并对每个模式执行单次写入操作
        for i in range(len(mode_list)):
            self.single_write_operation(mode_list[i], value[i])
            rospy.loginfo("单次写入操作完成")
            self.movement_condition()
        rospy.loginfo("写入操作完成")

    def _norm_command(self, value):
        """
        标准化命令参数列表。

        该方法将输入的value列表中的每个项目转换为一个包含六个元素的新列表，
        其中项目原来的三个元素分别占据新列表的第二个、第四个和第六个位置。
        初始的前两个和后两个元素被设为0。

        参数:
        value - 一个包含命令参数的列表，每个参数自身是一个包含三个元素的列表。

        返回值:
        返回一个新列表，其中包含了经过标准化处理的命令参数。
        """
        values = []
        for item in value:
            items = []
            for v in item:
                items += self._processdata(v) 
            values.append(items)
        return values

    def _processdata(self, data):
        """
        处理数据，将其转换为二进制并分割为两个部分。
        
        参数:
        - data: 需要处理的整数数据。
        
        返回值:
        - 一个包含两个整数的列表，分别是处理后的数据的前16位和后16位二进制转换后的结果。
        """

        # 判断正负
        if data >= 0:
            binary_str = bin(data)[2:].zfill(32)  # 正数直接转换为32位二进制数
        else:
            # 负数需要转换为补码形式，并确保补码长度为32位
            binary_str = bin(data & 0xffffffff)[2:].zfill(32)
        
        # 将二进制字符串分割并转换为整数
        data1 = int(binary_str[:16],2)  # 前16位
        data2 = int(binary_str[16:],2)  # 后16位
        return [data2, data1] 
    
    def single_write_operation(self, func_name, value):
        """
        根据给定的函数名称和值进行单次写入操作。
        
        :param func_name: 指定的函数名称，用于确定要写入的寄存器。
        :param value: 要写入的寄存器值，可以是序列。
        """
        address = self.remapping.write_register[func_name]["Address"]
        rospy.loginfo(f"func:{func_name}, address:{address}, value:{value}")
        self.client._writeRegisters(address, value)

    def movement_condition(self):
        while self.move != 1 and self.waited_time < self.max_wait_time:
            time.sleep(1)  # 每次等待1秒钟
            self.waited_time += 1

        if self.move == 1:
            pass
        else:
            rospy.logwarn("已达到最大等待时间,但move的值仍不为1,退出等待")
            self.command_anaylsis("0", [[0], [0]])

    def single_read_operation(self, func_name,mul):
        """
        根据给定的函数名称读取寄存器。
        
        :param func_name: 指定的函数名称，用于确定要读取的寄存器。
        :return: 读取的寄存器值，可以是序列。
        """
        address = self.remapping.read_register[func_name]["Address"]
        num_registers = self.remapping.read_register[func_name]["num"] * mul
        rospy.loginfo(f"func:{func_name}, address:{address}")

        return self.client.readRegisters(address, num_registers)

    def read_status(self):
        register_tmp = []
        for key, values in self.remapping.read_register.items():
            tmp = self.single_read_operation(key,1)
            register_tmp.append([key,tmp])
        return register_tmp

    def read_status2(self):
        first_key, first_value = next(iter(self.remapping.read_register.items()))
        tmp = self.single_read_operation(first_key,len(self.remapping.read_register))
        
        register_tmp = [tmp[i:i+2] for i in range(0, len(tmp), 2)]        
        return register_tmp
