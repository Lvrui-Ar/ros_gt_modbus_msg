import json
from tools.PLC_csv2json import CsvToJson
import sys


class ReadMapping:
    def __init__(self):
        self._initialize_properties()

    def _initialize_properties(self):
        """初始化类的属性"""

        csv2json = CsvToJson()
        self.tablepath = csv2json.path_config.table_path
        self.mappingpath = csv2json.path_config.mapping_path

        try:
            opera_content = self._read_file(self.mappingpath)
            json_mapping = self._read_file(self.tablepath)
        except Exception as e:
            # 日志记录或其他错误处理机制
            print(f"初始化失败: {e}")
            raise
        
        self.opera = opera_content["mapping"]
        self.json_mapping = json_mapping
        self.host = json_mapping.get("host")
        self.port = json_mapping.get("port")
        self.axis_num = json_mapping.get("axis_num")
        self.write_register = json_mapping.get("mapping", {}).get("write_register")
        self.read_register = json_mapping.get("mapping", {}).get("read_register")

    @staticmethod
    def _read_file(file_path)-> object:
        """读取并返回文件内容"""
        with open(file_path, "r") as j:
            data = json.load(j)
        return data



