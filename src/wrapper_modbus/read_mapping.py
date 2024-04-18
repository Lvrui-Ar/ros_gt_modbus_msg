import json
from wrapper_modbus.PLC_csv2json import CsvToJson

class ReadMapping:
    def __init__(self, file_path1):
        self._initialize_properties(file_path1)

    def _initialize_properties(self, file_path1,file_path2="./Address_table.json"):
        """初始化类的属性"""

        csv_to_json = CsvToJson()

        try:
            opera_content = self._read_file(file_path1)
            json_mapping = self._read_file(file_path2)
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
    def _read_file(self, file_path)-> object:
        """读取并返回文件内容"""
        with open(path, "r") as j:
            data = json.load(j)
        return data



