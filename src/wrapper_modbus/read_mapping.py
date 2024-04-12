import json

class ReadMapping:
    def __init__(self, file_path1, file_path2):
        self._initialize_properties(file_path1, file_path2)

    def _initialize_properties(self, file_path1, file_path2):
        """初始化类的属性"""
        try:
            opera_content = self._read_file(file_path1)
            json_mapping = self._validate_json(self._read_file(file_path2))
        except Exception as e:
            # 日志记录或其他错误处理机制
            print(f"初始化失败: {e}")
            raise

        self.opera = opera_content
        self.json_mapping = json_mapping
        self.host = json_mapping.get("host")
        self.port = json_mapping.get("port")
        self.write_register = json_mapping.get("mapping", {}).get("write_register")
        self.read_register = json_mapping.get("mapping", {}).get("read_register")

    @staticmethod
    def _read_file(self, file_path)-> object:
        """读取并返回文件内容"""
        with open(path, "r") as j:
            data = json.load(j)
        return data

    def _validate_json(self, json_string):
        """验证并解析JSON字符串"""
        try:
            return json.loads(json_string)
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
            raise