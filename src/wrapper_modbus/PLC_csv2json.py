import csv
import json

class CsvToJson:
    def __init__(self):
        file_name1 = "../../doc/Address_table.csv"
        mapping_w = self._read_csv(file_name1)

        file_name2 = "../../doc/Address_table2.csv"
        mapping_r = self._read_csv(file_name2)

        self._json_setting(mapping_w, mapping_r)

    def _read_csv(self, file_name):
        with open(file_name, 'r') as csv_file:
            csv_data = csv.reader(csv_file, delimiter=',')
            csv_data_filtered = [row[1:3] + [row[7]] for row in csv_data][2:]
        mapping = self._process_csv_data(csv_data_filtered)
        return mapping

    def _process_csv_data(self, data):
        register = {}
        for item in data:
            # 确保数据处理逻辑正确，这里假设item[2]是唯一的键
            register[item[2]] = {
                "Address": item[0][1:],
                "Type": item[1][1:],
                "num": 2  # 静态值，如果需要可以根据数据动态设置
            }
        return register

    def _json_setting(self, data1, data2):
        plc_json = {
            "host": "",
            "port": "502",
            "axis_num": 3,
            "mapping": {"write_register": data1, "read_register": data2}
        }
        json_file_path = 'Address_table.json'

        # 将字典转换为 JSON 格式的字符串并写入文件
        with open(json_file_path, 'w') as json_file:
            json.dump(plc_json, json_file, indent=4)

        print(f"JSON 文件已保存到 {json_file_path}")


    

# c = CsvToJson()



