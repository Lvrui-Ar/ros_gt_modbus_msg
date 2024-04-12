# -*- coding: utf-8 -*-

import csv
import json


def read_csv(file_name):
    csv_data = []
    with open(file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            csv_data.append(row)
    csv_data_filtered = [row[1:3] + [row[7]] for row in csv_data][2:]
    mapping = process_csv_data(csv_data_filtered)
    return mapping


def process_csv_data(data):
    register = {}
    for item in data:
        # ic(item)
        register[item[2]] = {
            "Address": item[0][1:],
            "Type": item[1][1:],
            "num": 2
        }
    return register


def json_setting(data1, data2):
    plc_json = {}
    plc_json["host"] = ""
    plc_json["port"] = "502"
    plc_json["mapping"] = {"write_register":data1, "read_register":data2}
    # 指定要保存的 JSON 文件路径
    json_file_path = 'mapping.json'

    # 使用 json.dumps() 将字典转换为 JSON 格式的字符串
    json_string = json.dumps(plc_json, indent=4)

    # 将 JSON 字符串写入文件
    with open(json_file_path, 'w') as json_file:
        json_file.write(json_string)

    print(f"JSON 文件已保存到 {json_file_path}")


if __name__ == '__main__':
    file_name1 = "../../doc/Address_table.csv"
    mapping_w = read_csv(file_name1)

    file_name2 = "../../doc/Address_table2.csv"
    mapping_r = read_csv(file_name2)

    json_setting(mapping_w, mapping_r)



