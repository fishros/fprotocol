import csv
import sys

# 输入文件内容
input_file_content = """"""
if len(sys.argv) != 3:
    print("Usage: python gen_code.py <input_file> <output_directory>")
    sys.exit(1)
input_file = sys.argv[1]
output_directory = sys.argv[2]
if output_directory=='.':
    output_directory = input_file[:input_file.rfind('/')]
file_name = sys.argv[1]
file_name = file_name[file_name.rfind('/')+1:].split('.')[0]
print(f"file_name: {file_name}", f"output_directory: {output_directory}")

with open(input_file, 'r') as file:
    input_file_content = ''.join([line for line in file if not line.strip().startswith('#')])

# 解析输入文件
struct_definition_csv, csv_data = input_file_content.split('---')
print(struct_definition_csv,csv_data)
# 解析CSV内容
csv_reader = csv.reader(csv_data.strip().splitlines())
data_list = list(csv_reader)
struct_list = list(csv.reader(struct_definition_csv.strip().splitlines()))
content = ""


# 生成.h文件内容
h_file_content = (
    f"#ifndef {file_name.upper()}_H\n"
    f"#define {file_name.upper()}_H\n\n"
    '#include "fprotocol.h"\n\n'
)

struct_maps = {}
for row in struct_list:
    struct_name,ttype,name,len = row
    if struct_name not in struct_maps.keys():
        struct_maps[struct_name] = []
    struct_maps[struct_name].append((ttype,name,len))

def struc2cdef(struct_name, struct):
    astr = 'typedef struct {\n'
    for ttype, name, len in struct:
        if len:
            astr += f'    {ttype} {name}[{len}];\n'
        else:
            astr += f'    {ttype} {name};\n'
    astr += f'}} __attribute__((packed))  {struct_name};'
    return astr

for struct_name,struct in struct_maps.items():
    h_file_content += struc2cdef(struct_name,struct) + "\n\n"

print(h_file_content)

for row in data_list:
    index, data_type, var_name, callback_flag = row
    h_file_content += f"extern {data_type} {var_name}; /*Index: {index} */\n"

for row in data_list:
    index, data_type, var_name, callback_flag = row
    if callback_flag == '1':
        h_file_content += (
            f"int16_t callback_{var_name}(uint16_t type, uint32_t from, uint16_t error_code);\n"
        )

for row in data_list:
    index, data_type, var_name, callback_flag = row
    h_file_content += (
        f"void write_{var_name}(fprotocol_handler *handler,uint16_t node,uint8_t response);\n"
        f"void read_{var_name}(fprotocol_handler *handler,uint16_t node);\n"
    )

h_file_content += f"\n\n\nextern fprotocol_get_index_info_t {file_name.lower()}_index_info;\n"
h_file_content += "\n#endif /* GENERATED_H */\n"

# 生成.c文件内容
c_file_content = f"""
#include \"{file_name}Proto.h\"

""".strip() + "\n\n"

# 定义变量
for row in data_list:
    _, data_type, var_name, _ = row
    c_file_content += f"{data_type} {var_name};\n"

# 定义数据表
c_file_content += "\nfprotocol_data data_table[] = {\n"
for i, row in enumerate(data_list):
    index, data_type, var_name, callback_flag = row
    callback_function = f"callback_{var_name}" if callback_flag == '1' else "NULL"
    c_file_content += f"    {{{index}, sizeof({var_name}), &{var_name}, {callback_function}}},\n"
c_file_content += "};\n\n"

# 实现fprotocol_get_index_info函数
c_file_content += f"fprotocol_data *{file_name.lower()}_fprotocol_get_index_info(uint16_t index)\n"
c_file_content += "{\n    switch (index)\n    {\n"
for i, row in enumerate(data_list):
    index, _, _, _ = row
    c_file_content += f"    case {index}:\n        return &data_table[{i}];\n        break;\n"
c_file_content += "    default:\n        break;\n    }\n    return NULL;\n}\n"

# 读取写入函数生成
for row in data_list:
    index, data_type, var_name, callback_flag = row
    c_file_content += (
        f"""void write_{var_name}(fprotocol_handler *handler,uint16_t node,uint8_t response)\n{{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, {index}, &{var_name}, sizeof({var_name}));\n}}\n""")
    c_file_content += (
        f"""void read_{var_name}(fprotocol_handler *handler,uint16_t node)\n{{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, {index}, &{var_name}, sizeof({var_name}));\n}}\n""")

c_file_content += f"fprotocol_get_index_info_t {file_name.lower()}_index_info = {file_name.lower()}_fprotocol_get_index_info;"

# 保存到文件
def save_to_file(filename, content):
    with open(filename, 'w') as f:
        f.write(content)

save_to_file(f"{output_directory}/{file_name}Proto.h", h_file_content)
save_to_file(f"{output_directory}/{file_name}Proto.c", c_file_content)
