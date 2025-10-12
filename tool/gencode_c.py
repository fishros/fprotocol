import csv
import sys
import os
import re
import shutil


def gen_field_enum_type(ttype):
    mapping = {
        'uint8_t': 'TYPE_UINT8',
        'uint16_t': 'TYPE_UINT16',
        'uint32_t': 'TYPE_UINT32',
        'int8_t': 'TYPE_INT8',
        'int16_t': 'TYPE_INT16',
        'int32_t': 'TYPE_INT32',
        'float': 'TYPE_FLOAT',
    }
    return mapping.get(ttype, 'TYPE_STRUCT')  # 默认用嵌套处理

def generate_struct_descriptor(struct_name, fields):
    field_defs = []
    filed_count  = 0
    for idx, (ttype, name, length, size_field) in enumerate(fields):
        filed_count += 1
        enum_type = gen_field_enum_type(ttype)
        offset = f"offsetof({struct_name}, {name})"
        size_field_str = size_field if size_field is not None else -1
        array_len = length if length is not None else 0
        field_defs.append(f"    {{{offset}, {enum_type}, {size_field_str}, {array_len}}},")
    return f"""
static const FieldDescriptor {struct_name}_fields[] = {{
{chr(10).join(field_defs)}
}};

static const StructDescriptor {struct_name}_desc = {{
    .size = sizeof({struct_name}),
    .field_count = {filed_count},
    .fields = {struct_name}_fields,
}};
"""


def generate_base_type_descriptor(used_base_type):
    base_type_def = ""
    for base_type in used_base_type:
        base_type_def += f"""
static const FieldDescriptor {base_type}_fields[] = {{
0, {gen_field_enum_type(base_type)}, -1, 0
}};

static const StructDescriptor {base_type}_desc = {{
    .size = sizeof({base_type}),
    .field_count = 1,
    .fields = {base_type}_fields,
}};
    """
        print(base_type_def)
    return base_type_def

# ====================================================================================================================

def copy_fprotocol_files(output_directory, code_type):
    """拷贝fprotocol相关文件到输出目录"""
    # 获取当前脚本所在目录的父目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    src_dir = os.path.join(project_root, 'src')
    
    # 拷贝fprotocol.h
    fprotocol_h_src = os.path.join(src_dir, 'fprotocol.h')
    fprotocol_h_dst = os.path.join(output_directory, 'fprotocol.h')
    if os.path.exists(fprotocol_h_src):
        shutil.copy2(fprotocol_h_src, fprotocol_h_dst)
        print(f"已拷贝 fprotocol.h 到 {output_directory}")
    
    # 拷贝fprotocol.c或fprotocol.cpp
    fprotocol_c_src = os.path.join(src_dir, 'fprotocol.c')
    if code_type == 'cpp':
        fprotocol_c_dst = os.path.join(output_directory, 'fprotocol.cpp')
    else:
        fprotocol_c_dst = os.path.join(output_directory, 'fprotocol.c')
    
    if os.path.exists(fprotocol_c_src):
        shutil.copy2(fprotocol_c_src, fprotocol_c_dst)
        print(f"已拷贝 fprotocol.{code_type} 到 {output_directory}")

def generate_c_code(input_file, output_directory, code_type='c'):
    """生成C/C++代码的主函数"""
    if output_directory=='.':
        output_directory = os.getcwd()  # 使用当前工作目录
        print(output_directory)
    
    file_name = input_file
    if '/' in file_name:
        file_name = file_name[file_name.rfind('/')+1:].split('.')[0]
    elif '\\' in file_name:
        file_name = file_name[file_name.rfind('\\')+1:].split('.')[0]
    else:
        file_name = file_name.split('.')[0]
    
    # 确保文件名首字母大写
    file_name = file_name[0].upper() + file_name[1:] if file_name else file_name
    
    print(f"file_name: {file_name}", f"output_directory: {output_directory}")

    with open(input_file, 'r',encoding='utf-8') as file:
        input_file_content = ''.join([line for line in file if not line.strip().startswith('#')])

    # 删除注释
    input_file_content = re.sub(r'#.*', '', input_file_content)

    # 解析输入文件
    struct_definition_csv, csv_data = input_file_content.split('---')
    print(struct_definition_csv,csv_data)
    # 解析CSV内容
    csv_reader = csv.reader(csv_data.strip().splitlines())
    data_list = list(csv_reader)
    struct_list = list(csv.reader(struct_definition_csv.strip().splitlines()))
    content = ""

    # =======================================================================================================================
    # 生成.h文件内容
    h_file_content = (
        f"#ifndef {file_name.upper()}_H\n"
        f"#define {file_name.upper()}_H\n\n"
        '#include "fprotocol.h"\n\n'
    )


    struct_maps = {}
    index = 0
    for row in struct_list:
        struct_name,ttype,name,len = row
        len = len.strip()
        if struct_name not in struct_maps.keys():
            struct_maps[struct_name] = []
            index = -1
        if len:
            new_name = f"_{name}_size"
            struct_maps[struct_name].append(('uint16_t',new_name,None,None))
            index+=1
            struct_maps[struct_name].append((ttype,name,len,index))
            index+=1
            continue
        index+=1
        struct_maps[struct_name].append((ttype,name,len,None))

    def struc2cdef(struct_name, struct):
        astr = 'typedef struct {\n'
        for ttype, name, len ,size_index in struct:
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


    # =======================================================================================================================

    # 生成.c文件内容
    c_file_content = f"""
#include \"{file_name}Proto.h\"

""".strip() + "\n\n"
    index2struct_desc = {}

    # 定义变量
    for row in data_list:
        _, data_type, var_name, _ = row
        c_file_content += f"{data_type} {var_name};\n"

    for struct_name,structs in struct_maps.items():
        # h_file_content += struc2cdef(struct_name,struct) + "\n\n"
        c_file_content+= generate_struct_descriptor(struct_name,structs)
        # print(struct_name,struct,)


    data_types_array = {}
    for row in data_list:
        index, data_type, var_name, _ = row
        if data_type not in struct_maps.keys():
            data_types_array[data_type] = None
        print(data_types_array.keys())
    c_file_content += generate_base_type_descriptor(data_types_array.keys())


    # 定义数据表
    c_file_content += "\nfprotocol_data data_table[] = {\n"
    for i, row in enumerate(data_list):
        index, data_type, var_name, callback_flag = row
        callback_function = f"callback_{var_name}" if callback_flag == '1' else "NULL"
        struct_desc = f"&{data_type}_desc" if data_type in struct_maps.keys() else f"&{data_type}_desc"
        # print(data_type,struct_maps)
        c_file_content += f"    {{{index}, sizeof({var_name}), &{var_name}, {callback_function},{struct_desc}}},\n"
        index2struct_desc[index] = struct_desc
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
        struct_desc  = "NULL" if index2struct_desc[index] == 'NULL' else f"{index2struct_desc[index]}" 
        c_file_content += (
            f"""void write_{var_name}(fprotocol_handler *handler,uint16_t node,uint8_t response)\n{{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, {index}, &{var_name}, sizeof({var_name}),{struct_desc});\n}}\n""")
        c_file_content += (
            f"""void read_{var_name}(fprotocol_handler *handler,uint16_t node)\n{{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, {index}, &{var_name}, sizeof({var_name}),{struct_desc});\n}}\n""")

    c_file_content += f"fprotocol_get_index_info_t {file_name.lower()}_index_info = {file_name.lower()}_fprotocol_get_index_info;"


    # =======================================================================================================================================================================
    # 保存到文件
    def save_to_file(filename, content):
        with open(filename, 'w',encoding='utf-8') as f:
            f.write(content)

    # 根据类型选择文件后缀
    if code_type == 'cpp':
        c_suffix = '.cpp'
        h_suffix = '.h'
    else:  # c
        c_suffix = '.c'
        h_suffix = '.h'

    save_to_file(f"{output_directory}/{file_name}Proto{h_suffix}", h_file_content)
    save_to_file(f"{output_directory}/{file_name}Proto{c_suffix}", c_file_content)
    
    # 拷贝fprotocol相关文件
    copy_fprotocol_files(output_directory, code_type)


def main():
    """命令行入口点"""
    if len(sys.argv) != 3:
        print("Usage: python gen_code.py <input_file> <output_directory>")
        sys.exit(1)
    input_file = sys.argv[1]
    output_directory = sys.argv[2]
    generate_c_code(input_file, output_directory)


if __name__ == "__main__":
    main()