import csv
import sys
import os
import re
import shutil

try:
    from importlib import resources
    try:
        # Python 3.9+
        from importlib.resources import files as resource_files
        _HAS_RESOURCE_FILES = True
    except ImportError:
        # Python 3.7-3.8
        _HAS_RESOURCE_FILES = False
except ImportError:
    _HAS_RESOURCE_FILES = False

try:
    import pkg_resources
    _HAS_PKG_RESOURCES = True
except ImportError:
    _HAS_PKG_RESOURCES = False


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
    """拷贝fprotocol相关文件到输出目录，从包内读取"""
    fprotocol_h_src = None
    fprotocol_c_src = None
    
    # 方法1: 使用 pkg_resources 从包内读取（推荐，兼容性最好）
    if _HAS_PKG_RESOURCES:
        try:
            fprotocol_h_path = pkg_resources.resource_filename('fprotocol', 'fprotocol.h')
            fprotocol_c_path = pkg_resources.resource_filename('fprotocol', 'fprotocol.c')
            if os.path.exists(fprotocol_h_path) and os.path.exists(fprotocol_c_path):
                fprotocol_h_src = fprotocol_h_path
                fprotocol_c_src = fprotocol_c_path
        except Exception:
            pass
    
    # 方法2: 使用 importlib.resources (Python 3.9+)
    if not fprotocol_h_src and _HAS_RESOURCE_FILES:
        try:
            from importlib.resources import files
            import fprotocol
            fprotocol_package = files(fprotocol)
            h_file = fprotocol_package / 'fprotocol.h'
            c_file = fprotocol_package / 'fprotocol.c'
            if h_file.exists() and c_file.exists():
                # 如果包未压缩，可以直接访问路径
                h_path = str(h_file)
                c_path = str(c_file)
                if os.path.exists(h_path) and os.path.exists(c_path):
                    fprotocol_h_src = h_path
                    fprotocol_c_src = c_path
        except Exception:
            pass
    
    # 方法3: 从包安装位置直接查找（fallback）
    if not fprotocol_h_src:
        try:
            import fprotocol
            package_dir = os.path.dirname(fprotocol.__file__)
            h_path = os.path.join(package_dir, 'fprotocol.h')
            c_path = os.path.join(package_dir, 'fprotocol.c')
            if os.path.exists(h_path) and os.path.exists(c_path):
                fprotocol_h_src = h_path
                fprotocol_c_src = c_path
        except Exception:
            pass
    
    # 拷贝fprotocol.h
    fprotocol_h_dst = os.path.join(output_directory, 'fprotocol.h')
    if fprotocol_h_src and os.path.exists(fprotocol_h_src):
        shutil.copy2(fprotocol_h_src, fprotocol_h_dst)
        print(f"已拷贝 fprotocol.h 到 {output_directory}")
    else:
        print(f"错误: 找不到 fprotocol.h 源文件（请在包内包含该文件）")
        return
    
    # 拷贝fprotocol.c或fprotocol.cpp
    if code_type == 'cpp':
        fprotocol_c_dst = os.path.join(output_directory, 'fprotocol.cpp')
    else:
        fprotocol_c_dst = os.path.join(output_directory, 'fprotocol.c')
    
    if fprotocol_c_src and os.path.exists(fprotocol_c_src):
        shutil.copy2(fprotocol_c_src, fprotocol_c_dst)
        print(f"已拷贝 fprotocol.{code_type} 到 {output_directory}")
    else:
        print(f"错误: 找不到 fprotocol.c 源文件（请在包内包含该文件）")

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
    parts = input_file_content.split('---')
    if len(parts) >= 2:
        struct_definition_csv = parts[0]
        csv_data = parts[1]
    else:
        # 如果只有一个部分，假设没有结构定义
        struct_definition_csv = ""
        csv_data = parts[0] if parts else ""
    
    print(struct_definition_csv, csv_data)
    # 解析CSV内容
    csv_reader = csv.reader(csv_data.strip().splitlines())
    data_list = []
    for row in csv_reader:
        if len(row) >= 3:
            # 清理每个字段的空白字符
            row = [field.strip() for field in row]
            # 确保有4列：index, data_type, var_name, callback_flag
            while len(row) < 4:
                row.append('0')  # 默认不回调
            data_list.append(row)
    
    print(f"解析的数据列表: {data_list}")
    
    struct_list = []
    if struct_definition_csv.strip():
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
        if len(row) < 3:
            continue  # 跳过不完整的行
        struct_name, ttype, name = row[0], row[1], row[2]
        length = row[3] if len(row) > 3 else ""
        
        length = length.strip()
        struct_name = struct_name.strip()
        ttype = ttype.strip()
        name = name.strip()
        
        if struct_name not in struct_maps.keys():
            struct_maps[struct_name] = []
            index = -1
        if length:
            new_name = f"_{name}_size"
            struct_maps[struct_name].append(('uint16_t', new_name, None, None))
            index += 1
            struct_maps[struct_name].append((ttype, name, length, index))
            index += 1
            continue
        index += 1
        struct_maps[struct_name].append((ttype, name, length, None))

    def struc2cdef(struct_name, struct):
        astr = 'typedef struct {\n'
        for ttype, name, length, size_index in struct:
            if length:
                astr += f'    {ttype} {name}[{length}];\n'
            else:
                astr += f'    {ttype} {name};\n'
        astr += f'}} __attribute__((packed))  {struct_name};'
        return astr

    for struct_name,struct in struct_maps.items():
        h_file_content += struc2cdef(struct_name,struct) + "\n\n"

    print(h_file_content)

    for row in data_list:
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        h_file_content += f"extern {data_type} {var_name}; /*Index: {index} */\n"

    for row in data_list:
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        # 清理callback_flag，只取第一个字符
        callback_flag = callback_flag[0] if callback_flag else '0'
        if callback_flag == '1':
            h_file_content += (
                f"int16_t callback_{var_name}(uint16_t type, uint32_t from, uint16_t error_code);\n"
            )

    for row in data_list:
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
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
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        c_file_content += f"{data_type} {var_name};\n"

    for struct_name,structs in struct_maps.items():
        # h_file_content += struc2cdef(struct_name,struct) + "\n\n"
        c_file_content+= generate_struct_descriptor(struct_name,structs)
        # print(struct_name,struct,)


    data_types_array = {}
    for row in data_list:
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        if data_type not in struct_maps.keys():
            data_types_array[data_type] = None
        print(data_types_array.keys())
    c_file_content += generate_base_type_descriptor(data_types_array.keys())


    # 定义数据表
    c_file_content += "\nfprotocol_data data_table[] = {\n"
    for i, row in enumerate(data_list):
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        # 清理callback_flag，只取第一个字符
        callback_flag = callback_flag[0] if callback_flag else '0'
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
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        c_file_content += f"    case {index}:\n        return &data_table[{i}];\n        break;\n"
    c_file_content += "    default:\n        break;\n    }\n    return NULL;\n}\n"



    # 读取写入函数生成
    for row in data_list:
        index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
        struct_desc  = "NULL" if index2struct_desc[index] == 'NULL' else f"{index2struct_desc[index]}" 
        c_file_content += (
            f"""void write_{var_name}(fprotocol_handler *handler,uint16_t node,uint8_t response)\n{{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, {index}, &{var_name}, sizeof({var_name}),{struct_desc});\n}}\n""")
        c_file_content += (
            f"""void read_{var_name}(fprotocol_handler *handler,uint16_t node)\n{{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, {index}, &{var_name}, 0,{struct_desc});\n}}\n""")

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
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python gencode_c.py <input_file> <output_directory> [code_type]")
        print("  code_type: 'c' for C code (default), 'cpp' for C++ code")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_directory = sys.argv[2]
    code_type = sys.argv[3] if len(sys.argv) == 4 else 'c'
    
    if code_type not in ['c', 'cpp']:
        print("Error: code_type must be 'c' or 'cpp'")
        sys.exit(1)
    
    generate_c_code(input_file, output_directory, code_type)


if __name__ == "__main__":
    main()