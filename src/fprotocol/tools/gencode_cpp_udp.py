import csv
import sys
import os
import re
import shutil

def gen_field_enum_type(ttype, is_cpp=False):
    if is_cpp:
        mapping = {
            'uint8_t': 'FProtocol::FieldType::UINT8',
            'uint16_t': 'FProtocol::FieldType::UINT16',
            'uint32_t': 'FProtocol::FieldType::UINT32',
            'int8_t': 'FProtocol::FieldType::INT8',
            'int16_t': 'FProtocol::FieldType::INT16',
            'int32_t': 'FProtocol::FieldType::INT32',
            'float': 'FProtocol::FieldType::FLOAT',
        }
        return mapping.get(ttype, 'FProtocol::FieldType::STRUCT')
    else:
        mapping = {
            'uint8_t': 'TYPE_UINT8',
            'uint16_t': 'TYPE_UINT16',
            'uint32_t': 'TYPE_UINT32',
            'int8_t': 'TYPE_INT8',
            'int16_t': 'TYPE_INT16',
            'int32_t': 'TYPE_INT32',
            'float': 'TYPE_FLOAT',
        }
        return mapping.get(ttype, 'TYPE_STRUCT')

def generate_struct_descriptor(struct_name, fields, is_cpp=False):
    field_defs = []
    filed_count = 0
    for idx, (ttype, name, length, size_field) in enumerate(fields):
        filed_count += 1
        enum_type = gen_field_enum_type(ttype, is_cpp)
        offset = f"offsetof({struct_name}, {name})"
        size_field_str = size_field if size_field is not None else -1
        array_len = length if length is not None and length != "" else 0
        
        if is_cpp:
            field_defs.append(f"        FProtocol::FieldDescriptor({offset}, {enum_type}, {size_field_str}, {array_len})")
        else:
            field_defs.append(f"    {{{offset}, {enum_type}, {size_field_str}, {array_len}}}")
    
    joined_fields = ",\n".join(field_defs)
    if is_cpp:
        return f"""
// Struct descriptor for {struct_name}
static std::vector<FProtocol::FieldDescriptor> {struct_name}_fields = {{
{joined_fields}
}};

static std::shared_ptr<FProtocol::StructDescriptor> {struct_name}_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof({struct_name}), {struct_name}_fields);
"""
    else:
        return f"""
static const FieldDescriptor {struct_name}_fields[] = {{
{joined_fields}
}};

static const StructDescriptor {struct_name}_desc = {{
    .size = sizeof({struct_name}),
    .field_count = {filed_count},
    .fields = {struct_name}_fields,
}};
"""
def generate_base_type_descriptor(used_base_type, is_cpp=False):
    base_type_def = ""
    for base_type in used_base_type:
        if is_cpp:
            base_type_def += f"""
static std::vector<FProtocol::FieldDescriptor> {base_type}_fields = {{
    FProtocol::FieldDescriptor(0, {gen_field_enum_type(base_type, is_cpp)}, -1, 0)
}};

static std::shared_ptr<FProtocol::StructDescriptor> {base_type}_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof({base_type}), {base_type}_fields);
"""
    return base_type_def

def copy_fprotocol_files(output_directory, code_type):
    """Copy fprotocol headers/sources to the output directory."""
    script_dir = os.path.dirname(os.path.abspath(__file__))

    possible_roots = [
        os.path.dirname(os.path.dirname(os.path.dirname(script_dir))),
        os.getcwd(),
    ]

    try:
        import fprotocol
        package_dir = os.path.dirname(fprotocol.__file__)
        possible_roots.insert(0, package_dir)
        print(f"Found fprotocol package dir: {package_dir}")
    except ImportError:
        print("fprotocol package not installed, trying project paths")

    src_h = None
    src_cpp = None

    for root in possible_roots:
        print(f"Check root: {root}")

        if code_type == 'cpp':
            test_h = os.path.join(root, 'src', 'fprotocol', 'fprotocol.hpp')
            test_cpp = os.path.join(root, 'src', 'fprotocol', 'fprotocol.cpp')
            if not os.path.exists(test_h):
                test_h = os.path.join(root, 'fprotocol.hpp')
                test_cpp = os.path.join(root, 'fprotocol.cpp')
        else:
            test_h = os.path.join(root, 'fprotocol.h')
            test_cpp = os.path.join(root, 'fprotocol.c')
            if not os.path.exists(test_h):
                test_h = os.path.join(root, 'src', 'fprotocol', 'fprotocol.h')
                test_cpp = os.path.join(root, 'src', 'fprotocol', 'fprotocol.c')

        print(f"  Check file: {test_h}")
        print(f"  Check file: {test_cpp}")

        if os.path.exists(test_h) and os.path.exists(test_cpp):
            src_h = test_h
            src_cpp = test_cpp
            print(f"Found source: {src_h}")
            print(f"Found source: {src_cpp}")
            break
        else:
            print("  Files not found")

    if not src_h or not src_cpp:
        print("Error: fprotocol source files not found")
        print("Ensure:")
        print("1) Run from project root, or")
        print("2) fprotocol is installed, or")
        print("3) src/fprotocol contains the source files")
        return False

    if code_type == 'cpp':
        dst_h = os.path.join(output_directory, 'fprotocol.hpp')
        dst_cpp = os.path.join(output_directory, 'fprotocol.cpp')
    else:
        dst_h = os.path.join(output_directory, 'fprotocol.h')
        dst_cpp = os.path.join(output_directory, 'fprotocol.c')

    try:
        shutil.copy2(src_h, dst_h)
        print(f"Copied {os.path.basename(dst_h)} to {output_directory}")

        shutil.copy2(src_cpp, dst_cpp)
        print(f"Copied {os.path.basename(dst_cpp)} to {output_directory}")
        return True
    except Exception as e:
        print(f"Copy failed: {e}")
        return False

def generate_c_code(input_file, output_directory, code_type='c'):
    """生成C/C++代码的主函数"""
    print(f"开始生成代码: {input_file} -> {output_directory}, 类型: {code_type}")
    
    if output_directory == '.':
        output_directory = os.getcwd()
    
    file_name = os.path.basename(input_file).split('.')[0]
    file_name = file_name[0].upper() + file_name[1:] if file_name else file_name
    
    print(f"file_name: {file_name}, output_directory: {output_directory}")

    with open(input_file, 'r', encoding='utf-8') as file:
        input_file_content = ''.join([line for line in file if not line.strip().startswith('#')])

    print(f"文件内容: {input_file_content}")
    
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
    
    print(f"CSV数据: {csv_data}")
    
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

    # 解析结构体定义
    struct_maps = {}
    index = 0
    for row in struct_list:
        if len(row) < 3:
            continue
        struct_name, ttype, name = row[0], row[1], row[2]
        len_val = row[3] if len(row) > 3 else ""
        len_val = len_val.strip()
        
        if struct_name not in struct_maps.keys():
            struct_maps[struct_name] = []
            index = -1
        if len_val:
            new_name = f"_{name}_size"
            struct_maps[struct_name].append(('uint16_t', new_name, None, None))
            index += 1
            struct_maps[struct_name].append((ttype, name, len_val, index))
            index += 1
            continue
        index += 1
        struct_maps[struct_name].append((ttype, name, len_val, None))

    def struc2cdef(struct_name, struct):
        astr = 'typedef struct {\n'
        for ttype, name, len_val, size_index in struct:
            if len_val:
                astr += f'    {ttype} {name}[{len_val}];\n'
            else:
                astr += f'    {ttype} {name};\n'
        astr += f'}} __attribute__((packed)) {struct_name};'
        return astr

    is_cpp = (code_type == 'cpp')
    print(f"是否为C++: {is_cpp}")

    # 生成.hpp文件内容
    if is_cpp:
        h_file_content = (
            f"#ifndef {file_name.upper()}_HPP\n"
            f"#define {file_name.upper()}_HPP\n\n"
            '#include "fprotocol.hpp"\n'
            '#include <memory>\n'
            '#include <functional>\n'
            '#include <cstddef>  // for offsetof\n\n'
            'using namespace FProtocol;\n\n'
        )
        
        # 添加结构体定义
        for struct_name, struct in struct_maps.items():
            h_file_content += struc2cdef(struct_name, struct) + "\n\n"
        
        # C++版本的类声明
        h_file_content += f"class {file_name}Protocol {{\n"
        h_file_content += "private:\n"
        
        # 成员变量
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            h_file_content += f"    {data_type} {var_name}_;\n"
        
        # 回调函数成员变量
        for row in data_list:
            if len(row) >= 4:
                index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
                # 清理callback_flag，只取第一个字符
                callback_flag = callback_flag[0] if callback_flag else '0'
                if callback_flag == '1':
                    h_file_content += f"    FProtocol::CallbackFunction {var_name}_callback_;\n"
        
        h_file_content += "\npublic:\n"
        h_file_content += f"    {file_name}Protocol();\n"
        h_file_content += f"    ~{file_name}Protocol() = default;\n\n"
        
        # Getter和Setter方法
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            h_file_content += f"    const {data_type}& get_{var_name}() const {{ return {var_name}_; }}\n"
            h_file_content += f"    void set_{var_name}(const {data_type}& value) {{ {var_name}_ = value; }}\n"
            h_file_content += f"    {data_type}* get_{var_name}_ptr() {{ return &{var_name}_; }}\n\n"
        
        # 回调函数设置方法
        for row in data_list:
            if len(row) >= 4:
                index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
                # 清理callback_flag，只取第一个字符
                callback_flag = callback_flag[0] if callback_flag else '0'
                if callback_flag == '1':
                    h_file_content += f"    void set_{var_name}_callback(FProtocol::CallbackFunction callback) {{ {var_name}_callback_ = callback; }}\n"
                    h_file_content += f"    template<typename T>\n"
                    h_file_content += f"    void set_{var_name}_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {{\n"
                    h_file_content += f"        {var_name}_callback_ = [obj, method](uint16_t type, uint8_t from, uint16_t error_code) -> int16_t {{\n"
                    h_file_content += f"            return (obj->*method)(type, from, error_code);\n"
                    h_file_content += f"        }};\n"
                    h_file_content += f"    }}\n"
        
        h_file_content += "\n"
        
        # 读写函数声明
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            h_file_content += f"    void write_{var_name}(FProtocol::Handler* handler, uint8_t node, bool response = false);\n"
            h_file_content += f"    void read_{var_name}(FProtocol::Handler* handler, uint8_t node);\n"
        
        h_file_content += f"\n    FProtocol::ProtocolData* getIndexInfo(uint16_t index);\n"
        h_file_content += f"    FProtocol::GetIndexInfoFunction getIndexInfoFunction();\n"
        h_file_content += "};\n\n"
        h_file_content += f"#endif /* {file_name.upper()}_HPP */\n"

    # 生成.cpp文件内容
    if is_cpp:
        c_file_content = f'#include "{file_name}Proto.hpp"\n'
        c_file_content += '#include <cstring>  // for memset\n\n'
        
        # 生成结构体描述符
        for struct_name, structs in struct_maps.items():
            c_file_content += generate_struct_descriptor(struct_name, structs, is_cpp)
        
        # 生成基础类型描述符
        data_types_array = set()
        for row in data_list:
            data_type = row[1]
            if data_type not in struct_maps.keys():
                data_types_array.add(data_type)
        c_file_content += generate_base_type_descriptor(data_types_array, is_cpp)
        
        # 构造函数实现
        c_file_content += f"\n{file_name}Protocol::{file_name}Protocol() {{\n"
        c_file_content += "    // Initialize member variables\n"
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            if data_type in ['uint8_t', 'uint16_t', 'uint32_t', 'int8_t', 'int16_t', 'int32_t']:
                c_file_content += f"    {var_name}_ = 0;\n"
            elif data_type == 'float':
                c_file_content += f"    {var_name}_ = 0.0f;\n"
            else:
                c_file_content += f"    memset(&{var_name}_, 0, sizeof({var_name}_));\n"
        
        # 初始化回调函数
        for row in data_list:
            if len(row) >= 4:
                index, data_type, var_name, callback_flag = row[0], row[1], row[2], row[3]
                # 清理callback_flag，只取第一个字符
                callback_flag = callback_flag[0] if callback_flag else '0'
                if callback_flag == '1':
                    c_file_content += f"    {var_name}_callback_ = nullptr;\n"
        
        c_file_content += "}\n\n"

        # getIndexInfo函数实现
        c_file_content += f"FProtocol::ProtocolData* {file_name}Protocol::getIndexInfo(uint16_t index) {{\n"
        c_file_content += "    static std::vector<std::unique_ptr<FProtocol::ProtocolData>> protocol_data_instances;\n"
        c_file_content += "    if (protocol_data_instances.empty()) {\n"
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            callback_flag = row[3] if len(row) >= 4 else '0'
            # 清理callback_flag，只取第一个字符
            callback_flag = callback_flag[0] if callback_flag else '0'
            
            if callback_flag == '1':
                callback_function = f"[this](uint16_t type, uint8_t from, uint16_t error_code) -> int16_t {{ return {var_name}_callback_ ? {var_name}_callback_(type, from, error_code) : 0; }}"
            else:
                callback_function = "nullptr"
            
            struct_desc = f"{data_type}_desc"
            c_file_content += f"        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>({index}, sizeof({var_name}_), &{var_name}_, {callback_function}, {struct_desc}));\n"
        c_file_content += "    }\n"
        c_file_content += "    \n"
        c_file_content += "    for (auto& data : protocol_data_instances) {\n"
        c_file_content += "        if (data->getIndex() == index) {\n"
        c_file_content += "            return data.get();\n"
        c_file_content += "        }\n"
        c_file_content += "    }\n"
        c_file_content += "    return nullptr;\n"
        c_file_content += "}\n\n"

        # getIndexInfoFunction函数实现
        c_file_content += f"FProtocol::GetIndexInfoFunction {file_name}Protocol::getIndexInfoFunction() {{\n"
        c_file_content += f"    return [this](uint16_t index) -> FProtocol::ProtocolData* {{ return this->getIndexInfo(index); }};\n"
        c_file_content += "}\n\n"

        # 读取写入函数生成
        for row in data_list:
            index, data_type, var_name = row[0], row[1], row[2]
            struct_desc = f"{data_type}_desc"
            c_file_content += (
                f"""void {file_name}Protocol::write_{var_name}(FProtocol::Handler* handler, uint8_t node, bool response) {{
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA,
                   {index}, &{var_name}_, sizeof({var_name}_), {struct_desc});
}}

""")
            c_file_content += (
                f"""void {file_name}Protocol::read_{var_name}(FProtocol::Handler* handler, uint8_t node) {{
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, {index}, nullptr, 0, {struct_desc});
}}

""")

    # 保存到文件
    def save_to_file(filename, content):
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(content)

    if code_type == 'cpp':
        save_to_file(f"{output_directory}/{file_name}Proto.hpp", h_file_content)
        save_to_file(f"{output_directory}/{file_name}Proto.cpp", c_file_content)
    
        # Generate UDP test main and CMakeLists.txt
        callback_setup_lines = []
        for row in data_list:
            if len(row) >= 4 and row[3] and row[3][0] == '1':
                var_name = row[2]
                callback_setup_lines.append(
                    (
                        "        protocol_.set_{}_callback([](uint16_t type, uint8_t from, uint16_t error_code) -> int16_t {{\\n"
                        "            std::printf(\\\"cb {}: type=0x%02X from=0x%02X err=0x%04X\\\\n\\\", type, from, error_code);\\n"
                        "            return 0;\\n"
                        "        }});\\n"
                    ).format(var_name, var_name)
                )
        callback_setup = "".join(callback_setup_lines)
        callback_setup = "".join(callback_setup_lines)
        callback_setup = "".join(callback_setup_lines)



    # 拷贝fprotocol相关文件
    copy_success = copy_fprotocol_files(output_directory, code_type)
    
    if copy_success:
        print(f"成功生成{'面向对象C++' if is_cpp else 'C'}代码到: {output_directory}")
    else:
        print(f"代码生成完成，但fprotocol源文件拷贝失败。请手动拷贝fprotocol.{'hpp' if is_cpp else 'h'}和fprotocol.{'cpp' if is_cpp else 'c'}文件到输出目录。")

def main():
    """CLI entry point."""
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print('Usage: python gencode_cpp_udp.py <input_file> <output_directory> [code_type]')
        print("  code_type: 'c' for C code (default), 'cpp' for C++ code")
        sys.exit(1)

    input_file = sys.argv[1]
    output_directory = sys.argv[2]
    code_type = sys.argv[3] if len(sys.argv) == 4 else 'c'

    if code_type not in ['c', 'cpp']:
        print("Error: code_type must be 'c' or 'cpp'")
        sys.exit(1)

    generate_c_code(input_file, output_directory, code_type)

if __name__ == '__main__':
    main()
