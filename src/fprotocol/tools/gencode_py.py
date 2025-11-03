import csv
import sys
import re
import os
import shutil

def copy_fprotocol_file(output_directory):
    """复制fprotocol.py文件到输出目录"""
    # 获取当前脚本的目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 获取包根目录（当前脚本在tools目录下，所以需要上一级）
    package_root = os.path.dirname(current_dir)
    # fprotocol.py文件路径（现在在包根目录下）
    fprotocol_source = os.path.join(package_root, "fprotocol.py")
    fprotocol_dest = os.path.join(output_directory, "fprotocol.py")
    
    if os.path.exists(fprotocol_source):
        shutil.copy2(fprotocol_source, fprotocol_dest)
        print(f"已复制 fprotocol.py 到 {fprotocol_dest}")
    else:
        print(f"警告: 找不到源文件 {fprotocol_source}")

def generate_python_code(input_file, output_directory):
    """生成Python代码的主函数"""
    if output_directory=='.':
        output_directory = os.getcwd()  # 使用当前工作目录
    
    # 复制fprotocol.py文件
    copy_fprotocol_file(output_directory)
    
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

    with open(input_file, 'r', encoding='utf-8') as file:
        input_file_content = ''.join([line for line in file if not line.strip().startswith('#')])

    print(input_file_content)
    # 删除注释
    input_file_content = re.sub(r'#.*', '', input_file_content)


    # 解析输入文件
    struct_definition_csv, csv_data = input_file_content.split('---')

    # 解析CSV内容
    csv_reader = csv.reader(csv_data.strip().splitlines())
    data_list = list(csv_reader)
    struct_list = list(csv.reader(struct_definition_csv.strip().splitlines()))
    content = ""

    def get_type_str_len(stype,len):
        type2struct = {
            'uint32_t': "I",
            'uint8_t': "B",
            'int32_t': "i",
            'int16_t': "h",
            'uint16_t': "H",
            'int8_t': "b",
            'float': "f",
            'double': "d",
            'char': "c"
        }
        type2len = {
            'uint32_t': 4,
            'uint8_t': 1,
            'int32_t': 4,
            'int16_t': 2,
            'uint16_t': 2,
            'int8_t': 1,
            'float': 4,
            'double': 8,
            'char': 1
        }
        if len:
            len_int = int(len)  # 确保len是整数
            if stype == 'uint8_t':
                return f'{len_int}s', len_int
            elif stype == 'char':
                return f'{len_int}s', len_int  # char数组也使用s格式
            else:
                return f'{type2struct[stype]*len_int}',type2len[stype] * len_int
        else:
            return type2struct[stype], type2len[stype]
        
    def get_default_values(stype,len):
        type2defaultv = {
            'uint32_t': 0,
            'uint8_t': 0,
            'int32_t': 0,
            'int16_t': 0,
            'uint16_t': 0,
            'int8_t': 0,
            'float': 0.0,
            'double': 0.0,
            'char': "b'\x00'"
        }
        if len:
            len_int = int(len)  # 确保len是整数
            if stype == 'uint8_t':
                return """b'\\x00' * """ + str(len_int)
            elif stype == 'char':
                return f'b\'\\x00\' * {len_int}'  # char数组使用简洁格式
            else:
                return [type2defaultv[stype]] * len_int
                # return (type2defaultv[stype],) * len
        else:
            return type2defaultv[stype]


    struct_maps = {}
    for row in struct_list:
        struct_name,ttype,name,len = row
        len = len.strip()
        struct_name = struct_name.strip()
        ttype = ttype.strip()
        name = name.strip()
        if struct_name not in struct_maps.keys():
            struct_maps[struct_name] = []
        struct_maps[struct_name].append((ttype,name,len))

    def struct2class(struct_name, struct):
        # ==============计算 len 和 str================
        # struct_len = 0
        struct_str = ''
        desc_str = ""
        desc_str += f"{struct_name}_desc = ["
        for ttype, name, len in struct:
            # print(ttype, name, len)
            type_str, type_len = get_type_str_len(ttype, int(len) if len else None)
            if len:
                len_int = int(len)
                print(f"_{name}_size","H")
                desc_str += f'("_{name}_size","H",0),'  # 设置为0，让DynamicStruct自动设置
            print(name, type_str)
            desc_str += f'("{name}","{type_str}",{get_default_values(ttype,len)}),'
        print(desc_str)
        # desc_str[-1] = ']'
        desc_str = desc_str[:-1] + "]\n"
        return desc_str

    for struct_name,struct in struct_maps.items():
        content += struct2class(struct_name,struct)   

    # 定义类型映射字典（在使用前定义）
    type2struct = {
        'uint32_t': "I",
        'uint8_t': "B",
        'int32_t': "i",
        'int16_t': "h",
        'uint16_t': "H",
        'int8_t': "b",
        'float': "f",
        'double': "d",
        'char': "c"
    }

    sum_content = f"class {file_name}Proto:\n"

    for l in content.splitlines():
        sum_content += f"    {l}\n"
    #   content = f"import struct\n\n"



    sum_content += f'\n    def __init__(self):'
    sum_content += f'\n        self.index_table = {{}}'
    for addr, data_type, var_name, callback_flag in data_list:
        if data_type in struct_maps.keys():
            sum_content += f'\n        self.{var_name} = DynamicStruct(self.{data_type}_desc)'
        else:
            # 为基础类型创建 BaseValue 包装对象
            default_val = get_default_values(data_type, 0)
            format_char = type2struct.get(data_type, "B")
            sum_content += f'\n        self.{var_name} = BaseValue({default_val}, "{format_char}")'
        sum_content += f'\n        self.index_table[{addr}] = self.{var_name}'


    sum_content += f'\n\n    def get_index_data(self,index):'
    sum_content += f'\n        return self.index_table[index]'
    
    # 添加 __setattr__ 以支持直接赋值（如 obj.led = 1），保持 BaseValue 对象
    sum_content += f'\n\n    def __setattr__(self, name, value):'
    sum_content += f'\n        # 如果属性是 BaseValue 且已存在，则更新其值而不是替换对象'
    sum_content += f'\n        if hasattr(self, name) and isinstance(getattr(self, name), BaseValue):'
    sum_content += f'\n            getattr(self, name).value = value'
    sum_content += f'\n        else:'
    sum_content += f'\n            object.__setattr__(self, name, value)'

    for addr, data_type, var_name, callback_flag in data_list:
        sum_content += f'\n\n    def write_{var_name}(self,fprotocol,type,node):'
        if data_type in struct_maps.keys():
            sum_content += f'\n        bytes_data = self.{var_name}.to_bytes()'
        else:
            # 基础类型也使用 to_bytes() 方法
            sum_content += f'\n        bytes_data = self.{var_name}.to_bytes()'
        sum_content += f'\n        fprotocol.fprotocol_write(node,type,{addr},bytes_data,len(bytes_data))'
    
    for addr, data_type, var_name, callback_flag in data_list:
        sum_content += f'\n\n    def read_{var_name}(self,fprotocol,node):'
        sum_content += f'\n        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,{addr},[],0)'

    sum_content = 'import struct\nfrom fprotocol import DynamicStruct,FProtocolType,BaseValue\n' + sum_content

    # 保存到文件
    print(sum_content)
    def save_to_file(filename, sum_content):
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(sum_content)
        print(f"Saved to {filename}")
    save_to_file(f"{output_directory}/{file_name.lower()}_proto.py",sum_content )


def main():
    """命令行入口点"""
    if len(sys.argv) != 3:
        print("Usage: python gen_code.py <input_file> <output_directory>")
        sys.exit(1)
    input_file = sys.argv[1]
    output_directory = sys.argv[2]
    generate_python_code(input_file, output_directory)


if __name__ == "__main__":
    main()