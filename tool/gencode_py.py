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
        if stype == 'uint8_t':
            return f'{len}s', len
        else:
            return f'{type2struct[stype]*len}',type2len[stype] * len
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
        if stype == 'uint8_t':
            return """b'\\x00' * """ + str(len)
        else:
            return [type2defaultv[stype]] * len
            # return (type2defaultv[stype],) * len
    else:
        return type2defaultv[stype]


struct_maps = {}
for row in struct_list:
    struct_name,ttype,name,len = row
    if struct_name not in struct_maps.keys():
        struct_maps[struct_name] = []
    struct_maps[struct_name].append((ttype,name,len))

def struct2class(struct_name, struct):
    # ==============计算 len 和 str================
    struct_len = 0
    struct_str = ''
    for ttype, name, len in struct:
        # print(ttype, name, len)
        type_str, type_len = get_type_str_len(ttype, int(len) if len else None)
        struct_len += type_len
        struct_str += type_str

    content = f"class {struct_name}:\n"
    content += f"   csize = {struct_len}\n"
    content += f"   cstruct = '{struct_str}'\n"
    content += f"   def __init__(self):\n"
    for ttype, name, len in struct:
        default_value = get_default_values(ttype, int(len) if len else None)
        content += f"       self.{name} = {default_value}\n"
    content += f"       self.callback = None\n\n"
    # ==================parse====================
    content += f"   def parse(self, data):\n"
    content += f"       if len(data) != self.csize:\n"
    content += f'           raise ValueError(f"Data length must be self.csize bytes, got {{len(data)}} bytes.")\n'
    content += f'       unpacked = struct.unpack(self.cstruct, data)\n'
    index = 0
    for ttype, name, len in struct:
        if not len:
            content += f"       self.{name} = unpacked[{index}]\n"
            index += 1
        else:
            if ttype == 'uint8_t':
                content += f"       self.{name} = unpacked[{index}]\n"
                index += 1
            else:
                content += f"       self.{name} = list(unpacked[{index}:{index + int(len)}])\n"
                index += int(len)
    # ===================to_bytes=================== 
    content += f"\n   def to_bytes(self):\n"

    def get_array_str(name,len):
        astr = ''
        for index in range(len):
            astr += f"self.{name}[{index}],"
        return astr[:-1]
    content += f"       return struct.pack(self.cstruct, " + ", ".join(
        [f"{get_array_str(name,int(len))}" if len and ctype != 'uint8_t' else f"self.{name}" for ctype, name, len in struct]
    ) + ")\n"
    return content

for struct_name,struct in struct_maps.items():
    content += struct2class(struct_name,struct)   

for row in data_list:
    index, data_type, var_name, callback_flag = row


sum_content = f"class {file_name}Proto:\n"

for l in content.splitlines():
    sum_content += f"    {l}\n"
#   content = f"import struct\n\n"

sum_content += f'\n    def __init__(self):'
sum_content += f'\n        self.index_table = {{}}'
for addr, data_type, var_name, callback_flag in data_list:
    sum_content += f'\n        self.{var_name} = self.{data_type}()'
    sum_content += f'\n        self.index_table[{addr}] = self.{var_name}'


sum_content += f'\n\n    def get_index_data(self,index):'
sum_content += f'\n        return self.index_table[index]'

sum_content += f'\n\n    def send_{var_name}(self,fprotocol,type,node):'
sum_content += f'\n        fprotocol.fprotocol_write(node,type,{addr},self.{var_name}.to_bytes(),self.{var_name}.csize)'

sum_content = 'import struct\n' + sum_content


# 保存到文件
print(sum_content)
def save_to_file(filename, sum_content):
    with open(filename, 'w') as f:
        f.write(sum_content)
    print(f"Saved to {filename}")
save_to_file(f"{output_directory}/{file_name.lower()}ptoto.py",sum_content )
