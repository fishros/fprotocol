import struct
from fprotocol import DynamicStruct,FProtocolType,BaseValue
class RobotProto:
    float3d_t_desc = [("x","f",0.0),("y","f",0.0),("z","f",0.0)]
    nav_opt_t_desc = [("_data_size","H",0),("data","512s",b'\x00' * 512)]

    def __init__(self):
        self.index_table = {}
        self.cmd_vel = DynamicStruct(self.float3d_t_desc)
        self.index_table[0x0001] = self.cmd_vel
        self.nav_target = DynamicStruct(self.nav_opt_t_desc)
        self.index_table[0x0002] = self.nav_target

    def get_index_data(self,index):
        return self.index_table[index]

    def __setattr__(self, name, value):
        # 如果属性是 BaseValue 且已存在，则更新其值而不是替换对象
        if hasattr(self, name) and isinstance(getattr(self, name), BaseValue):
            getattr(self, name).value = value
        else:
            object.__setattr__(self, name, value)

    def write_cmd_vel(self,fprotocol,type,node):
        bytes_data = self.cmd_vel.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0001,bytes_data,len(bytes_data))

    def write_nav_target(self,fprotocol,type,node):
        bytes_data = self.nav_target.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0002,bytes_data,len(bytes_data))

    def read_cmd_vel(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0001,[],0)

    def read_nav_target(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0002,[],0)