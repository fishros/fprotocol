import struct
from fprotocol import DynamicStruct
class RobotProto:
    float3d_t_desc = [("x","f",0.0),("y","f",0.0),("z","f",0.0)]

    def __init__(self):
        self.index_table = {}
        self.led = 0
        self.index_table[0x0001] = self.led
        self.odom = DynamicStruct(self.float3d_t_desc)
        self.index_table[0x0002] = self.odom

    def get_index_data(self,index):
        return self.index_table[index]

    def write_led(self,fprotocol,type,node):
        bytes_data = struct.pack("B",self.led)
        fprotocol.fprotocol_write(node,type,0x0001,bytes_data,len(bytes_data))

    def write_odom(self,fprotocol,type,node):
        bytes_data = self.odom.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0002,bytes_data,len(bytes_data))