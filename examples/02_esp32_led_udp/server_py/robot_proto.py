import struct
from fprotocol import DynamicStruct,FProtocolType
class RobotProto:

    def __init__(self):
        self.index_table = {}
        self.led = 0
        self.index_table[0x0001] = self.led

    def get_index_data(self,index):
        return self.index_table[index]

    def write_led(self,fprotocol,type,node):
        bytes_data = struct.pack("B",self.led)
        fprotocol.fprotocol_write(node,type,0x0001,bytes_data,len(bytes_data))

    def read_led(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0001,[],0)