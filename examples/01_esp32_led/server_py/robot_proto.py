import struct
from fprotocol import DynamicStruct,FProtocolType,BaseValue
class RobotProto:
    config_wifi_t_desc = [("_name_size","H",0),("name","64s",b'\x00' * 64),("_pswd_size","H",0),("pswd","64s",b'\x00' * 64)]

    def __init__(self):
        self.index_table = {}
        self.led = BaseValue(0, "B")
        self.index_table[0x0001] = self.led
        self.config_wifi = DynamicStruct(self.config_wifi_t_desc)
        self.index_table[0x0002] = self.config_wifi

    def get_index_data(self,index):
        return self.index_table[index]

    def __setattr__(self, name, value):
        # 如果属性是 BaseValue 且已存在，则更新其值而不是替换对象
        if hasattr(self, name) and isinstance(getattr(self, name), BaseValue):
            getattr(self, name).value = value
        else:
            object.__setattr__(self, name, value)

    def write_led(self,fprotocol,type,node):
        bytes_data = self.led.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0001,bytes_data,len(bytes_data))

    def write_config_wifi(self,fprotocol,type,node):
        bytes_data = self.config_wifi.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0002,bytes_data,len(bytes_data))

    def read_led(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0001,[],0)

    def read_config_wifi(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0002,[],0)