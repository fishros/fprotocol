import struct
class ExampleProto:
    class canframe_t:
       csize = 14
       cstruct = 'IBB8s'
       def __init__(self):
           self.arbitration_id = 0
           self.dlc = 0
           self.rtr = 0
           self.data = b'        '
           self.callback = None
    
       def parse(self, data):
           if len(data) != self.csize:
               raise ValueError(f"Data length must be self.csize bytes, got {len(data)} bytes.")
           unpacked = struct.unpack(self.cstruct, data)
           self.arbitration_id = unpacked[0]
           self.dlc = unpacked[1]
           self.rtr = unpacked[2]
           self.data = unpacked[3]
    
       def to_bytes(self):
           return struct.pack(self.cstruct, self.arbitration_id, self.dlc, self.rtr, self.data)

    def __init__(self):
        self.index_table = {}
        self.can_recv_frame = self.canframe_t()
        self.index_table[0x0001] = self.can_recv_frame
        self.can_send_frame = self.canframe_t()
        self.index_table[0x0002] = self.can_send_frame

    def get_index_data(self,index):
        return self.index_table[index]

    def send_can_send_frame(self,fprotocol,type,node):
        fprotocol.fprotocol_write(node,type,0x0002,self.can_send_frame.to_bytes(),self.can_send_frame.csize)