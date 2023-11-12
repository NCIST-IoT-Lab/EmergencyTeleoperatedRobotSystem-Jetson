''' ETRS
    网络通信
'''
import socket
import sys
# import protobuf.DataMessage_pb2 
from protobuf.DataMessage_pb2 import DataMessage
import struct
from google.protobuf.message import Message
sys.path.append("/home/ncistwlwsys/hezhizhou-projects/disks/SingleAzureKinect3DReconstruction/src/object_detection/protobuf")

class BaseCommunicator:
    ip_address = ""
    port = 0

    def __init__(self):
        pass
        # self.ip_address = ip_address
        # self.port = port
    
    def connect(self, ip_address: str, port: int):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(ip_address, port)


    def __get_data_length(self) -> int:
        msg_len_data = self.recv(4)
        msg_len = struct.unpack("i", msg_len_data)
        print("message length: ", msg_len)
        return msg_len


    def __recv_data(self) -> bytes:
        length = self.__set_data_length()
        meg_bytes = b""
        while length > 0:
            data = self.s.recv(length)
            meg_bytes += data
            length -= len(data)
        return meg_bytes


    def __set_data_length(self, data: bytes) -> bytes:
        return struct.pack("i", len(data))
        
        
    def __send_data(self, data: bytes) -> None:
        new_data = b""
        new_data += self.__get_data_length(data)
        new_data += data
        self.s.send(new_data)

        
    def send_message(self, message: Message) -> None:
        msg_str = message.SerializeToString()
        if msg_str is None:
            print("message 为空")
            return
        self.__send_data(msg_str)


    def recv_message(self) -> Message:
        msg_bytes = self.__recv_data()
        data_msg = DataMessage()
        data_msg.ParseFromString(msg_bytes)
        return data_msg


class CppCommunicator(BaseCommunicator):
    ip_address = "localhost"
    port = 0


    def __init__(self):
        super().__init__()

        
        
         







