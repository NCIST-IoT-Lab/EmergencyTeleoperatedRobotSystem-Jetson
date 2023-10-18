# 接收protobuf测试
import socket
import sys
sys.path.append("/home/ncistwlwsys/hezhizhou-projects/SingleAzureKinect3DReconstruction/src/python/protobuf")
import protobuf.DataMessage_pb2 as datamsg

# 连接 localhost 的 924 端口
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', 924))
print("recving...")
recv_buff = s.recv(1024)
data_message = datamsg.DataMessage()
data_message.ParseFromString(recv_buff)
print(data_message.data)




