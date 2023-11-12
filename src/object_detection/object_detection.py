''' ETRS
    目标检测
'''
# from threading import Thread
# from network import CppCommunicator, BaseCommunicator
# from google.protobuf.message import Message
# from protobuf.DataMessage_pb2 import DataMessage
# from queue import Queue
# import point_cloud as pc
from inference import DetModel, DetectionResultType, PointCloudType
import numpy as np




# class RecvTread(Thread):
#     def __init__(self, com: BaseCommunicator, queue: Queue):
#         super().__init__(self)
#         self.com = com
#         self.queue = queue
        

#     def run(self):
#         print("开始接收数据")       
#         # 接收点云
#         while True:
#             self.queue.put(self.__recv_message())
        

#     def __recv_message(self) -> Any:
#         msg = self.com.recv_message()
#         return self.__analyze_message(msg)


#     def __analyze_message(self, msg: DataMessage) -> Any:
#         if msg.type == DataMessage.Type.POINT_CLOUD:
#             point_cloud = msg.point_cloud
#             points = point_cloud.points
#             return pc.protobuf_to_ndarray(points)
#         else:
#             print("未知消息类型")
#         return


# def main():
    # 连接服务器
    # cppcom = CppCommunicator()
    # cppcom.connect(924)
    # recv_queue = Queue()

    # recv_t = RecvTread(cppcom, recv_queue)
    # recv_t.start()

def detect_objects(point_cloud: PointCloudType) -> DetectionResultType:
    config_file = '/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection/configs/votenet_8xb16_sunrgbd-3d.py'  # 配置文件
    ckpt_file = '/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection/checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'   # 模型文件

    det_model = DetModel(config_file, ckpt_file)

    point_cloud = np.array(point_cloud)
    result = det_model.inference(point_cloud)
    return result