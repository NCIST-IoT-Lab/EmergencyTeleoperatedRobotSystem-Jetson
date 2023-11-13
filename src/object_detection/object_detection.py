""" ETRS
    目标检测
    Created by HoChihchou on 2023/11/12
"""

from inference import DetModel, DetectionResultType, PointCloudType
import numpy as np


def detect_objects(point_cloud: PointCloudType) -> DetectionResultType:
    config_file = "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection/configs/votenet_8xb16_sunrgbd-3d.py"  # 配置文件
    ckpt_file = "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection/checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth"  # 模型文件

    det_model = DetModel(config_file, ckpt_file)

    point_cloud = np.array(point_cloud)
    result = det_model.inference(point_cloud)

    return result
