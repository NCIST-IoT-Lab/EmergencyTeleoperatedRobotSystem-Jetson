''' ETRS
    推理点云
'''
from mmdet3d.apis import init_model, inference_detector
from mmdet3d.structures import Det3DDataSample
import numpy as np
from typing import Tuple, List

PointType = np.ndarray
PointCloudType = List[PointType]
BoundingBoxType = Tuple[float, float, float, float, float, float, float]
DetectionObjectType = Tuple[str, BoundingBoxType, float]
DetectionResultType = List[DetectionObjectType]

label_dict={
    0: 'bed',
    1: 'table',
    2: 'sofa',
    3: 'chair',
    4: 'toilet',
    5: 'desk',
    6: 'dresser',
    7: 'night_stand',
    8: 'bookshelf',
    9: 'bathtub',
}

class DetModel:
    def __init__(self, cfg_file: str, ckpt_file: str) -> None:
        self.cfg_file = cfg_file
        self.ckpt_file = ckpt_file
        self.device = 'cuda:0'
        # 初始化模型
        self.model = init_model(self.cfg_file, self.ckpt_file, device=self.device)

    
    def inference(self, point_cloud: PointCloudType) -> DetectionResultType:
        return self.__convert_result(inference_detector(self.model, point_cloud))


    def __convert_result(self, result: Det3DDataSample) -> DetectionResultType:
        result = result[0].pred_instances_3d
        labels = np.array(result['labels_3d'].cpu())
        bboxes = np.array(result['bboxes_3d'].tensor.cpu(), dtype = float)
        scores = np.array(result['scores_3d'].cpu(), dtype = float)
        return self.__get_best_result(labels, bboxes, scores)


    def __get_best_result(self, labels: np.array, bboxes: np.array, scores: np.array) -> DetectionResultType:
        det_result = []     # 存放最终结果
        best_score = {}     # 存放每个类别最高分的索引
        for i in range(len(scores)):
            if scores[i] > best_score.get(labels[i], 0):
                best_score[labels[i]] = i
        str_labels = list(map(lambda x: label_dict[x], labels))
        for i in best_score.values():
            bbox = (bboxes[i][0], bboxes[i][1], bboxes[i][2], bboxes[i][3], bboxes[i][4], bboxes[i][5], bboxes[i][6])
            det_obj = (str_labels[i], bbox, scores[i])
            det_result.append(det_obj)
        return det_result
        
        
    def __remove_low_score(self):
        pass