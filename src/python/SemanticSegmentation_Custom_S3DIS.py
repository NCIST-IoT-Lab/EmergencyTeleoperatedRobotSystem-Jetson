# -*- coding: UTF-8 -*-

import os
import open3d as o3d
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d
import numpy as np

# 配置文件路径
cfg_file = "/home/ncistwlwsys/hezhizhou-projects/Open3D-ML/ml3d/configs/randlanet_s3dis.yml"
# 加载配置文件
cfg = _ml3d.utils.Config.load_from_file(cfg_file)

# 创建 RandLANet 模型，RandLANet 是一个基于点云的语义分割模型
model = ml3d.models.RandLANet(**cfg.model)

# 创建 pipeline
pipeline = ml3d.pipelines.SemanticSegmentation(model, device="cuda", **cfg.pipeline)

# 下载预训练模型权重
ckpt_folder = "./logs/"
os.makedirs(ckpt_folder, exist_ok=True)
ckpt_path = ckpt_folder + "randlanet_s3dis_202201071330utc.pth"
randlanet_url = "https://storage.googleapis.com/open3d-releases/model-zoo/randlanet_s3dis_202201071330utc.pth"
if not os.path.exists(ckpt_path):
    cmd = "wget {} -O {}".format(randlanet_url, ckpt_path)
    os.system(cmd)

# 加载预训练模型权重
pipeline.load_ckpt(ckpt_path=ckpt_path)

s3dis_labels = ml3d.datasets.S3DIS.get_label_to_names()

# 读取点云文件
print("Reading pcd")
load_pcd = o3d.io.read_point_cloud(
    filename="/home/ncistwlwsys/hezhizhou-projects/ETRS/bin/ply/final_cloud.ply23-10-26_21-10-37.ply", print_progress=True)

load_data = {
    'point': np.array(load_pcd.points, dtype=np.float32),
    'feat': np.array(load_pcd.colors, dtype=np.float32), 
    'label': np.zeros((len(load_pcd.points),), dtype=np.int32),
}

# 使用一个样本运行推理
# 返回字典，包含 'predict_labels' 和 'predict_scores' 两个字段
result = pipeline.run_inference(load_data)

# 可视化推理结果
# s3dis_labels = ml3d.datasets.S3DIS.get_label_to_names()
vis = ml3d.vis.Visualizer()
lut = ml3d.vis.LabelLUT()
for val in sorted(s3dis_labels.keys()):
    lut.add_label(s3dis_labels[val], val)
vis.set_lut("labels", lut)
vis.set_lut("pred", lut)

pred_label = (result['predict_labels'] + 1).astype(np.int32)
# 因为预测值没有0值，所以填充“unlabeled”值。
pred_label[0] = 0

vis_d = [{
    'name': 's3dis_point_cloud',
    'points': load_data["point"],
    'pred': pred_label
}]
# print(data["label"]);
# [12 12 12 ...  2  2  2]
# print(s3dis_labels);
# {0: 'ceiling', 1: 'floor', 2: 'wall', 3: 'beam', 4: 'column', 5: 'window', 6: 'door', 7: 'table', 8: 'chair', 9: 'sofa', 10: 'bookcase', 11: 'board', 12: 'clutter'}


vis.visualize(vis_d)


# print(result);

# evaluate performance on the test set; this will write logs to './logs'.
# pipeline.run_test()

# print(123);
