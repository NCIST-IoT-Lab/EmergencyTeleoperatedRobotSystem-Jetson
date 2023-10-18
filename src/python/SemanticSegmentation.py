# -*- coding: UTF-8 -*-

import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d

# 配置文件路径
cfg_file = "/home/ncistwlwsys/hezhizhou-projects/Open3D-ML/ml3d/configs/randlanet_semantickitti.yml"
# 加载配置文件
cfg = _ml3d.utils.Config.load_from_file(cfg_file)

# 创建 RandLANet 模型，RandLANet 是一个基于点云的语义分割模型
model = ml3d.models.RandLANet(**cfg.model)
# 设置数据集路径
cfg.dataset['dataset_path'] = "/home/ncistwlwsys/hezhizhou-projects/disk/SemanticKITTI"
# 创建数据集
dataset = ml3d.datasets.SemanticKITTI(cfg.dataset.pop('dataset_path', None), **cfg.dataset)
# 创建 pipeline
pipeline = ml3d.pipelines.SemanticSegmentation(model, dataset=dataset, device="cuda", **cfg.pipeline) 

# 下载预训练模型权重
ckpt_folder = "./logs/"
os.makedirs(ckpt_folder, exist_ok=True)
ckpt_path = ckpt_folder + "randlanet_semantickitti_202201071330utc.pth"
randlanet_url = "https://storage.googleapis.com/open3d-releases/model-zoo/randlanet_semantickitti_202201071330utc.pth"
if not os.path.exists(ckpt_path):
    cmd = "wget {} -O {}".format(randlanet_url, ckpt_path)
    os.system(cmd)

# 加载预训练模型权重
pipeline.load_ckpt(ckpt_path=ckpt_path)

# 获取测试集
test_split = dataset.get_split("test")
# 获取测试集中的第一个数据
data = test_split.get_data(0)

# run inference on a single example.
# returns dict with 'predict_labels' and 'predict_scores'.
result = pipeline.run_inference(data)

# evaluate performance on the test set; this will write logs to './logs'.
pipeline.run_test()