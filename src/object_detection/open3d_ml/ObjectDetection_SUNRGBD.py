# -*- coding: UTF-8 -*-

import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d

# 配置文件路径
cfg_file = (
    "/home/ncistwlwsys/hezhizhou-projects/Open3D-ML/ml3d/configs/pointpillars_kitti.yml"
)

# 加载配置文件
cfg = _ml3d.utils.Config.load_from_file(cfg_file)

# 创建 PointPillars 模型，PointPillars 是一个基于点云的目标检测模型
model = ml3d.models.PointPillars(**cfg.model)


# 设置数据集路径
cfg.dataset["dataset_path"] = "/home/ncistwlwsys/hezhizhou-projects/disk/SunRGBD"
# 创建数据集
dataset = ml3d.datasets.SunRGBD(cfg.dataset.pop("dataset_path", None), **cfg.dataset)
# 创建 pipeline
pipeline = ml3d.pipelines.ObjectDetection(
    model, dataset=dataset, device="cuda", **cfg.pipeline
)

# 下载预训练模型权重
ckpt_folder = "./logs/"
os.makedirs(ckpt_folder, exist_ok=True)
ckpt_path = ckpt_folder + "pointpillars_kitti_202012221652utc.pth"
pointpillar_url = "https://storage.googleapis.com/open3d-releases/model-zoo/pointpillars_kitti_202012221652utc.pth"
if not os.path.exists(ckpt_path):
    cmd = "wget {} -O {}".format(pointpillar_url, ckpt_path)
    os.system(cmd)

# 加载预训练模型权重
pipeline.load_ckpt(ckpt_path=ckpt_path)

# 获取测试集
test_split = dataset.get_split("test")
# 获取测试集中的第一个数据
data = test_split.get_data(0)

# 保存data的点云数据为ply文件
# pcd = o3d.t.geometry.PointCloud()
# pcd.point.positions = data['point']
# o3d.t.io.write_point_cloud(
# "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/python/data_kitti.ply", pcd)

# 读取点云文件
print("Reading pcd")
# load_pcd = o3d.io.read_point_cloud(
# filename="/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/python/lab_point_cloud.ply", print_progress=True)

# print(np.array(load_pcd.points, dtype=np.float32))

# load_data = {
# 'point': np.array(load_pcd.points, dtype=np.float32),
# 'full_point': np.array(load_pcd.points, dtype=np.float32),
# 'feat': None,
# 'label': None,
# 'bounding_boxes': None,
# }


# 使用一个样本运行推理
# 返回字典，包含 'predict_labels' 和 'predict_scores' 两个字段
result = pipeline.run_inference(data)

print(result)
# 可视化推理结果
s3dis_labels = ml3d.datasets.KITTI.get_label_to_names()
vis = ml3d.vis.Visualizer()
lut = ml3d.vis.LabelLUT()
for val in sorted(s3dis_labels.keys()):
    lut.add_label(s3dis_labels[val], val)
vis.set_lut("labels", lut)
vis.set_lut("pred", lut)

vis_d = [
    {
        "name": "kitti_point_cloud",
        "points": data["point"],
        "bounding_boxes": result[0],
    }
]

vis.visualize(vis_d)


# evaluate performance on the test set; this will write logs to './logs'.
# pipeline.run_test()
