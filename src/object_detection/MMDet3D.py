from mmdet3d.apis import init_model, inference_detector

# 命令行下载预训练模型
# mim download mmdet3d --config votenet_8xb16_sunrgbd-3d.py --dest checkpoints
# 保存在 ./checkpoints 当中

config_file = 'src/python/configs/votenet_8xb16_sunrgbd-3d.py'  # 配置文件
ckpt_file = 'src/python/checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'   # 模型文件

# 初始化模型
model = init_model(config_file, ckpt_file, device='cuda:0') 

# SUNRGBD数据集的测试文件
# pcd = '/home/ncistwlwsys/hezhizhou-projects/disk/SUNRGBD/...'

# 1005点云测试文件
pcd = 'test_point_clouds/lab_point_cloud2.ply'

result, data = inference_detector(model, pcd)

# 保存文件
# np.save('test_point_clouds/lab_point_cloud2.npy', data['pts_bbox'])
