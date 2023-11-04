import open3d.ml as _ml3d
import open3d.ml.torch as ml3d
from open3d._ml3d.torch import (
    PointPillars,
    ObjectDetection,
)

cfg_file = "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/python/configs/train_pointpillars_sunrgbd.yml"
cfg = _ml3d.utils.Config.load_from_file(cfg_file)

path = "/home/ncistwlwsys/hezhizhou-projects/disk/S3DIS/Stanford3dDataset_v1.2_Aligned_Version/"

print(f"cfg.model:{cfg.model}")
model = PointPillars(**cfg.model)

cfg.dataset["dataset_path"] = path

dataset = ml3d.datasets.SunRGBD(dataset_path=path, use_cache=True)

pipeline = ObjectDetection(model=model, dataset=dataset, max_epoch=100, **cfg.pipeline)


# prints training progress in the console.
pipeline.run_train()
