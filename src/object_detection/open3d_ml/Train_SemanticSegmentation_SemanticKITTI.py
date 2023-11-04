import open3d.ml as _ml3d
import open3d.ml.torch as ml3d
from open3d._ml3d.torch import RandLANet, SemanticSegmentation, PointPillars, ObjectDetection


path = '/home/ncistwlwsys/hezhizhou-projects/disk/SemanticKITTI'

dataset = ml3d.datasets.SemanticKITTI(dataset_path=path, use_cache=True)

# create the model with random initialization.
model = RandLANet()

pipeline = SemanticSegmentation(model=model, dataset=dataset, max_epoch=100)

# prints training progress in the console.
pipeline.run_train()
