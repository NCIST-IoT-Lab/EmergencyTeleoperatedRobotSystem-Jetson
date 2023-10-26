from open3d.ml.datasets import BaseDataset

class SUNRGBD(BaseDataset):
    def __init__(self,
                dataset_path,
                name="SUNRGBD",
                cache_dir='./logs/cache',
                use_cache=False,
                **kwargs):

        super().__init__(dataset_path=dataset_path,
                        #  info_path=info_path,
                         name=name,
                         cache_dir=cache_dir,
                         use_cache=use_cache,
                         **kwargs)

    

        # read file lists.

    def get_split(self, split):
        return MyDatasetSplit(self, split=split)

    def is_tested(self, attr):
        return 1
        # checks whether attr['name'] is already tested.

    def save_test_result(self, results, attr):
        return 1
        # save results['predict_labels'] to file.


class MyDatasetSplit():
    def __init__(self, dataset, split='train'):
        self.split = split
        self.path_list = []
        # collect list of files relevant to split.

    def __len__(self):
        return len(self.path_list)

    def get_data(self, idx):
        path = self.path_list[idx]
        # points, features, labels = read_pc(path)
        # return {'point': points, 'feat': features, 'label': labels}

    def get_attr(self, idx):
        path = self.path_list[idx]
        name = path.split('/')[-1]
        return {'name': name, 'path': path, 'split': self.split}
