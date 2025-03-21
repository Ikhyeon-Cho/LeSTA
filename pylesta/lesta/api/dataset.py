"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/dataset.py
"""
from ..core.datasets.pcd_dataset import RiskWeightedPCDDataset


class TraversabilityDataset:
    """
    A wrapper dataset class with a unified interface for the other dataset classes.
    """
    # Add your custom dataset class here
    dataset_list = {
        'pcd_dataset': RiskWeightedPCDDataset,
    }

    def __init__(self, cfg: dict, train=True):
        dset_type = cfg['type'].lower()
        if train:
            file_path = cfg['training_data']
        else:
            file_path = cfg['validation_data']
        self.dataset = self._build_dataset(dset_type, file_path, cfg)

    def _build_dataset(self, dset_type: str, file_path: str, cfg: dict):
        if dset_type not in self.dataset_list:
            raise ValueError(f"Dataset type {dset_type} not implemented")
        return self.dataset_list[dset_type](file_path, cfg=cfg)

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):
        return self.dataset[idx]

    def __getattr__(self, name):
        return getattr(self.dataset, name)

    def get_labeled_set(self):
        return self.dataset.get_labeled_set()

    def get_unlabeled_set(self):
        return self.dataset.get_unlabeled_set()
