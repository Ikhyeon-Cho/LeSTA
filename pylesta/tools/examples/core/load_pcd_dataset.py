from lesta.core.datasets.pcd_dataset import PCDDataset, RiskWeightedPCDDataset
from torch.utils.data import DataLoader
from utils.param import yaml
import numpy as np

if __name__ == "__main__":

    cfg = yaml.load("lesta_training/configs/lesta.yaml")
    DATASET_CFG = cfg['DATASET']
    file_path = DATASET_CFG['training_data']

    dataset = PCDDataset(file_path, DATASET_CFG)
    data_loader = DataLoader(
        dataset,
        batch_size=512,
        shuffle=False
    )
    for batch in data_loader:
        print(batch["feats"].shape)
        print(batch["label"].shape)
        print(np.unique(batch["label"]))
        # find min max of batch["feats"] at column 0
        print(batch["feats"][:, 0].min(), batch["feats"][:, 1].max())
        print()
        break

    labeled_set = dataset.get_labeled_set()
    unlabeled_set = dataset.get_unlabeled_set()

    labeled_data_loader = DataLoader(
        labeled_set,
        batch_size=512,
        shuffle=False
    )

    for batch in labeled_data_loader:
        print(batch["feats"].shape)
        print(batch["label"].shape)
        print(np.unique(batch["label"]))
        # find min max of batch["feats"] at column 0
        print(batch["feats"][:, 0].min(), batch["feats"][:, 1].max())
        print()
        break

    unlabeled_data_loader = DataLoader(
        unlabeled_set,
        batch_size=512,
        shuffle=False
    )

    for batch in unlabeled_data_loader:
        print(batch["feats"].shape)
        print(batch["label"].shape)
        print(np.unique(batch["label"]))
        # find min max of batch["feats"] at column 0
        print(batch["feats"][:, 0].min(), batch["feats"][:, 1].max())
        print()
        break
