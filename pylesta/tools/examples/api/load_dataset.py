from lesta.api.dataset import TraversabilityDataset
from torch.utils.data import DataLoader
from utils.param import yaml
import numpy as np


def load_training_set(cfg):
    """
    The way to load the training set.
    Training set includes:
    - labeled set (Label: [0, 1])
    - unlabeled set (Label: [-1])
    """
    training_set = TraversabilityDataset(cfg=cfg, train=True)

    example_data_loader = DataLoader(
        training_set,
        batch_size=len(training_set),
        shuffle=False
    )
    print("len(training_set):", len(training_set))
    for batch in example_data_loader:
        print("np.unique(batch['label']):", np.unique(batch["label"]))
        print()
        break


def load_labeled_set(cfg):
    """
    The way to load the labeled set.
    Used in training loop.
    """
    training_set = TraversabilityDataset(cfg=cfg, train=True)
    labeled_set = training_set.get_labeled_set()

    example_data_loader = DataLoader(
        labeled_set,
        batch_size=len(labeled_set),
        shuffle=False
    )
    print("len(labeled_set):", len(labeled_set))
    for batch in example_data_loader:
        print("np.unique(batch['label']):", np.unique(batch["label"]))
        print()
        break


def load_unlabeled_set(cfg):
    """
    The way to load the unlabeled set.
    Used in pseudo-labeling loop.
    """
    training_set = TraversabilityDataset(cfg=cfg, train=True)
    unlabeled_set = training_set.get_unlabeled_set()

    example_data_loader = DataLoader(
        unlabeled_set,
        batch_size=len(unlabeled_set),
        shuffle=False
    )
    print("len(unlabeled_set):", len(unlabeled_set))
    for batch in example_data_loader:
        print("np.unique(batch['label']):", np.unique(batch["label"]))
        print()
        break


if __name__ == "__main__":

    cfg = yaml.load("lesta_training/configs/lesta.yaml")
    DATASET_CFG = cfg['DATASET']

    load_training_set(cfg=DATASET_CFG)

    load_labeled_set(cfg=DATASET_CFG)

    load_unlabeled_set(cfg=DATASET_CFG)
