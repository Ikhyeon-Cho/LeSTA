import torch
from torch.utils.data import Dataset, random_split

import pandas as pd


class TraversabilityDataset(Dataset):
    def __init__(self, csv_file=None, features=None, labels=None, transform=None):

        if csv_file:
            df = pd.read_csv(csv_file)
            self.features = torch.tensor(
                df.drop("traversability_label", axis=1).values, dtype=torch.float32)
            self.labels = torch.tensor(
                df["traversability_label"].values, dtype=torch.float32).unsqueeze(1)

        else:
            self.features = features
            self.labels = labels

        self.transform = transform

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        feature = self.features[idx]
        label = self.labels[idx]

        if self.transform:
            feature = self.transform(feature)

        return feature, label

    def random_split(self, test_ratio, seed):
        generator = torch.Generator().manual_seed(seed)
        test_size = int(test_ratio * len(self))
        train_size = len(self) - test_size

        train_subset, test_subset = random_split(
            self, [train_size, test_size], generator=generator)

        train_dataset = TraversabilityDataset(
            features=self.features[train_subset.indices],
            labels=self.labels[train_subset.indices])
        test_dataset = TraversabilityDataset(
            features=self.features[test_subset.indices],
            labels=self.labels[test_subset.indices])

        return train_dataset, test_dataset

    def append(self, new_features, new_labels):
        self.features = torch.cat([self.features, new_features], dim=0)
        self.labels = torch.cat([self.labels, new_labels], dim=0)

    def to_csv(self, file_path):
        df = pd.DataFrame(self.features.numpy(),
                          columns=['step', 'slope', 'roughness', 'curvature', 'variance'])
        df["traversability_label"] = self.labels.numpy().astype(int)
        df.to_csv(file_path, index=False)
