import torch
from torch.utils.data import Dataset, random_split

import pandas as pd


class TraversabilityDataset(Dataset):
    def __init__(self, csv_file, transform=None):
        self.dataframe = pd.read_csv(csv_file)
        self.features = torch.tensor(self.dataframe.drop(
            "traversability_label", axis=1).values, dtype=torch.float32)
        self.labels = torch.tensor(
            self.dataframe["traversability_label"].values, dtype=torch.float32).unsqueeze(1)
        self.transform = transform

    def __len__(self):
        return len(self.dataframe)

    def __getitem__(self, idx):
        feature = self.features[idx]
        label = self.labels[idx]

        if self.transform:
            feature = self.transform(feature)

        return feature, label

    def random_split(self, test_ratio, manual_seed):
        generator = torch.Generator().manual_seed(manual_seed)
        test_size = int(test_ratio * len(self))
        train_size = len(self) - test_size
        return random_split(self, [train_size, test_size], generator=generator)