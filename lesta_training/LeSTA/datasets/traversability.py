import os
import torch
import numpy as np
from torch.utils.data import Dataset, random_split
from LeSTA.datasets.transform import NoWeighting


class TraversabilityDataset(Dataset):
    def __init__(self, path=None, features=None, labels=None, transform=None, download=False):
        """Initialize dataset either from file or from tensors directly"""

        self.dataset_root = path
        self.transform = transform
        self.classes = {"traversable": 1,
                        "non-traversable": 0,
                        "unknown": -1}

        if path is not None:
            # Initialize from file
            if not os.path.exists(self.dataset_root) and not download:
                raise FileNotFoundError(
                    f"Dataset not found at {self.dataset_root}")
            elif download:
                self._download()
            else:
                # Populate tensors
                self.features = torch.tensor([])
                self.labels = torch.tensor([])
                self._load_data()

        elif features is not None:
            # Initialize from tensors
            self.features = features
            self.labels = labels if labels is not None else torch.ones(
                len(features), 1) * -1
        else:
            raise ValueError(
                "Either path or features and labels must be provided")

        if self.transform is None:
            self.transform = NoWeighting()

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        label = self.labels[idx]
        if label == 1:  # only apply transform to footprint samples
            feature, sample_weight = self.transform(self.features[idx])
        else:
            feature, sample_weight = NoWeighting()(self.features[idx])
        return feature, label, sample_weight

    def _load_data(self):
        """Load csv data from self.dataset_root directory"""
        data = np.loadtxt(self.dataset_root, delimiter=',', skiprows=1)
        self.features = torch.tensor(
            data[:, :-1], dtype=torch.float32)  # All columns except last
        self.labels = torch.tensor(
            data[:, -1], dtype=torch.float32).unsqueeze(1)  # label column

    def _download(self):
        raise NotImplementedError(
            "Downloading is not supported for this dataset")

    # for debug purpose
    def to_csv(self, file_path):
        header = 'step,slope,roughness,curvature,variance,intrinsic_risk,cumulative_risk,risk_weight,traversability_label'
        data = np.concatenate([
            self.features.numpy(),
            self.labels.numpy().astype(int)
        ], axis=1)
        np.savetxt(file_path, data, delimiter=',',
                   header=header, comments='', fmt='%g')

    @staticmethod
    def random_split(dataset, training_ratio, seed):

        generator = torch.Generator().manual_seed(seed)
        validation_size = int((1 - training_ratio) * len(dataset))
        training_size = len(dataset) - validation_size

        # random_split returns a Subset object
        training_subset, validation_subset = random_split(
            dataset, [training_size, validation_size], generator=generator)

        # Convert Subset to TraversabilityDataset
        training_set = TraversabilityDataset(
            features=dataset.features[training_subset.indices],
            labels=dataset.labels[training_subset.indices],
            transform=dataset.transform
        )
        validation_set = TraversabilityDataset(
            features=dataset.features[validation_subset.indices],
            labels=dataset.labels[validation_subset.indices],
            transform=dataset.transform
        )

        return training_set, validation_set


if __name__ == "__main__":

    project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    data_path = os.path.join(project_root, "data/labeled_data.csv")

    dataset = TraversabilityDataset(path=data_path,
                                    download=False)
    feature, label, weight = dataset[0]
    print("Feature shape:", feature.shape)
    print("Label shape:", label.shape)
    print("Weight shape:", weight.shape)
    print()

    training_set, validation_set = TraversabilityDataset.random_split(
        dataset, training_ratio=0.8, seed=42)
    print(f"Labeled samples: {len(dataset)}")
    print(f"Training samples: {len(training_set)}")
    print(f"Validation samples: {len(validation_set)}")
