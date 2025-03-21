from .base import PCDDatasetBase
import numpy as np
import torch


class PCDDataset(PCDDatasetBase):
    def __init__(self, file_path: str, cfg: dict):
        super().__init__(file_path, cfg)

        # Member variables
        self.cfg = cfg
        self.file_path = file_path
        self.feature_fields: list = None
        self.feature_vectors: np.ndarray = None  # shape: (N, num_features)
        self.feature_means: np.ndarray = None    # shape: (num_features,)
        self.feature_stds: np.ndarray = None     # shape: (num_features,)
        self.skip_normalization = cfg.get('skip_dataset_normalization', False)

        self._parse_features()

    def __len__(self):
        return len(self.feature_vectors)

    def __getitem__(self, idx):
        """Get item at index. Returns pytorch tensors"""

        features, label = self.feature_vectors[idx], self.labels[idx]
        if not self.skip_normalization:
            features = self._normalize_features(features)

        # Convert to pytorch tensors
        features = torch.from_numpy(features).float()
        label = torch.tensor(float(label), dtype=torch.float32)
        return {"feats": features, "label": label}

    def _parse_features(self):
        """Extract feature vectors based on config."""

        # Extract feature fields
        if "feature_fields" in self.cfg:
            self.feature_fields = self.cfg["feature_fields"]
        else:
            self.feature_fields = self.point_fields

        # Extract feature vectors
        feat_field_indices = []
        for feat_field in self.feature_fields:
            if feat_field not in self.point_fields:
                raise ValueError(
                    f"Feature '{feat_field}' not found in PCD fields")
            feat_field_indices.append(self.point_fields.index(feat_field))

        self.feature_vectors = self.points[:, feat_field_indices]

        # Calculate feature means and stds
        self._calc_normalization_constants()

    def _normalize_features(self, features: np.ndarray):
        """z-score normalization of feature vector"""
        if self.feature_means is None or self.feature_stds is None:
            self._calc_normalization_constants()
        return (features - self.feature_means) / self.feature_stds

    def _calc_normalization_constants(self):
        """Calculate feature means and stds from dataset."""
        self.feature_means = self.feature_vectors.mean(axis=0)
        self.feature_stds = self.feature_vectors.std(axis=0)
        # Prevent division by zero by setting zero std to a small value
        self.feature_stds = np.where(
            self.feature_stds == 0, 1e-8, self.feature_stds)

    def get_normalization_constants(self):
        """Get feature means and stds"""
        if self.feature_means is None or self.feature_stds is None:
            self._calc_normalization_constants()
        return self.feature_means, self.feature_stds

    @classmethod
    def from_subset(cls, features, labels, file_path, cfg):
        """Create a new dataset from a subset of the current dataset."""
        instance = cls(file_path, cfg)
        instance.feature_vectors = features
        instance.labels = labels
        return instance

    def get_labeled_set(self):
        """Get the dataset containing only labeled points (labels 0 or 1)."""
        labeled_indices = self._get_labeled_indices()
        labeled_subset = self.from_subset(self.feature_vectors[labeled_indices],
                                          self.labels[labeled_indices],
                                          self.file_path,
                                          self.cfg)
        return labeled_subset

    def get_unlabeled_set(self):
        """Get the dataset containing only unlabeled points (labels -1)."""
        unlabeled_indices = self._get_unlabeled_indices()
        unlabeled_subset = self.from_subset(self.feature_vectors[unlabeled_indices],
                                            self.labels[unlabeled_indices],
                                            self.file_path,
                                            self.cfg)
        return unlabeled_subset

    def _get_labeled_indices(self):
        """Get indices of labeled points (labels 0 or 1)."""
        return np.where((self.labels == 0) | (self.labels == 1))[0]

    def _get_unlabeled_indices(self):
        """Get indices of unlabeled points (labels -1)."""
        return np.where(self.labels == -1)[0]

    def add_data(self, features, labels):
        """Add data to the dataset."""
        self.labels = np.concatenate([self.labels, labels])
        self.feature_vectors = np.concatenate([self.feature_vectors, features])

    def remove_data(self, indices):
        """Remove data from the dataset."""
        self.feature_vectors = np.delete(self.feature_vectors, indices, axis=0)
        self.labels = np.delete(self.labels, indices)
