import torch
from torch.utils.data import Dataset, random_split
import pandas as pd


class TraversabilityDataset(Dataset):
    def __init__(self, csv_file=None, features=None, labels=None):

        if csv_file:
            df = pd.read_csv(csv_file)
            self.features = torch.tensor(
                df.drop("traversability_label", axis=1).values, dtype=torch.float32)
            self.labels = torch.tensor(
                df["traversability_label"].values, dtype=torch.float32).unsqueeze(1)

        else:
            self.features = features
            self.labels = labels

        if self.__is_labeled(self.labels):
            self.weights = self.calc_weights()
        else:
            self.weights = None

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):

        if self.weights is not None:
            return self.features[idx], self.labels[idx], self.weights[idx]
        else:
            return self.features[idx], self.labels[idx]

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

    def append(self, new_dataset):
        self.features = torch.cat([self.features, new_dataset.features], dim=0)
        self.labels = torch.cat([self.labels, new_dataset.labels], dim=0)
        self.weights = torch.cat([self.weights, new_dataset.weights], dim=0)

    # for debug purpose
    def to_csv(self, file_path):
        df = pd.DataFrame(self.features.numpy(),
                          columns=['step', 'slope', 'roughness', 'curvature', 'variance', 'intrinsic_risk', 'cumulative_risk', 'risk_weight'])
        df["traversability_label"] = self.labels.numpy().astype(int)
        df.to_csv(file_path, index=False)

    def __is_labeled(self, label):
        if label[0] > -0.5:  # -1 for unlabeled samples
            return True

    def calc_weights(self):
        intrinsic_risks = self.__calc_intrinsic_risk()
        cumulative_risks = self.__calc_cumulative_risk(intrinsic_risks)
        weights = self.__calc_sample_weights(
            intrinsic_risks, cumulative_risks)

        return weights

    def __calc_intrinsic_risk(self):

        feature_weights = torch.tensor(
            # step, slope, roughness, curvature, variance
            [1.0, 1.0, 1.0, 1.0, 1.0])

        # normalize each feature vector to [0, 1]
        features_min = self.features.min(dim=0).values
        features_max = self.features.max(dim=0).values
        features_scaled = (self.features - features_min) / \
            (features_max - features_min)

        # Calculate intrinsic risk only for footprint samples
        footprint_mask = (self.labels == 1).squeeze()  # shape: [N]

        intrinsic_risks = torch.zeros_like(
            self.features[:, 0]).unsqueeze(1)  # shape: [N, 1]

        intrinsic_risks[footprint_mask] = torch.sum(
            feature_weights * self.features[footprint_mask], dim=1).unsqueeze(1)

        return intrinsic_risks

    def __calc_cumulative_risk(self, intrinsic_risks):

        # Select intrinsic risks that are non-zero (corresponding to footprint samples)
        footprint_mask = (intrinsic_risks > 0).squeeze()  # shape: [N]
        footprint_risks = intrinsic_risks[footprint_mask]  # shape: [N, 1]
        n_footprints = len(footprint_risks)

        # Sort intrinsic risk
        _, idx_from_raw_to_sorted = torch.sort(footprint_risks, dim=0)
        idx_from_raw_to_sorted = idx_from_raw_to_sorted.squeeze()

        # Calculate empirical CDF
        ranks = torch.arange(1, n_footprints + 1).float().unsqueeze(1)
        eCDF = ranks / n_footprints

        # Map cumulative risk back to the original order
        idx_from_sorted_to_raw = torch.zeros_like(idx_from_raw_to_sorted)
        idx_from_sorted_to_raw[idx_from_raw_to_sorted] = torch.arange(
            n_footprints)

        cumulative_risks = torch.zeros_like(intrinsic_risks)
        cumulative_risks[footprint_mask] = eCDF[idx_from_sorted_to_raw]

        return cumulative_risks

    def __calc_sample_weights(self, intrinsic_risks, cumulative_risks):

        sample_weights = torch.zeros_like(intrinsic_risks)
        sample_weights = intrinsic_risks * cumulative_risks

        # normalize the weights to [0, 1]
        # note that the minimum risk weight should not be zero
        non_zero_weights = sample_weights[sample_weights > 0]
        if len(non_zero_weights) == 0:
            return torch.ones_like(sample_weights)
        else:
            min_weight = non_zero_weights.min()
            max_weight = non_zero_weights.max()

            sample_weights = (sample_weights - min_weight) / \
                (max_weight - min_weight)

            # add 1 to avoid zero risk weight
            sample_weights = 1.0 * sample_weights + 1.0

            return sample_weights
