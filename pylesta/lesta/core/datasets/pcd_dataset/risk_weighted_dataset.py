from .dataset import PCDDataset
import numpy as np


class RiskWeightedPCDDataset(PCDDataset):
    """
    A dataset class with risk weights for each sample.
    """

    def __init__(self, file_path, cfg):
        super().__init__(file_path, cfg)

        # Member variables
        self.feature_weights = np.array(list(cfg['feature_weights'].values()))
        self.intrinsic_risks = None
        self.cumulative_risks = None
        self.risk_weights = None

        # Calculate intrinsic risks and cumulative risks
        self._calc_intrinsic_risks()
        self._calc_cumulative_risks()
        self._calc_risk_weights()

    def __len__(self):
        return super().__len__()

    def __getitem__(self, idx):
        """Get item at index."""

        sample = super().__getitem__(idx)
        sample['risk_weights'] = self.risk_weights[idx]

        return sample

    def _calc_intrinsic_risks(self):
        """Calculate intrinsic risk of the dataset."""
        # Initialize all intrinsic risks to 1
        self.intrinsic_risks = np.ones_like(self.labels)

        # Only calculate intrinsic risk for samples with label 1 (traversable)
        mask = (self.labels == 1)
        if np.any(mask):
            self.intrinsic_risks[mask] = self._calc_intrinsic_risk(
                self.feature_vectors[mask])

    def _calc_intrinsic_risk(self, input_features):
        """Calculate intrinsic risk of a feature vector."""
        feature_values = self.__normalize_features(input_features)
        intrinsic_risk = np.sum(self.feature_weights * feature_values, axis=1)
        return intrinsic_risk

    def _calc_cumulative_risks(self):
        """Calculate cumulative risk of the dataset."""
        self.cumulative_risks = np.zeros_like(self.intrinsic_risks)

        # Get intrinsic risks only from samples with label 1 (traversable)
        footprint_mask = (self.labels == 1)
        footprint_risks = self.intrinsic_risks[footprint_mask]

        if len(footprint_risks) > 0:
            # Vectorized calculation using percentile rank
            # For each risk value, calculate its percentile in the footprint distribution
            sorted_footprint_risks = np.sort(footprint_risks)
            risk_ranks = np.searchsorted(
                sorted_footprint_risks, self.intrinsic_risks, side='right')
            self.cumulative_risks = risk_ranks / len(footprint_risks)

    def _calc_risk_weights(self):
        """Calculate risk weights of the dataset."""

        # Initialize risk weights to 1 (neutral weight)
        self.risk_weights = np.ones_like(self.intrinsic_risks)

        # Only calculate and apply risk weights for samples with label 1
        footprint_mask = (self.labels == 1)
        if np.any(footprint_mask):
            # Element-wise product of intrinsic and cumulative risks
            raw_weights = self.intrinsic_risks[footprint_mask] * \
                self.cumulative_risks[footprint_mask]

            # Scale the weights to have a more meaningful range
            min_weight = 1.0  # Minimum weight (neutral)
            max_weight = 1.0 + raw_weights.max()  # Maximum weight

            # Min-max scaling to the desired range
            if np.max(raw_weights) > np.min(raw_weights):  # Avoid division by zero
                normalized_weights = (
                    raw_weights - np.min(raw_weights)) / (np.max(raw_weights) - np.min(raw_weights))
                self.risk_weights[footprint_mask] = min_weight + \
                    normalized_weights * (max_weight - min_weight)
            else:
                self.risk_weights[footprint_mask] = min_weight

    def __normalize_features(self, features=None):
        """
        Normalize features to range [0,1] using robust scaling to handle outliers.
        If features is None, normalize and return self.feature_vectors.
        Otherwise, normalize and return the provided features.
        """
        if features is None:
            features = self.feature_vectors
            normalize_in_place = True
        else:
            normalize_in_place = False

        # Get normalization constants from parent class
        means, stds = self.get_normalization_constants()

        # Z-score normalization first (handles outliers better than min-max)
        normalized = (features - means) / stds

        # Apply sigmoid function to map to (0,1) range while handling outliers gracefully
        normalized = 1 / (1 + np.exp(-normalized))

        if normalize_in_place:
            self.feature_vectors = normalized
            return self.feature_vectors
        else:
            return normalized
