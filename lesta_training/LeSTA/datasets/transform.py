import torch
import os.path as osp
import yaml


class NoWeighting:
    def __init__(self):
        pass

    def __call__(self, feature_vector):
        # Handle both single sample and batch cases
        if feature_vector.dim() == 1:
            sample_weight = torch.ones(1)
        else:
            sample_weight = torch.ones(feature_vector.size(0))
        return feature_vector, sample_weight


class RiskBasedWeighting:
    """Risk-based weighting scheme"""

    def __init__(self,
                 dataset,
                 # step, slope, roughness, curvature, variance
                 importance_weight=[1.0, 1.0, 1.0, 1.0, 1.0]):
        self.dataset = dataset
        self.importance_weight = torch.tensor(importance_weight).unsqueeze(0)

        # Construct empirical CDF during initialization
        dataset_risks = self._calc_intrinsic_risk(self.dataset.features)
        self.sorted_risks = torch.sort(dataset_risks)[0]

    def __call__(self, feature_vector):
        intrinsic_weight = self._calc_intrinsic_risk(feature_vector)
        cumulative_weight = self._calc_cumulative_risk(intrinsic_weight)

        return feature_vector, (1 + intrinsic_weight) * (1 + cumulative_weight)

    def _calc_intrinsic_risk(self, feature_vector):
        # normalize each feature vector to [0, 1]
        features_min = self.dataset.features.min(dim=0).values
        features_max = self.dataset.features.max(dim=0).values
        features_normalized = (feature_vector - features_min) / \
            (features_max - features_min)

        return torch.sum(self.importance_weight * features_normalized, dim=1)

    def _calc_cumulative_risk(self, intrinsic_risks):
        """Calculate cumulative risk probability using pre-computed empirical CDF.

        Args:
            intrinsic_risks: Tensor of intrinsic risk values to evaluate

        Returns:
            Tensor of cumulative probabilities for the input risks
        """
        cumulative_probs = torch.zeros_like(intrinsic_risks)
        for i, risk in enumerate(intrinsic_risks):
            cumulative_probs[i] = torch.sum(
                self.sorted_risks <= risk) / len(self.sorted_risks)

        return cumulative_probs


if __name__ == "__main__":

    transform = NoWeighting()
    feature, sample_weight = transform(torch.randn(1, 5))
    print("Sample (feature) shape:", feature.shape)
    print("Sample weight shape:", sample_weight.shape)
