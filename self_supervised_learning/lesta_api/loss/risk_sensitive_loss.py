import torch
import torch.nn as nn


class RiskSensitiveCELoss(nn.Module):
    def __init__(self, dataset):
        super(RiskSensitiveCELoss, self).__init__()
        self.feature_weights = torch.tensor(
            # step, slope, roughness, curvature, variance
            [1.0, 1.0, 1.0, 1.0, 1.0])

        self.dataset = dataset

        self.__calc_intrinsic_risk(dataset)

        self.__calc_cumulative_risk()

        self.__calc_risk_weights()

    def __calc_intrinsic_risk(self, dataset):

        # normalize each feature vector to [0, 1]
        features_min = dataset.features.min(dim=0).values
        features_max = dataset.features.max(dim=0).values
        dataset.features = (dataset.features - features_min) / \
            (features_max - features_min)

        # Calculate intrinsic risk only for footprint samples
        footprint_mask = (dataset.labels == 1).squeeze()  # shape: [N]

        self.intrinsic_risks = torch.zeros_like(
            dataset.features[:, 0]).unsqueeze(1)  # shape: [N, 1]

        self.intrinsic_risks[footprint_mask] = torch.sum(
            dataset.features[footprint_mask] * self.feature_weights, dim=1).unsqueeze(1)

        # save for visualization
        # self.dataset.features = torch.cat(
        #     [self.dataset.features, self.intrinsic_risks], dim=1)

    def __calc_cumulative_risk(self):

        # Select intrinsic risks that are non-zero (corresponding to footprint samples)
        footprint_mask = (self.intrinsic_risks > 0).squeeze()  # shape: [N]
        footprint_risks = self.intrinsic_risks[footprint_mask]  # shape: [N, 1]
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

        self.cumulative_risks = torch.zeros_like(self.intrinsic_risks)
        self.cumulative_risks[footprint_mask] = eCDF[idx_from_sorted_to_raw]

        # save for visualization
        # self.dataset.features = torch.cat(
        #     [self.dataset.features, self.cumulative_risks], dim=1)

    def __calc_risk_weights(self):

        # Calculate risk weights
        self.risk_weights = torch.zeros_like(self.intrinsic_risks)
        self.risk_weights = self.intrinsic_risks * self.cumulative_risks

        # normalize risk weights to [0, 1]
        # note that the minimum risk weight should not be zero
        min_risk_weight = torch.min(self.risk_weights[self.risk_weights > 0])
        max_risk_weight = torch.max(self.risk_weights)
        self.risk_weights = (self.risk_weights - min_risk_weight) / \
            (max_risk_weight - min_risk_weight)

        # add 1 to avoid zero risk weight
        self.risk_weights = 1.0 * self.risk_weights + 1.0

        # # Get the minimum risk weight except for zero
        # min_risk_weight = torch.min(self.risk_weights[self.risk_weights > 0])
        # self.risk_weights[self.risk_weights == 0] = min_risk_weight
        # # self.risk_weights = self.risk_weights / min_risk_weight
        # self.risk_weights = self.risk_weights / torch.max(self.risk_weights)

        # # save for visualization
        # self.dataset.features = torch.cat(
        #     [self.dataset.features, self.risk_weights], dim=1)

        # self.dataset.to_csv(
        #     '/home/ikhyeon/Downloads/features_with_risk_weights.csv')
        # pass

    def forward(self, inputs, targets):

        epsilon = 1e-7  # Add small epsilon to avoid log(0)
        loss = - (targets * torch.log(inputs + epsilon) + 
                  (1 - targets) * torch.log1p(1 - inputs + epsilon))
        
        # weighted_loss = self.risk_weights * loss

        # return weighted_loss.mean()
        return loss.mean()
    
    def get_risk_weights(self):
        return self.risk_weights


class EntropyRegularization:
    def __init__(self, lambda_reg=0.1):
        self.lambda_reg = lambda_reg

    def __call__(self, outputs, risk_weights):
        # Check if risk_weights has the same shape as the outputs
        if outputs.shape != risk_weights.shape:
            raise ValueError(
                f"Shape mismatch: outputs shape {outputs.shape} "
                f"and risk_weights shape {
                    risk_weights.shape} must be the same."
            )

        # Compute entropy of the outputs
        entropy = -outputs * torch.log(outputs + 1e-10)
        - (1 - outputs) * torch.log(1 - outputs + 1e-10)
        # Apply per-instance cumulative risk weights
        weighted_entropy = risk_weights * entropy
        # Return the regularization term
        return self.lambda_reg * weighted_entropy.mean()
