import torch
import torch.nn as nn


class RiskSensitiveCELoss(nn.Module):
    def __init__(self, dataset):
        super(RiskSensitiveCELoss, self).__init__()
        self.bce_loss = nn.BCELoss()
        self.feature_weights = torch.tensor(
            [1.0, 1.0, 1.0, 1.0, 1.0])  # user-defined feature weights

        self.__calc_intrinsic_risk(dataset)
        self.__calc_cumulative_risk()
        self.__calc_risk_weights()

    def __calc_intrinsic_risk(self, dataset):

        # normalize each feature vector to [0, 1]
        features_min = dataset.features.min(dim=0).values
        features_max = dataset.features.max(dim=0).values
        dataset.features = (dataset.features - features_min) / \
            (features_max - features_min)

        self.intrinsic_risks = torch.sum(
            dataset.features * self.feature_weights, dim=1).unsqueeze(1)

    def __calc_cumulative_risk(self):

        # Sort intrinsic risk
        sample_risks_sorted, indices = torch.sort(self.intrinsic_risks, dim=0)
        indices = indices.squeeze()

        # Rank the data and normalize
        ranks = torch.arange(
            1, len(sample_risks_sorted) + 1).float().unsqueeze(1)
        cdf = ranks / len(sample_risks_sorted)

        self.cumulative_risks = torch.empty_like(cdf)
        self.cumulative_risks[indices] = cdf

    def __calc_risk_weights(self):
        risk_weights = 
        pass

    def forward(self, preds, targets):

        loss = self.bce_loss(preds, targets)
        weighted_loss = self.intrinsic_risk * loss

        return weighted_loss.mean()


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
