import torch
import torch.nn as nn


class RiskAwareCELoss(nn.Module):
    def __init__(self, weight=None):
        super(RiskAwareCELoss, self).__init__()
        self.cross_entropy = nn.CrossEntropyLoss()

    def forward(self, inputs, targets, risk_weights):
        loss = self.cross_entropy(inputs, targets)
        weighted_loss = risk_weights * loss
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
