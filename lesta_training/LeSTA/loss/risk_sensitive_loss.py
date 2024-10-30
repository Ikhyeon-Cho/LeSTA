import torch
import torch.nn as nn
import torch.nn.functional as F


class WeightedCrossEntropyLoss(nn.Module):
    def __init__(self):
        super().__init__()
        self.bce_with_logits = nn.BCEWithLogitsLoss(reduction='none')

    def forward(self, logits, targets, weights):
        base_loss = self.bce_with_logits(logits, targets)
        weighted_loss = base_loss * weights
        return weighted_loss.mean()


class EntropyRegularization(nn.Module):
    def __init__(self, lambda_entropy=0.1):
        super().__init__()
        self.lambda_entropy = lambda_entropy

    def forward(self, logits, weights):

        # Ensure weights has same shape as logits
        if weights.dim() == 1:
            weights = weights.unsqueeze(1)  # (B,) -> (B, 1)

        # Convert logits to probabilities
        probs = torch.sigmoid(logits)  # (B, 1)

        # Calculate binary entropy: -p*log(p) - (1-p)*log(1-p)
        entropy = -(probs * torch.log(probs + 1e-7) +
                    (1 - probs) * torch.log(1 - probs + 1e-7))

        # Apply sample weights to entropy
        weighted_entropy = weights * entropy  # (B, 1)

        # Negative entropy because we want to maximize entropy
        return -self.lambda_entropy * weighted_entropy.mean()
