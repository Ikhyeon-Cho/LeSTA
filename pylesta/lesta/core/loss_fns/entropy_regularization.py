import torch
import torch.nn as nn
import torch.nn.functional as F


class EntropyRegularization(nn.Module):
    """
    Entropy regularization module that can be added to other loss functions.

    Lower entropy means more confident predictions. Depending on the coefficient sign:
    - Positive coefficient: penalizes confident predictions (encourages exploration)
    - Negative coefficient: rewards confident predictions (encourages exploitation)
    """

    def __init__(self, coefficient=0.1):
        super().__init__()
        self.coefficient = coefficient

    def forward(self, logits):
        """
        Calculate the entropy of the predicted probabilities.
        """
        probs = torch.sigmoid(logits)
        probs = torch.clamp(probs, 1e-7, 1.0 - 1e-7)

        # Calculate binary entropy: -p*log(p) - (1-p)*log(1-p)
        entropy = -probs * torch.log(probs) - \
            (1 - probs) * torch.log(1 - probs)

        return self.coefficient * entropy.mean()
