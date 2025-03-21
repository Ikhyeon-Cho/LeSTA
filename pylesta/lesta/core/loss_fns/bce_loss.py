import torch.nn as nn
import torch.nn.functional as F


class BCELoss(nn.Module):
    """
    Binary Cross Entropy with Logits Loss with the ability to update positive weights.
    Extends PyTorch's BCEWithLogitsLoss with a method to update the positive weight.
    """

    def __init__(self, pos_weight=None, reduction='mean', weight=None):
        super().__init__()
        self.pos_weight = pos_weight
        self.reduction = reduction
        self.weight = weight

    def forward(self, input, target, instance_weights=None):
        return F.binary_cross_entropy_with_logits(
            input, target,
            weight=self.weight,
            reduction=self.reduction,
            pos_weight=self.pos_weight
        )

    def set_pos_weight(self, pos_weight):
        self.pos_weight = pos_weight
