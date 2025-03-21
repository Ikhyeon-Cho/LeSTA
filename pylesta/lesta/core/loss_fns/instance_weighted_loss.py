from torch.nn import Module, BCEWithLogitsLoss
from .entropy_regularization import EntropyRegularization


class InstanceWeightedLoss(Module):
    def __init__(self, reduction='mean', pos_weight=None):
        super().__init__()
        self.reduction = reduction
        self.criterion = BCEWithLogitsLoss(
            reduction='none', pos_weight=pos_weight)

    def set_pos_weight(self, pos_weight):
        self.criterion.pos_weight = pos_weight

    def forward(self, logits, targets, instance_weights=None):
        if logits.dim() > 1:
            logits = logits.squeeze(-1)

        loss = self.criterion(logits, targets.float())

        if instance_weights is None:
            weighted_loss = loss
        else:
            # Ensure instance_weights can broadcast correctly
            if instance_weights.dim() == 1 and loss.dim() > 1:
                instance_weights = instance_weights.view(
                    -1, *([1] * (loss.dim() - 1)))

            weighted_loss = loss * instance_weights

        if self.reduction == 'mean':
            return weighted_loss.mean()
        elif self.reduction == 'sum':
            return weighted_loss.sum()
        else:
            return weighted_loss


class InstanceWeightedBCEWithLogitLossAndEntropy(Module):
    """
    Combines instance-weighted BCE loss with entropy regularization.
    """

    def __init__(self, entropy_coefficient=0.1, reduction='mean', pos_weight=None):
        super().__init__()
        self.reduction = reduction
        self.criterion = BCEWithLogitsLoss(
            reduction='none', pos_weight=pos_weight)
        self.entropy_reg = EntropyRegularization(
            coefficient=entropy_coefficient)

    def set_pos_weight(self, pos_weight):
        self.criterion.pos_weight = pos_weight

    def forward(self, logits, targets, instance_weights=None):
        if logits.dim() > 1:
            logits = logits.squeeze(-1)

        loss = self.criterion(logits, targets.float())

        if instance_weights is None:
            weighted_loss = loss
        else:
            # Ensure instance_weights can broadcast correctly
            if instance_weights.dim() == 1 and loss.dim() > 1:
                instance_weights = instance_weights.view(
                    -1, *([1] * (loss.dim() - 1)))

            weighted_loss = loss * instance_weights

        if self.reduction == 'mean':
            bce_loss = weighted_loss.mean()
        elif self.reduction == 'sum':
            bce_loss = weighted_loss.sum()
        else:
            bce_loss = weighted_loss

        # Calculate entropy regularization
        entropy_term = self.entropy_reg(logits)

        return bce_loss + entropy_term
