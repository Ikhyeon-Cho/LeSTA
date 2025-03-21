"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/loss.py
"""

from ..core.loss_fns import BCELoss
from ..core.loss_fns import InstanceWeightedLoss
from ..core.loss_fns import InstanceWeightedBCEWithLogitLossAndEntropy


class LossFactory:
    """
    A factory class for creating loss functions.
    """
    # Add your custom loss function here
    loss_list = {
        'bce_loss': BCELoss,
        'instance_weighted_loss': InstanceWeightedLoss,
        'risk_sensitive_loss': InstanceWeightedBCEWithLogitLossAndEntropy,
    }

    def __init__(self, cfg: dict, dataset=None):
        self.cfg = cfg
        loss_type = cfg['type'].lower()
        self.loss_fn = self._build_loss_fn(loss_type)

    def _build_loss_fn(self, loss_type: str):
        if loss_type not in self.loss_list:
            raise ValueError(f'Loss type {loss_type} not implemented')

        # Remove or add other parameters as needed
        loss_cfg = {
            'reduction': 'mean',
            # 'entropy_coefficient': self.cfg['entropy_coefficient'],
        }

        return self.loss_list[loss_type](**loss_cfg)

    def __getattr__(self, name):
        return getattr(self.loss_fn, name)

    def __str__(self):
        return str(self.loss_fn)

    def __call__(self, *args, **kwargs):
        return self.loss_fn(*args, **kwargs)

    def set_pos_weight(self, pos_weight):
        if hasattr(self.loss_fn, 'set_pos_weight'):
            self.loss_fn.set_pos_weight(pos_weight)
        else:
            raise ValueError(
                'Loss function does not support setting pos_weight')
