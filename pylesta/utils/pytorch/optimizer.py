"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: utils/pytorch/optimizer.py
"""

import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler


class TrainingOptimizer:
    def __init__(self, model, cfg: dict):
        self.optimizer_type = cfg['type'].lower()
        self.scheduler_type = cfg.get('scheduler', 'none').lower()
        self.optimizer = self._build_optimizer(model, cfg)
        self.scheduler = self._build_scheduler(cfg)

    def _build_optimizer(self, model, cfg: dict):
        """Builds the optimizer based on configuration."""
        if self.optimizer_type == 'adam':
            return optim.Adam(
                model.parameters(),
                lr=cfg['learning_rate'],
                betas=(cfg.get('Adam_beta1', 0.9),
                       cfg.get('Adam_beta2', 0.999))
            )
        elif self.optimizer_type == 'sgd':
            return optim.SGD(
                model.parameters(),
                lr=cfg['learning_rate'],
                momentum=cfg.get('SGD_momentum', 0.9),
                weight_decay=cfg.get('SGD_weight_decay', 0)
            )
        else:
            raise ValueError(
                f"Optimizer type {self.optimizer_type} not implemented")

    def _build_scheduler(self, cfg: dict):
        """Builds the scheduler based on configuration."""

        if self.scheduler_type == 'none':
            return None
        elif self.scheduler_type == 'step':
            return lr_scheduler.StepLR(
                self.optimizer,
                step_size=cfg.get('scheduler_step_size', 10),
                gamma=cfg.get('scheduler_gamma', 0.1)
            )
        elif self.scheduler_type == 'exponential':
            return lr_scheduler.ExponentialLR(
                self.optimizer,
                gamma=cfg.get('scheduler_gamma', 0.98)
            )
        elif self.scheduler_type == 'plateau':
            return lr_scheduler.ReduceLROnPlateau(
                self.optimizer,
                mode='min',
                factor=cfg.get('scheduler_gamma', 0.1),
                patience=cfg.get('scheduler_patience', 10)
            )
        else:
            raise ValueError(
                f"Scheduler type {self.scheduler_type} not implemented")

    def __str__(self):
        return f"{self.optimizer_type.capitalize()} optimizer with " + \
               (f"{self.scheduler_type} scheduler" if self.scheduler else "no scheduler")
