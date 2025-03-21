"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: logger/tboard.py
Date: 2024/11/2 18:50
"""

from torch.utils.tensorboard import SummaryWriter
import torch
import numpy as np


def Tensorboard(log_dir: str):
    """Get TensorBoard logger instance"""
    return SummaryWriter(log_dir=log_dir)


class Logger:
    def __init__(self, log_dir: str, ns: str = None):
        self.tb_writer = SummaryWriter(log_dir=log_dir + '/events')
        self.namespace = ns

    def log_batch_loss(self, loss: dict, steps: list = None):
        """Log batch losses to tensorboard.

        Args:
            batch_loss: Dictionary of batch losses {loss_name: loss_values}
            steps: List of steps corresponding to batch losses
        """
        if not isinstance(loss, dict):
            raise TypeError(
                'TensorboardLogger: Batch loss must be a dictionary')

        if steps is None:
            for loss_name, loss_vals in loss.items():
                steps = list(range(len(loss_vals)))
                break

        for loss_name, loss_vals in loss.items():
            loss_vals = torch.stack(loss_vals).cpu().numpy()
            steps = np.array(steps)

            # Handle tensorboard namespace
            tag = f'Batch Loss/{loss_name}'
            if self.namespace:
                tag = f'{self.namespace} {tag}'

            loss_tracked = zip(loss_vals, steps)
            for val, step in loss_tracked:
                self.tb_writer.add_scalar(tag, val, step)

    def log_epoch_loss(self, loss: dict, epoch: int):
        """Write mean epoch losses to tensorboard.

        Args:
            loss: Dictionary of epoch losses {loss_name: avg_loss}
            epoch: Current epoch number
        """
        for loss_name, avg_loss in loss.items():
            tag = f'Epoch Loss/{loss_name}'
            if self.namespace:
                tag = f'{self.namespace} {tag}'
            self.tb_writer.add_scalar(tag, avg_loss, epoch)

    def log_lr(self, lr: float, step: int):
        """Log learning rate to tensorboard.

        Args:
            lr: Learning rate
            step: step number
        """
        self.tb_writer.add_scalar('Learning Rate', lr, step)


if __name__ == "__main__":
    pass
