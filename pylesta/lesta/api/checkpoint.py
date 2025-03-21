"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/checkpoint.py
"""

from utils.pytorch.checkpoint import CheckpointManager
import torch
import os.path as osp


class LestaSaver:
    """
    A checkpoint saver class for saving the model, optimizer, and scheduler.
    """

    def __init__(self, cfg: dict):
        self.chkpt_dir = cfg['checkpoint_dir']
        self.chkpt_saver = CheckpointManager(base_dir=self.chkpt_dir)

    def save(self, model, optimizer, scheduler, epoch):

        checkpoint = {
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
        }
        if scheduler:
            checkpoint['scheduler_state_dict'] = scheduler.state_dict()

        self.chkpt_saver.save(checkpoint, f'epoch_{epoch}.pth')

    def save_for_libtorch(self, model, epoch, example_input=None):

        model.eval()
        device = next(model.parameters()).device

        if example_input is None:
            example_input = torch.randn(1, model.input_dim).to(device)

        try:
            traced_model = torch.jit.trace(model, example_input)
            self.chkpt_saver.save_libtorch(traced_model, f'epoch_{epoch}.pt')
        except Exception as e:
            print(f'Error saving model for LibTorch: {e}')
            raise e
