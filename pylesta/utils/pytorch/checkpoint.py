"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: utils/pytorch/checkpoint.py
"""

import torch
from datetime import datetime
from pathlib import Path
from typing import Optional


class CheckpointManager:

    def __init__(self,
                 base_dir: str = "checkpoints",
                 experiment_name: Optional[str] = None):

        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True, parents=True)
        self.checkpoint_dir = self.create_checkpoint_dir(experiment_name)

    def create_checkpoint_dir(self, experiment_name: Optional[str] = None) -> Path:
        """
        Expected format: 20250307_185012_experiment_name
        """

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dir_name = timestamp
        if experiment_name:
            dir_name = f"{timestamp}_{experiment_name}"

        checkpoint_dir = self.base_dir / dir_name
        checkpoint_dir.mkdir(exist_ok=True)
        return checkpoint_dir

    def save(self, obj, filename, *args, **kwargs):
        """
        Save a PyTorch object to a file using torch.save.

        Args:
            obj: The object to save
            filename: The filename to save to (relative to checkpoint directory)
            *args, **kwargs: Additional arguments passed directly to torch.save

        Returns:
            Path to the saved checkpoint file
        """
        filepath = self.checkpoint_dir / filename
        torch.save(obj, filepath, *args, **kwargs)
        return str(filepath)

    def save_libtorch(self, model, filename, *args, **kwargs):
        filepath = self.checkpoint_dir / filename
        torch.jit.save(model, filepath, *args, **kwargs)
        return str(filepath)

    def load(self, filename, *args, **kwargs):
        """
        Load a PyTorch object from a file using torch.load.

        Args:
            filename: The filename to load from (relative to checkpoint directory)
            *args, **kwargs: Additional arguments passed directly to torch.load

        Returns:
            The loaded object
        """
        filepath = self.checkpoint_dir / filename
        return torch.load(filepath, *args, **kwargs)
