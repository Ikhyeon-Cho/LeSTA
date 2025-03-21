"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/trainer.py
"""

import torch
from torch.utils.data import DataLoader
from lesta.api.logger import LestaLogger
from lesta.api.checkpoint import LestaSaver


class LestaTrainer:
    def __init__(self, network, datasets, criterion, optimizer, scheduler, device, cfg):

        # Setup model and datasets
        self.network = network.to(device)
        self.labeled_dataset = datasets['train'].get_labeled_set()
        self.unlabeled_dataset = datasets['train'].get_unlabeled_set()

        # Check if validation dataset is provided
        self.has_validation = 'val' in datasets
        self.val_dataset = datasets.get('val', None)

        # Training components
        criterion.set_pos_weight(self.labeled_dataset.get_pos_weight())
        self.criterion = criterion
        self.optimizer = optimizer
        self.scheduler = scheduler
        self.device = device

        # Load logging and checkpointing
        self.cfg = cfg
        self.logger = LestaLogger(cfg=cfg)
        self.chkpt_saver = LestaSaver(cfg=cfg)

        # Load self-training configs
        self.num_st_iteration = self.cfg['max_iterations']
        self.st_confidence_threshold = self.cfg['confidence_threshold']
        self.min_pseudo_labeling_ratio = self.cfg['min_pseudo_labeling_ratio']

        # Load training configs
        self.num_epochs = self.cfg['num_epochs']
        self.batch_size = self.cfg['batch_size']
        self.scheduler_period = self.cfg['scheduler_period']
        self.checkpoint_save_period = self.cfg['checkpoint_save_period']

        # Initialize training state variables
        self.epoch = 0
        self.global_step = 0
        self.train_batch_losses = []
        self.train_epoch_losses = []
        self.val_batch_losses = []
        self.val_epoch_losses = []

        # Initialize pseudo-labeling state
        self.pseudo_labeling_ratio = 0.0

    @property
    def lr(self):
        return self.optimizer.param_groups[0]['lr']

    def run(self):
        """
        Main self-training loop.
        """
        for st_iteration in range(self.num_st_iteration):
            self.st_iteration = st_iteration
            self.logger.log_self_training_round(
                round=st_iteration,
                max_round=self.num_st_iteration
            )
            # Training & validation
            self.train_model()

            # Pseudo-labeling
            self.pseudo_labeling()

            self.logger.log_pseudo_labeling_result(
                self.st_confidence_threshold,
                self.pseudo_labeling_ratio,
                len(self.labeled_dataset),
                len(self.unlabeled_dataset)
            )

            if self.check_stopping_criteria():
                return

    def train_model(self):
        """
        Train and validate the model.
        """
        for epoch in range(self.num_epochs):
            self.logger.log_train_info(
                epoch=epoch,
                num_epochs=self.num_epochs,
                round=self.st_iteration,
                lr=self.lr
            )

            # Train for one epoch
            self._train_one_epoch()
            self._update_training_metrics()

            # Update learning rate if scheduled per epoch
            if self.scheduler_period == 'epoch':
                self._update_lr()

            # Validate model
            if self.has_validation:
                self._validate_model()
                val_loss = self.val_epoch_losses[-1]
            else:
                val_loss = float('inf')

            self.logger.log_epoch_summary(
                epoch=self.epoch,
                train_loss=self.train_epoch_losses[-1],
                val_loss=val_loss
            )

            # Save checkpoint
            self.save_checkpoint()

    def _train_one_epoch(self):
        """
        Train the model using labeled dataset.
        Expected batch:
            batch = {
                'feats': torch.Tensor,  # Input features
                'label': torch.Tensor,  # Ground truth label
                'risk_weights': torch.Tensor,  # Risk weights
            }
        """
        self.network.train()
        self.train_loader = DataLoader(
            dataset=self.labeled_dataset,
            batch_size=self.batch_size,
            num_workers=2,
            shuffle=True
        )

        for batch_idx, batch in enumerate(self.train_loader):
            # Prepare inputs
            inputs = batch['feats'].to(self.device)
            labels = batch['label'].to(self.device)
            risk_weights = batch['risk_weights'].to(self.device)

            # Forward pass
            outputs = self.network(inputs).squeeze()
            loss = self.criterion(outputs, labels, risk_weights)
            # loss = self.criterion(outputs, labels)

            # Backward pass and optim
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            # Update learning rate if scheduled per batch
            if self.scheduler_period == 'batch':
                if self.scheduler:
                    self.scheduler.step()

            # Update training state
            self.global_step += 1
            self.train_batch_losses.append(loss.item())

            # Log progress
            self.logger.log_batch_loss(
                batch_loss=loss.item(),
                batch_idx=batch_idx,
                max_batch=len(self.train_loader)
            )
        return

    def _validate_model(self):
        """
        Validate the model on the validation set.
        """
        self.logger.log_phase_transition()

        self.network.eval()
        self.val_loader = DataLoader(
            self.val_dataset,
            batch_size=self.batch_size,
            num_workers=2,
            shuffle=False
        )

        with torch.no_grad():
            for batch in self.val_loader:
                # Prepare inputs
                inputs = batch['feats'].to(self.device)
                labels = batch['label'].to(self.device)

                # Forward pass
                outputs = self.network(inputs).squeeze()
                loss = self.criterion(outputs, labels)

                # Record validation loss
                self.val_batch_losses.append(loss.item())

        # Calculate and record epoch validation loss
        epoch_loss = sum(self.val_batch_losses) / len(self.val_batch_losses)
        self.val_epoch_losses.append(epoch_loss)
        self.val_batch_losses = []  # Reset validation batch loss
        return

    def save_checkpoint(self):
        """
        Save the model, optimizer, and scheduler for checkpointing.
        Also saves the best model when validation loss improves.
        """
        use_libtorch = self.cfg.get('save_for_libtorch', False)

        # Regular checkpoint saving based on period
        if self.epoch % self.checkpoint_save_period == 0:
            self._save_model(use_libtorch, self.epoch)

        # Save best model based on validation loss
        if self._is_best_model():
            self._save_model(use_libtorch, "best")
            self.logger.log_best_model_saved(self.val_epoch_losses[-1])

    def _is_best_model(self):
        """Check if current model is the best so far based on validation loss."""
        if not self.val_epoch_losses:
            return False

        if len(self.val_epoch_losses) == 1:
            return True

        return self.val_epoch_losses[-1] < min(self.val_epoch_losses[:-1])

    def pseudo_labeling(self):
        """
        Perform pseudo-labeling on unlabeled data.
        """
        # Early return if no unlabeled samples
        if len(self.unlabeled_dataset) == 0:
            self.logger.log_no_unlabeled_samples()
            return

        self.network.eval()
        unlabeled_loader = DataLoader(
            self.unlabeled_dataset,
            batch_size=self.batch_size,
            num_workers=2,
            shuffle=False
        )

        pseudo_labels = []
        confident_masks = []

        with torch.no_grad():
            for batch in unlabeled_loader:
                inputs = batch['feats'].to(self.device)

                # Get model predictions
                outputs = self.network(inputs)
                probabilities = torch.sigmoid(outputs).squeeze()

                # Handle single-item batch case
                if probabilities.dim() == 0:
                    probabilities = probabilities.unsqueeze(0)

                # Identify confident predictions
                confident_mask = (probabilities >= self.st_confidence_threshold) | (
                    probabilities <= (1 - self.st_confidence_threshold))

                # Skip if no confident predictions in this batch
                if not confident_mask.any():
                    confident_masks.append(confident_mask.cpu())
                    continue

                # Generate pseudo labels
                pseudo_label = torch.zeros_like(probabilities)
                # Set high confidence positives to 1
                pseudo_label[confident_mask & (probabilities >= 0.5)] = 1.0
                # Mark unconfident predictions as -1
                pseudo_label[~confident_mask] = -1.0

                pseudo_labels.append(pseudo_label[confident_mask].cpu())
                confident_masks.append(confident_mask.cpu())

        # Early return if no confident predictions
        if len(pseudo_labels) == 0:
            self.pseudo_labeling_ratio = 0.0
            return

        # Process collected pseudo labels
        pseudo_labels = torch.cat(pseudo_labels).numpy()
        confident_masks = torch.cat(confident_masks).numpy()
        feats = self.unlabeled_dataset.feature_vectors[confident_masks]

        # Calculate pseudo-labeling ratio
        self.pseudo_labeling_ratio = confident_masks.sum() / len(self.unlabeled_dataset)

        # Update datasets with pseudo labels
        self.labeled_dataset.add_data(feats, pseudo_labels)
        self.unlabeled_dataset.remove_data(confident_masks)

    def check_stopping_criteria(self):
        """
        Check if the pseudo-labeling generates enough confident predictions.
        """
        if self.pseudo_labeling_ratio <= self.min_pseudo_labeling_ratio:
            self.logger.log_stopping_criteria(
                self.pseudo_labeling_ratio,
                self.min_pseudo_labeling_ratio
            )
            return True
        return False

    def _update_training_metrics(self):
        """Update training metrics after each epoch."""
        # Calculate and record epoch training loss
        epch_loss = sum(self.train_batch_losses) / len(self.train_batch_losses)
        self.train_epoch_losses.append(epch_loss)
        self.train_batch_losses = []  # Reset batch loss

        # Increment epoch counter
        self.epoch += 1

    def _update_lr(self):
        """Update learning rate using scheduler if available."""
        if self.scheduler:
            self.scheduler.step()

    def _save_model(self, use_libtorch, epoch):
        """Save model checkpoint."""
        if use_libtorch:
            self.chkpt_saver.save_for_libtorch(self.network, epoch)
        else:
            self.chkpt_saver.save(
                self.network, self.optimizer, self.scheduler, epoch)
