"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/logger.py
"""

from utils.logger import Logger


class LestaLogger():
    """
    A Logger class for monitoring the training process.
    """

    def __init__(self, cfg: dict):
        self.logger = Logger(cfg=cfg)
        self.log_interval = cfg['log_period']

    def log_self_training_round(self, round, max_round):
        st_round = (
            f'\033[33m Self-training round [{round+1}/{max_round}] \033[0m')
        self.logger.print.info(st_round)

    def log_train_info(self, epoch, num_epochs, round, lr):
        epoch_info = (
            f'\033[32m ====== Epoch {epoch+1}/{num_epochs} (Round {round+1}) ====== \033[0m')
        lr_info = (f'Learning rate: {lr:.6f}')
        self.logger.print.info('')
        self.logger.print.info(epoch_info)
        self.logger.print.info(lr_info)
        self.logger.print.info('')

    def log_batch_loss(self, batch_loss, batch_idx, max_batch):
        batch_loss_info = (
            f"Step [{batch_idx+1}/{max_batch}] | "
            f"Batch loss: {batch_loss:.4f}"
        )
        if (batch_idx == 0) or (batch_idx+1) % self.log_interval == 0:
            self.logger.print.info(batch_loss_info)

    def log_phase_transition(self):
        info_header = (f'Training epoch completed. Validating...')
        self.logger.print.info('')
        self.logger.print.info(info_header)

    def log_epoch_summary(self, epoch, train_loss, val_loss):
        epoch_summary_header = (
            f'\033[32m ====== Epoch {epoch} Summary ====== \033[0m')
        train_loss_info = (f"Training Loss: {train_loss:.4f}")
        val_loss_info = (f"Validation Loss: {val_loss:.4f}")
        self.logger.print.info('')
        self.logger.print.info(epoch_summary_header)
        self.logger.print.info(train_loss_info)
        self.logger.print.info(val_loss_info)

    def log_best_model_saved(self, val_loss):
        best_model_info = (
            f'\033[33m✓ New best model saved with validation loss: {val_loss:.4f} \033[0m')
        self.logger.print.info(best_model_info)

    def log_pseudo_labeling_result(self, confidence_threshold, pseudo_labeling_ratio, n_labeled, n_unlabeled):
        header = (f'\033[32m ====== Pseudo-labeling result ====== \033[0m')
        confidence_threshold = (
            f"Confidence threshold: {confidence_threshold}")
        pseudo_labeling_ratio = (
            f"Pseudo-labeling ratio: {pseudo_labeling_ratio*100:.2f}%")
        labeled_set_info = (
            f"Total labeled samples: {n_labeled}")
        unlabeled_set_info = (
            f"Remaining unlabeled samples: {n_unlabeled}")
        self.logger.print.info('')
        self.logger.print.info(header)
        self.logger.print.info(confidence_threshold)
        self.logger.print.info(pseudo_labeling_ratio)
        self.logger.print.info(labeled_set_info)
        self.logger.print.info(unlabeled_set_info)
        self.logger.print.info('')

    def log_no_unlabeled_samples(self):
        self.logger.print.info('No unlabeled samples for pseudo-labeling.')

    def log_stopping_criteria(self, pseudo_labeing_ratio, criteria_ratio):
        quitting_info = (
            f'\033[33m Stopping criteria met. Quitting...\033[0m')
        criteria_info = (
            f"Requires pseudo-labeling ratio over {criteria_ratio*100:.2f}%")
        pseudo_labeling_ratio_info = (
            f"Current pseudo-labeling ratio: {pseudo_labeing_ratio*100:.2f}%")
        self.logger.print.info(quitting_info)
        self.logger.print.info(criteria_info)
        self.logger.print.info(pseudo_labeling_ratio_info)
