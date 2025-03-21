"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: logger/console.py
Date: 2024/11/12 22:50
"""

import logging
import os
from datetime import datetime


class Logger:
    """Logger class for handling logging setup and operations"""

    def __init__(self, log_dir: str, filename: str = None):
        # Create logger
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)

        # Clear any existing handlers (to avoid duplicates)
        if self.logger.hasHandlers():
            self.logger.handlers.clear()

        # Set output format
        formatter = logging.Formatter('%(asctime)s -- %(message)s')

        # Add console handler to print logs
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        # Ensure log directory exists
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        if filename is None:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'train_{current_time}.log'

        # Add file handler to save log file
        file_handler = logging.FileHandler(os.path.join(log_dir, filename))
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def info(self, message):
        self.logger.info(message)

    def warning(self, message):
        self.logger.warning(message)

    def error(self, message):
        self.logger.error(message)

    def debug(self, message):
        self.logger.debug(message)


if __name__ == "__main__":
    # Example code
    logger = Logger(log_dir='test', filename='train.log')
    logger.info('Hello, World!')
