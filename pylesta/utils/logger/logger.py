"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: logger/logger.py
Date: 2025/02/21 18:50
"""

from dataclasses import dataclass
from .console import Logger as ConsoleLogger
from .tboard import Logger as TensorboardLogger


@dataclass
class Logger:
    print: ConsoleLogger = None
    tboard: TensorboardLogger = None

    def __init__(self, log_dir: str = None, ns: str = None, cfg: dict = None):

        # if both log_dir and cfg is not defined, raise error
        if log_dir is None and cfg is None:
            raise ValueError("[Logger] log_dir or cfg must be defined")

        log_dir = cfg['log_dir'] if log_dir is None else log_dir
        ns = cfg['namespace'] if ns is None else ns

        self.print = ConsoleLogger(log_dir)
        self.tboard = TensorboardLogger(log_dir, ns)
