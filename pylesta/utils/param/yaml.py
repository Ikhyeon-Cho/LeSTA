"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: config_tools.py
Date: 2024/11/2 18:50
"""

import os
import yaml as pyyaml


def load(yaml_path):
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"The file {yaml_path} does not exist.")
    return pyyaml.safe_load(open(yaml_path, 'r'))
