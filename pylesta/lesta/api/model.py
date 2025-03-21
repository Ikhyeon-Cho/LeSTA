"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho/LeSTA
File: lesta/api/model.py
"""

from ..core.models import MLPClassifier


class TraversabilityNetwork:
    """
    A wrapper model class with a unified interface for the other model classes.
    """
    # Add your custom model class here
    model_list = {
        'mlp': MLPClassifier,
    }

    def __init__(self, cfg, dataset=None):
        model_type = cfg['type'].lower()

        if dataset is not None:
            # If dataset is provided, add normalization constants to the config
            use_normalization_layer = cfg.get('use_normalization_layer', True)
            if use_normalization_layer and hasattr(dataset, 'get_normalization_constants'):
                feature_means, feature_stds = dataset.get_normalization_constants()
                cfg = cfg.copy()  # Create a copy to avoid modifying the original
                cfg['feature_means'] = feature_means
                cfg['feature_stds'] = feature_stds

        self.model = self._build_model(model_type, cfg)

    def _build_model(self, model_type: str, cfg: dict):
        if model_type not in self.model_list:
            raise ValueError(f"Model type {model_type} not implemented")
        return self.model_list[model_type](cfg=cfg)

    def __getattr__(self, name):
        return getattr(self.model, name)

    def __str__(self):
        return str(self.model)
