import torch
import torch.nn as nn


class NormalizationLayer(nn.Module):
    """
    Do z-score normalization on input features
    """

    def __init__(self, feature_means, feature_stds):
        super().__init__()
        # Register buffers so they're saved with the model
        self.register_buffer("feature_means", torch.tensor(
            feature_means, dtype=torch.float32))
        self.register_buffer("feature_stds", torch.tensor(
            feature_stds, dtype=torch.float32))

    def forward(self, x):
        return (x - self.feature_means) / self.feature_stds


class MLPClassifier(nn.Module):
    """
    A simple MLP traversability classifier.
    """

    def __init__(self,
                 input_dim=5,
                 hidden_dims=[12, 4],  # For real-time inference on CPU
                 output_dim=1,
                 dropout_rate=0.3,
                 cfg=None):
        super(MLPClassifier, self).__init__()

        if cfg is not None:
            input_dim = cfg['input_dim']
            hidden_dims = cfg['hidden_dims']
            output_dim = cfg['output_dim']
            dropout_rate = cfg['dropout_rate']
            feature_means = cfg.get('feature_means', None)
            feature_stds = cfg.get('feature_stds', None)

        self.input_dim = input_dim

        # Build feature normalization layer
        self.feature_normalizer = None
        if feature_means is not None and feature_stds is not None:
            self.feature_normalizer = NormalizationLayer(feature_means,
                                                         feature_stds)

        # Build network layers
        layers = []
        current_dim = input_dim
        for h_dim in hidden_dims:
            layers.append(nn.Linear(current_dim, h_dim))
            layers.append(nn.ReLU())
            layers.append(nn.BatchNorm1d(h_dim))
            layers.append(nn.Dropout(dropout_rate))
            current_dim = h_dim

        # Output layer
        layers.append(nn.Linear(current_dim, output_dim))

        # Initialize network
        self.network = nn.Sequential(*layers)
        self._init_weights()

    def _init_weights(self):
        """Initialize network weights using Kaiming initialization"""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(
                    m.weight, mode='fan_in', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, x):
        if self.feature_normalizer is not None:
            x = self.feature_normalizer(x)

        return self.network(x)
