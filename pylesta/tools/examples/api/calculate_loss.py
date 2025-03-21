import torch
from lesta.api.loss import LossFactory
from utils.param import yaml

if __name__ == "__main__":

    cfg = yaml.load("lesta_training/configs/lesta.yaml")
    LOSS_CFG = cfg['LOSS']

    criterion = LossFactory(cfg=LOSS_CFG)

    # Create dummy data
    batch_size = 10
    preds = torch.sigmoid(torch.randn(batch_size, 1)).squeeze()
    targets = torch.randint(0, 2, (batch_size, 1)).float().squeeze()
    risk_weights = torch.tensor([1.0, 2.0, 0.5, 1.5, 0.5, 1.0, 2.0, 0.5, 1.5, 0.5])

    print(preds)
    print(targets)
    print(risk_weights)

    loss = criterion(preds, targets, risk_weights)
    print(loss)

    loss = criterion(preds, targets)
    print(loss)
