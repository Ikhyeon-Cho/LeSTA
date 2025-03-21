from lesta.api.model import TraversabilityNetwork
from utils.param import yaml
from lesta.api.dataset import TraversabilityDataset

if __name__ == "__main__":

    cfg = yaml.load("lesta_training/configs/lesta.yaml")
    MODEL_CFG = cfg['MODEL']

    net = TraversabilityNetwork(cfg=MODEL_CFG)
    print(net)

    # (Optional) Dataset can be provided to the model
    DATASET_CFG = cfg['DATASET']
    train_dataset = TraversabilityDataset(cfg=DATASET_CFG, train=True)

    net = TraversabilityNetwork(cfg=MODEL_CFG, dataset=train_dataset)
    print(net)
