import argparse

from utils.param import yaml
from utils.pytorch import seed, machine
from utils.pytorch.optimizer import TrainingOptimizer
from lesta.api.dataset import TraversabilityDataset
from lesta.api.model import TraversabilityNetwork
from lesta.api.loss import LossFactory
from lesta.api.trainer import LestaTrainer

def main(args):

    CFG_PATH = args.config
    TRAINING_SET = args.dataset

    print(f'\033[32m========= Training config: "{CFG_PATH}" =========\033[0m')
    print('=> Loading configs...')

    # Load configs and extract components
    cfg = yaml.load(CFG_PATH)
    DATASET_CFG = cfg['DATASET']
    MODEL_CFG = cfg['MODEL']
    LOSS_CFG = cfg['LOSS']
    OPTIMIZER_CFG = cfg['OPTIMIZER']
    TRAINER_CFG = cfg['TRAINER']

    if TRAINING_SET is not None:
        DATASET_CFG['training_data'] = TRAINING_SET

    # Setup environment
    seed.seed_all(42)
    device = machine.get_device()

    # Load datasets
    print(f'=> Loading {DATASET_CFG["type"]}...')
    train_dataset = TraversabilityDataset(cfg=DATASET_CFG, train=True)
    print(f' - {len(train_dataset)} training samples')
    dataset_dict = {'train': train_dataset}
    if DATASET_CFG['validation_data']:
        val_dataset = TraversabilityDataset(cfg=DATASET_CFG, train=False)
        print(f' - {len(val_dataset)} validation samples')
        dataset_dict['val'] = val_dataset
    else:
        print('\033[33m - No validation dataset found\033[0m')

    # Initialize model
    print(f'=> Loading {MODEL_CFG["type"]} model...')
    net = TraversabilityNetwork(cfg=MODEL_CFG, dataset=train_dataset)
    print(net)

    # Setup training components
    print('=> Initializing trainer...')
    print('=> Using device:', device)
    criterion = LossFactory(cfg=LOSS_CFG)
    optim = TrainingOptimizer(model=net, cfg=OPTIMIZER_CFG)

    # Create and run trainer
    trainer = LestaTrainer(
        network=net,
        datasets=dataset_dict,
        criterion=criterion,
        optimizer=optim.optimizer,
        scheduler=optim.scheduler,
        device=device,
        cfg=TRAINER_CFG
    )

    print('=> Start training:')
    trainer.run()

    chkpt_dir = TRAINER_CFG['checkpoint_dir']
    print(
        f'\033[32m====== Training done. Saved at: {chkpt_dir}/ ======\033[0m')


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str,
                        default='pylesta/configs/lesta.yaml')
    parser.add_argument('--dataset', type=str,
                        default=None)

    args = parser.parse_args()
    main(args)
