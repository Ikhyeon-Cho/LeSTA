import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset

from LeSTA.model import MLPClassifier
from LeSTA.loss.risk_sensitive_loss import WeightedCrossEntropyLoss, EntropyRegularization
from LeSTA.datasets.traversability import TraversabilityDataset
from LeSTA.datasets.transform import *
import os.path as osp
from tqdm import tqdm
import yaml


def train_model(net, dataloader_dict, loss_fn, optimizer, num_epochs):

    # Initialize entropy regularization
    entropy_reg = EntropyRegularization(lambda_entropy=0.1)

    for epoch in range(num_epochs):

        print(f'Epoch [{epoch+1}/{num_epochs}]')
        print('-----------')

        for phase in ['train', 'val']:
            if phase == 'train':
                net.train()
            else:
                net.eval()

            epoch_loss = 0.0
            epoch_corrects = 0

            for inputs, labels, sample_weights in tqdm(dataloader_dict[phase]):

                # Initialize optimizer
                optimizer.zero_grad()

                with torch.set_grad_enabled(phase == 'train'):
                    # Forward pass
                    outputs = net(inputs)  # Shape: [batch_size, 1]
                    # Backward pass
                    if phase == 'train':
                        loss = loss_fn(outputs, labels, sample_weights)
                        entropy_loss = entropy_reg(outputs, sample_weights)
                        total_loss = loss + entropy_loss
                        total_loss.backward()
                        optimizer.step()
                    else:
                        loss = nn.BCEWithLogitsLoss()(outputs, labels)

                    # Update epoch loss and accuracy
                    epoch_loss += loss.item() * inputs.size(0)
                    preds = (torch.sigmoid(outputs) > 0.5).float()
                    epoch_corrects += torch.sum(preds == labels.data)

            epoch_loss = epoch_loss / len(dataloader_dict[phase].dataset)
            epoch_acc = epoch_corrects.double(
            ) / len(dataloader_dict[phase].dataset)

            print(f'{phase} Loss: {epoch_loss: .4f} \n{
                  phase} Acc: {epoch_acc: .4f}')
            print()


def pseudo_labeling(net, unlabeled_dataloader, threshold):

    net.eval()
    unlabeled_set = unlabeled_dataloader.dataset

    # Early return if no unlabeled samples
    if len(unlabeled_set) == 0:
        print('No unlabeled samples to pseudo-label.')
        return None, None  # pseudo-labeled set, remaining unlabeled set

    with torch.no_grad():
        # Get predictions for all unlabeled data
        inputs, _, _ = next(iter(unlabeled_dataloader))
        outputs = net(inputs)
        pred_confidence = torch.sigmoid(outputs).squeeze()

        # Select high confidence predictions
        pos_mask = pred_confidence > threshold
        neg_mask = pred_confidence < (1 - threshold)
        confident_mask = pos_mask | neg_mask

        if not torch.any(confident_mask):
            print('No confident predictions found.')
            return None, unlabeled_set

        # Create pseudo-labeled dataset
        confident_features = unlabeled_set.features[confident_mask]
        confident_labels = torch.where(
            pred_confidence[confident_mask] > 0.5,
            # 1 for traversable
            torch.ones_like(pred_confidence[confident_mask]),
            # 0 for non-traversable
            torch.zeros_like(pred_confidence[confident_mask])
        ).unsqueeze(1)
        pseudo_labeled_dataset = TraversabilityDataset(
            features=confident_features,
            labels=confident_labels,
            transform=unlabeled_set.transform
        )
        print(f'Found {len(pseudo_labeled_dataset)} confident predictions.')

        # Create remaining dataset
        remaining_mask = ~confident_mask
        if torch.any(remaining_mask):
            remaining_dataset = TraversabilityDataset(
                features=unlabeled_set.features[remaining_mask],
                labels=unlabeled_set.labels[remaining_mask],
                transform=unlabeled_set.transform
            )
        else:
            remaining_dataset = None

        if remaining_dataset:
            print(f'Remaining unlabeled samples: {len(remaining_dataset)}')

        return pseudo_labeled_dataset, remaining_dataset


def self_training(net, dataloader_dict, loss_fn, optimizer, num_epochs,
                  confidence_threshold=0.9, max_iter=10):

    for iter in range(max_iter):
        print(f'\n\033[32mSelf-training iteration[{iter+1}/{max_iter}]\033[0m')

        print('-')
        print('\033[33m[1] Training from self-supervised labels...\033[0m\n')

        # Step 1: train_model()
        train_model(net, dataloader_dict, loss_fn, optimizer, num_epochs)

        print('-')
        print(f'\033[33m[2] Pseudo-labeling at round {iter+1}...\033[0m')

        # Step 2: pseudo_labeling()
        pseudo_labeled_set, remaining_set = pseudo_labeling(
            net, dataloader_dict['unlabeled'], confidence_threshold)

        if pseudo_labeled_set is None:
            print('No more pseudo-labels found. Ending self-training...')
            break

        if remaining_set is None:
            print('No unlabeled samples remaining. Ending self-training...')
            break

        # Step 3: Data augmentation: Combine pseudo-labels
        combined_dataset = torch.utils.data.ConcatDataset([
            dataloader_dict['train'].dataset,
            pseudo_labeled_set
        ])
        dataloader_dict['train'] = DataLoader(
            combined_dataset,
            batch_size=dataloader_dict['train'].batch_size,
            shuffle=True
        )

        # Step 4: Update unlabeled data
        dataloader_dict['unlabeled'] = DataLoader(
            remaining_set,
            batch_size=len(remaining_set),
            shuffle=False
        )

        print(f"Added {len(pseudo_labeled_set)} new labeled samples.")
        print(f'New training samples: {len(dataloader_dict["train"].dataset)}')
        print(
            f'Remaining unlabeled samples: {len(dataloader_dict["unlabeled"].dataset)}')

        # Step 5: Repeat until termination

    return net


def save_python_checkpoint(model, optimizer, epoch, path):
    checkpoint = {
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
    }
    torch.save(checkpoint, path)
    print(f'Python checkpoint saved to {path}')


def load_python_checkpoint(model, optimizer, path):
    if osp.isfile(path):
        checkpoint = torch.load(path)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        epoch = checkpoint['epoch']
        print(f'Python checkpoint loaded from {path}, epoch {epoch}')
        return epoch
    else:
        print(f'No checkpoint found at {path}')
        return 0


def save_for_libtorch(model, path):
    model.eval()
    traced_model = torch.jit.script(model)
    traced_model.save(path)
    print(f'LibTorch model saved to {path}')


if __name__ == '__main__':

    # Load config
    project_path = osp.dirname(osp.dirname(__file__))
    cfg_path = osp.join(
        project_path, 'lesta_training/configs', 'example_config.yaml')
    with open(cfg_path, 'r') as file:
        cfg = yaml.safe_load(file)

    # Load labeled dataset
    print('\033[33m[0] Loading dataset...\033[0m')

    labeled_set = TraversabilityDataset(path=cfg['labeled_data_path'],
                                        download=False)
    transform = RiskBasedWeighting(dataset=labeled_set,
                                   # step, slope, roughness, curvature, variance
                                   importance_weight=[1, 1, 1, 1, 1])

    # Split labeled dataset
    training_set, validation_set = TraversabilityDataset.random_split(
        labeled_set,
        training_ratio=cfg['training_ratio'],
        seed=cfg['random_seed']
    )
    unlabeled_set = TraversabilityDataset(path=cfg['unlabeled_data_path'],
                                          transform=transform,
                                          download=False)

    # Save dataloaders in a dict format
    batch_size = cfg['batch_size']
    dataloader_dict = {
        'train': DataLoader(training_set, batch_size=batch_size, shuffle=True),
        'val': DataLoader(validation_set, batch_size=batch_size, shuffle=False),
        "unlabeled": DataLoader(unlabeled_set, batch_size=len(unlabeled_set), shuffle=False)
    }
    print(f'Training samples: {len(training_set)}')
    print(f'Validation samples: {len(validation_set)}')
    print(f'Unlabeled samples: {len(unlabeled_set)}')

    # Initialize model, optimizer
    input_dim = labeled_set.features.shape[1]
    net = MLPClassifier(input_dim=input_dim)
    loss_fn = WeightedCrossEntropyLoss()
    optimizer = optim.Adam(net.parameters(), lr=cfg['learning_rate'])

    # Train model
    try:
        self_training(net,
                      dataloader_dict,
                      loss_fn,
                      optimizer,
                      num_epochs=cfg['num_epochs'],
                      confidence_threshold=cfg['pseudo_labeling_threshold'],
                      max_iter=cfg['self_training_iterations'])

    # Save model for LibTorch usage
        print('\033[32m\nTraining completed. Saving model for LibTorch...\033[0m')
        save_for_libtorch(net, path=cfg['checkpoint_path'])

    except KeyboardInterrupt:
        print('\033[32m\nTraining interrupted. Saving model for LibTorch...\033[0m')
        save_for_libtorch(net, path=cfg['checkpoint_path'])
