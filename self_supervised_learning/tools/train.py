import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

from lesta_api.models import MLPClassifier
from lesta_api.dataloader import TraversabilityDataset

import os
from tqdm import tqdm
import yaml


def train_model(net, dataloader_dict, criterion, optimizer, num_epochs):

    for epoch in range(num_epochs):  # epoch loop

        print(f'Epoch [{epoch+1}/{num_epochs}]')
        print('-----------')

        for phase in ['train', 'val']:  # training & validation loop per epoch
            if phase == 'train':
                net.train()
            else:
                net.eval()

            epoch_loss = 0.0
            epoch_corrects = 0

            for inputs, labels in tqdm(dataloader_dict[phase]):
                # initialize optimizer
                optimizer.zero_grad()

                # forward pass
                with torch.set_grad_enabled(phase == 'train'):
                    outputs = net(inputs)
                    loss = criterion(outputs, labels)
                    preds = (outputs > 0.5).float()  # binary classification

                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                    epoch_loss += loss.item() * inputs.size(0)
                    epoch_corrects += torch.sum(preds == labels.data)

            epoch_loss = epoch_loss / len(dataloader_dict[phase].dataset)
            epoch_acc = epoch_corrects.double(
            ) / len(dataloader_dict[phase].dataset)

            print(f'{phase} Loss: {epoch_loss:.4f} \n{
                  phase} Acc: {epoch_acc:.4f}')
            print()


def pseudo_labeling(net, unlabeled_dataset, threshold):

    net.eval()
    pseudo_labeled_data = []
    pseudo_labels = []

    with torch.no_grad():

        unlabeled_dataloader = DataLoader(
            unlabeled_dataset,
            batch_size=len(unlabeled_dataset),
            shuffle=False)

        for inputs, _ in tqdm(unlabeled_dataloader):

            outputs = net(inputs)
            predictive_confidence = outputs.squeeze()

            # Find confident predictions
            confident_mask = predictive_confidence > threshold
            confident_inputs = inputs[confident_mask]
            confident_preds = predictive_confidence[confident_mask]

            if len(confident_inputs) > 0:
                pseudo_labeled_data.append(confident_inputs)
                pseudo_labels.append(torch.ones_like(
                    confident_preds).unsqueeze(1))

            # Find confident negative predictions
            confident_mask_negatives = predictive_confidence < (1 - threshold)
            confident_inputs_negatives = inputs[confident_mask_negatives]
            confident_preds_negatives = predictive_confidence[confident_mask_negatives]

            if len(confident_inputs_negatives) > 0:
                pseudo_labeled_data.append(confident_inputs_negatives)
                pseudo_labels.append(
                    torch.zeros_like(confident_preds_negatives).unsqueeze(1))

    # Remove pseudo-labeled data from the unlabeled dataset
    unconfident_mask = ~confident_mask & ~confident_mask_negatives
    remaining_unlabeled_dataset = torch.utils.data.Subset(
        unlabeled_dataset, torch.where(unconfident_mask)[0])

    if len(pseudo_labeled_data) > 0:
        pseudo_labeled_data = torch.cat(pseudo_labeled_data)
        pseudo_labels = torch.cat(pseudo_labels)
        pseudo_labeled_dataset = torch.utils.data.TensorDataset(
            pseudo_labeled_data, pseudo_labels)

        print(f'Found {len(pseudo_labeled_dataset)} confident predictions.')
        return pseudo_labeled_dataset, remaining_unlabeled_dataset
    else:
        print('No confident predictions found.')
        return None, None


def self_training(net, dataloader_dict, criterion, optimizer,
                  threshold=0.9, max_iter=10):

    for iteration in range(max_iter):
        print()
        print(f'Self-training iteration [{iteration + 1}/{max_iter}]')

        # step 1: model training
        print('-')
        print('[1] Training from self-supervised labels...\n')
        train_model(net,
                    dataloader_dict,
                    criterion,
                    optimizer,
                    num_epochs=5)

        # step 2: pseudo-labeling
        print('-')
        print(f'[2] Pseudo-labeling at round {iteration + 1}...')
        pseudo_labeled_dataset, unlabeled_dataset = pseudo_labeling(
            net,
            dataloader_dict['unlabeled'].dataset,
            threshold)

        if pseudo_labeled_dataset is None:
            print('No confident predictions found. Ending self-training...')
            break

        # step 3: Add pseudo-labeled data to the labeled dataset
        combined_dataset = torch.utils.data.ConcatDataset(
            [dataloader_dict['train'].dataset, pseudo_labeled_dataset])

        dataloader_dict['train'] = DataLoader(
            combined_dataset,
            batch_size=dataloader_dict['train'].batch_size,
            shuffle=True)
        dataloader_dict['unlabeled'] = DataLoader(
            unlabeled_dataset,
            batch_size=dataloader_dict['unlabeled'].batch_size,
            shuffle=False)

        print(f"Added {len(pseudo_labeled_dataset)} new labeled samples.")

        print(f'New training samples: {len(combined_dataset)}')
        print(f'Remaining unlabeled samples: {
              len(dataloader_dict["unlabeled"].dataset)}')

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
    if os.path.isfile(path):
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

    # Load training parameters
    project_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_path, 'config',
                               'training_params.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    batch_size = config['batch_size']
    num_epochs = config['num_epochs']
    lr = config['learning_rate']
    test_ratio = config['test_ratio']
    manual_seed = config['random_seed']
    checkpoint_save_path = os.path.join(
        project_path, config['checkpoint_root'])

    # Load dataset
    labeled_dataset_path = os.path.join(
        project_path, config['labeled_data_root'])
    labeled_dataset = TraversabilityDataset(csv_file=labeled_dataset_path)

    unlabeled_dataset_path = os.path.join(
        project_path, config['unlabeled_data_root'])
    unlabeled_dataset = TraversabilityDataset(csv_file=unlabeled_dataset_path)

    terrain_feature_dim = labeled_dataset.features.shape[1]
    traversability_network = MLPClassifier(input_dim=terrain_feature_dim)
    criterion = nn.BCELoss()
    optimizer = optim.Adam(traversability_network.parameters(), lr=lr)

    train_dataset, val_dataset = labeled_dataset.random_split(
        test_ratio=test_ratio, manual_seed=manual_seed)
    dataloader_dict = {
        'train': DataLoader(train_dataset, batch_size=batch_size, shuffle=True),
        'val': DataLoader(val_dataset, batch_size=batch_size, shuffle=False),
        "unlabeled": DataLoader(unlabeled_dataset, batch_size=len(unlabeled_dataset), shuffle=False)
    }
    print(f'Training samples: {len(train_dataset)}')
    print(f'Validation samples: {len(val_dataset)}')
    print(f'Unlabeled samples: {len(unlabeled_dataset)}')

    # train
    try:
        self_training(traversability_network, dataloader_dict,
                      criterion, optimizer, threshold=0.9, max_iter=10)
        print('\nTraining completed. Saving model for LibTorch...')
        save_for_libtorch(traversability_network, checkpoint_save_path)

    except KeyboardInterrupt:
        print('\nTraining interrupted. Saving Python checkpoint...')
        save_python_checkpoint(traversability_network,
                               optimizer, 10, checkpoint_save_path)
