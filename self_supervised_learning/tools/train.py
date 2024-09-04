import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

from lesta_api.model import MLPClassifier
from lesta_api.dataloader import TraversabilityDataset
from lesta_api.loss import RiskSensitiveCELoss, EntropyRegularization

import os
from tqdm import tqdm
import yaml
import pandas as pd


def train_model(net, dataloader_dict, loss_fn, optimizer, num_epochs):

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
                    loss = loss_fn(outputs, labels)
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

    if len(unlabeled_dataset) == 0:
        print('No unlabeled samples to pseudo-label.')
        return None, None

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
    remaining_unlabeled_dataset = TraversabilityDataset(
        features=unlabeled_dataset.features[torch.where(unconfident_mask)[0]],
        labels=unlabeled_dataset.features[torch.where(unconfident_mask)[0]])

    if len(pseudo_labeled_data) > 0:
        pseudo_labeled_data = torch.cat(pseudo_labeled_data)
        pseudo_labels = torch.cat(pseudo_labels)
        pseudo_labeled_dataset = TraversabilityDataset(
            features=pseudo_labeled_data,
            labels=pseudo_labels)

        print(f'Found {len(pseudo_labeled_dataset)} confident predictions.')
        return pseudo_labeled_dataset, remaining_unlabeled_dataset
    else:
        print('No confident predictions found.')
        return None, None


def self_training(net, dataloader_dict, loss_fn, optimizer, num_epochs,
                  confidence_threshold=0.9, max_iter=10):

    for iteration in range(max_iter):
        print(
            f'\n\033[32mSelf-training iteration [{iteration + 1}/{max_iter}]\033[0m')

        # Step 1: model training
        print('-')
        print('\033[33m[1] Training from self-supervised labels...\033[0m\n')
        train_model(net,
                    dataloader_dict,
                    loss_fn,
                    optimizer,
                    num_epochs)

        # Step 2: pseudo-labeling
        print('-')
        print(
            f'\033[33m[2] Pseudo-labeling at round {iteration + 1}...\033[0m')
        pseudo_labeled_dataset, remaining_dataset = pseudo_labeling(
            net,
            dataloader_dict['unlabeled'].dataset,
            confidence_threshold)

        if pseudo_labeled_dataset is None:
            print('No more pseudo-labels found. Ending self-training...')
            break

        if len(remaining_dataset) == 0:
            print('No unlabeled samples remaining. Ending self-training...')
            break

        # Step 3: Add pseudo-labeled data to the labeled dataset
        dataloader_dict['train'].dataset.append(pseudo_labeled_dataset.features,
                                                pseudo_labeled_dataset.labels)

        # Step 4: Remove pseudo-labeled data from the unlabeled dataset
        dataloader_dict['unlabeled'] = DataLoader(
            remaining_dataset,
            batch_size=dataloader_dict['unlabeled'].batch_size,
            shuffle=False)

        print(f"Added {len(pseudo_labeled_dataset)} new labeled samples.")
        print(f'New training samples: {len(dataloader_dict['train'].dataset)}')
        print(f'Remaining unlabeled samples: {
              len(dataloader_dict["unlabeled"].dataset)}')

    # Save pseudo-labeled dataset for visualization
    pseudo_labeled_path = os.path.join(
        project_path, "data/pseudo_labeled_dataset.csv")

    dataloader_dict['train'].dataset.to_csv(pseudo_labeled_path)
    print(f'Pseudo-labels saved to {pseudo_labeled_path}')

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

    # Load config
    project_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_path, 'config',
                               'training_params.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    # Read self-training parameters
    self_training_iter = config['self_training_iterations']
    confidence_threshold = config['pseudo_labeling_threshold']
    batch_size = config['batch_size']
    num_epochs = config['num_epochs']
    lr = config['learning_rate']

    # Train-test split parameters
    test_ratio = config['test_ratio']
    random_seed = config['random_seed']

    # Load dataset
    labeled_dataset = TraversabilityDataset(
        csv_file=os.path.join(project_path, config['labeled_data_root']))

    unlabeled_dataset = TraversabilityDataset(
        csv_file=os.path.join(project_path, config['unlabeled_data_root']))

    train_dataset, val_dataset = labeled_dataset.random_split(test_ratio=test_ratio,
                                                              seed=random_seed)
    # Save dataloaders in a dictionary format
    dataloader_dict = {
        'train': DataLoader(train_dataset, batch_size=batch_size, shuffle=True),
        'val': DataLoader(val_dataset, batch_size=batch_size, shuffle=False),
        "unlabeled": DataLoader(unlabeled_dataset, batch_size=len(unlabeled_dataset), shuffle=False)
    }
    print(f'Training samples: {len(train_dataset)}')
    print(f'Validation samples: {len(val_dataset)}')
    print(f'Unlabeled samples: {len(unlabeled_dataset)}')

    # Initialize model
    feature_dim = labeled_dataset.features.shape[1]
    traversability_network = MLPClassifier(input_dim=feature_dim)
    loss_fn = nn.BCELoss()
    loss_fn = RiskSensitiveCELoss(train_dataset)
    optimizer = optim.Adam(traversability_network.parameters(), lr=lr)

    # # train
    # try:
    #     self_training(traversability_network, dataloader_dict,
    #                   loss_fn=loss_fn,
    #                   optimizer=optimizer,
    #                   num_epochs=num_epochs,
    #                   confidence_threshold=confidence_threshold,
    #                   max_iter=self_training_iter)

    #     print('\033[32m\nTraining completed. Saving model for LibTorch...\033[0m')

    #     checkpoint_path = os.path.join(project_path, config['checkpoint_root'])
    #     save_for_libtorch(traversability_network, checkpoint_path)

    # except KeyboardInterrupt:
    #     print('\033[32m\nTraining interrupted. Saving model for LibTorch...\033[0m')

    #     checkpoint_path = os.path.join(project_path, config['checkpoint_root'])
    #     save_for_libtorch(traversability_network, checkpoint_path)
