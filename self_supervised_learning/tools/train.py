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
        print()
        print('-----------')
        print(f'Epoch [{epoch+1}/{num_epochs}]')

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
                    preds = (outputs > 0.5).float()

                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                    epoch_loss += loss.item() * inputs.size(0)
                    epoch_corrects += torch.sum(preds == labels.data)

            epoch_loss = epoch_loss / \
                len(dataloader_dict[phase].dataset)
            epoch_acc = epoch_corrects.double(
            ) / len(dataloader_dict[phase].dataset)

            print(f'{phase} Loss: {
                epoch_loss:.4f} Acc: {epoch_acc:.4f}')


def pseudo_labeling(net, unlabeled_dataloader, threshold):

    net.eval()
    pseudo_labeled_data = []
    pseudo_labels = []

    with torch.no_grad():
        for inputs, _ in unlabeled_dataloader:
            outputs = net(inputs)
            # get rid of extra dimension, outputs shape will be [batch_size]
            predictive_confidence = outputs.squeeze()

            confident_mask = predictive_confidence > threshold
            confident_inputs = inputs[confident_mask]
            confident_preds = predictive_confidence[confident_mask]

            if len(confident_inputs) > 0:
                pseudo_labeled_data.append(confident_inputs)
                pseudo_labels.append(torch.ones_like(confident_preds) if confident_mask.sum(
                ) > 0 else torch.zeros_like(confident_preds))

            confident_mask_negatives = predictive_confidence < (1 - threshold)
            confident_inputs_negatives = inputs[confident_mask_negatives]
            confident_preds_negatives = predictive_confidence[confident_mask_negatives]

            if len(confident_inputs_negatives) > 0:
                pseudo_labeled_data.append(confident_inputs_negatives)
                pseudo_labels.append(torch.zeros_like(confident_preds_negatives) if confident_mask_negatives.sum() > 0
                                     else torch.ones_like(confident_preds_negatives))

    if len(pseudo_labeled_data) > 0:
        pseudo_labeled_data = torch.cat(pseudo_labeled_data)
        pseudo_labels = torch.cat(pseudo_labels)
        return pseudo_labeled_data, pseudo_labels
    else:
        return None, None


def self_training(net, dataloader_dict, criterion, optimizer,
                  threshold=0.9, max_iter=10):

    for iteration in range(max_iter):
        print(f'Iteration [{iteration + 1}/{max_iter}]')

        # step 1: training from self-supervised labels
        train_model(net, dataloader_dict,
                    criterion, optimizer, num_epochs=1)

        # step 2: pseudo-labeling
        pseudo_data, pseudo_labels = pseudo_labeling(
            net, dataloader_dict['unlabeled'], threshold)

        if pseudo_data is None or len(pseudo_data) == 0:
            print('No confident predictions found. Ending self-training...')
            break

        # step 3: Create a new dataset with pseudo-labeled data
        pseudo_dataloader = DataLoader(
            torch.utils.data.TensorDataset(pseudo_data, pseudo_labels),
            batch_size=dataloader_dict['train'].batch_size,
            shuffle=True)

        # step 4: Add pseudo-labeled data to the labeled dataset
        dataloader_dict['train'] = DataLoader(
            torch.utils.data.ConcatDataset(
                [dataloader_dict['train'].dataset, pseudo_dataloader.dataset]),
            batch_size=dataloader_dict['train'].batch_size,
            shuffle=True)

        print(f"Added {len(pseudo_data)} new labeled examples.")

    return net


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

    n_feature = labeled_dataset.features.shape[1]
    network = MLPClassifier(input_dim=n_feature)
    criterion = nn.BCELoss()
    optimizer = optim.Adam(network.parameters(), lr=lr)

    labeled_train_dataset, val_dataset = labeled_dataset.random_split(
        test_ratio=test_ratio, manual_seed=manual_seed)
    dataloader_dict = {
        'train': DataLoader(labeled_train_dataset, batch_size=batch_size, shuffle=True),
        'val': DataLoader(val_dataset, batch_size=batch_size, shuffle=False),
        "unlabeled": DataLoader(unlabeled_dataset, batch_size=batch_size, shuffle=True)
    }

    # train
    # train_model(network, dataloader_dict, criterion, optimizer, num_epochs)
    self_training(network, dataloader_dict, criterion, optimizer, threshold=0.9, max_iter=10)

    # save trained checkpoints in libtorch format
    network.eval()
    traced_model = torch.jit.script(network)
    traced_model.save(checkpoint_save_path)
    print(f'Model saved at {checkpoint_save_path}')
