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


if __name__ == '__main__':

    # self_supervised_learning directory
    project_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_path, 'config', 'training_params.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    csv_dataset_path = os.path.join(project_path, config['data_root'])
    checkpoint_save_path = os.path.join(
        project_path, config['checkpoint_root'])

    dataset = TraversabilityDataset(csv_file=csv_dataset_path)
    batch_size = config['batch_size']
    num_epochs = config['num_epochs']
    lr = config['learning_rate']
    test_ratio = config['test_ratio']
    manual_seed = config['random_seed']

    input_dim = dataset.features.shape[1]
    network = MLPClassifier(input_dim=input_dim)
    criterion = nn.BCELoss()
    optimizer = optim.Adam(network.parameters(), lr=lr)

    train_dataset, val_dataset = dataset.random_split(
        test_ratio=test_ratio, manual_seed=manual_seed)
    dataloader_dict = {
        'train': DataLoader(train_dataset, batch_size=batch_size, shuffle=True),
        'val': DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    }

    # train
    train_model(network, dataloader_dict, criterion, optimizer, num_epochs)

    # save trained checkpoints in libtorch format
    network.eval()
    traced_model = torch.jit.script(network)
    traced_model.save(checkpoint_save_path)
    print(f'Model saved at {checkpoint_save_path}')
