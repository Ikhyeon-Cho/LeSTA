import torch
import torch.optim as optim
import torch.nn as nn
import yaml
from models.mlp import MLPClassifier
from training.train_model import train_model
from training.data_loader import preprocess_data

with open('config/traversability/training_params.yaml', 'r') as file:
    config = yaml.safe_load(file)

file_path = 'data/labeled_data.csv'
X_train_tensor, X_test_tensor, y_train_tensor, y_test_tensor = preprocess_data(file_path, config['test_size'], config['random_seed'])

input_dim = X_train_tensor.shape[1]
model = MLPClassifier(input_dim)

criterion = nn.BCELoss()
optimizer = optim.Adam(model.parameters(), lr=config['learning_rate'])

train_model(model, criterion, optimizer, X_train_tensor, y_train_tensor, config['num_epochs'])

torch.save(model.state_dict(), 'saved_models/mlp_classifier.pth')

# Testing the model
model.eval()
with torch.no_grad():
    test_outputs = model(X_test_tensor)
    predictions = (test_outputs > 0.5).float()
    accuracy = (predictions.eq(y_test_tensor).sum() / y_test_tensor.shape[0]).item()
    print(f'Accuracy: {accuracy:.4f}')
