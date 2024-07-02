import torch
import torch.optim as optim
import torch.nn as nn
import yaml
from traversability_learning_api.models.mlp import MLPClassifier
from traversability_learning_api.dataloader import preprocess_data


def train_model(model, criterion, optimizer, X_train_tensor, y_train_tensor, num_epochs):
    for epoch in range(num_epochs):
        model.train()
        optimizer.zero_grad()
        outputs = model(X_train_tensor)
        loss = criterion(outputs, y_train_tensor)
        loss.backward()
        optimizer.step()
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')


with open('/home/ikhyeon/ros/test_ws/src/traversability_learning/self_supervised_learning/config/training_params.yaml', 'r') as file:
    config = yaml.safe_load(file)

file_path = '/home/ikhyeon/ros/test_ws/src/traversability_learning/self_supervised_learning/data/labeled_data.csv'
X_train_tensor, X_test_tensor, y_train_tensor, y_test_tensor = preprocess_data(
    file_path, config['test_size'], config['random_seed'])

input_dim = X_train_tensor.shape[1]
model = MLPClassifier(input_dim)

criterion = nn.BCELoss()
optimizer = optim.Adam(model.parameters(), lr=config['learning_rate'])


train_model(model, criterion, optimizer, X_train_tensor,
            y_train_tensor, config['num_epochs'])

# Save libtorch checkpoints
# torch.save(model.state_dict(), 'checkpoints/mlp_classifier.pth')
model.eval()

traced_model = torch.jit.script(model)
traced_model.save('/home/ikhyeon/ros/test_ws/src/traversability_learning/self_supervised_learning/checkpoints/mlp_classifier.pth')
# torch.save(traced_model.state_dict(), '/home/ikhyeon/ros/test_ws/src/traversability_learning/self_supervised_learning/checkpoints/mlp_classifier.pth')

# Testing the model
with torch.no_grad():
    test_outputs = model(X_test_tensor)
    predictions = (test_outputs > 0.5).float()
    accuracy = (predictions.eq(y_test_tensor).sum() /
                y_test_tensor.shape[0]).item()
    print(f'Accuracy: {accuracy:.4f}')
