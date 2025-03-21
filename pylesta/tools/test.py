import argparse
import torch
from utils.param import yaml
from utils.pytorch import machine
from lesta.api.model import TraversabilityNetwork


def main(args):
    CFG_PATH = args.config
    CHECKPOINT_PATH = args.checkpoint

    print(
        f'\033[32m========= Testing model: "{CHECKPOINT_PATH}" =========\033[0m')

    # Load configuration
    print(f'=> Loading config from: {CFG_PATH}')
    cfg = yaml.load(CFG_PATH)
    MODEL_CFG = cfg['MODEL']

    # Set device
    device = machine.get_device()
    if args.device == 'cpu':
        device = torch.device('cpu')
    print(f'=> Using device: {device}')

    # Load model
    print('=> Creating model architecture...')
    model = TraversabilityNetwork(cfg=MODEL_CFG)

    # Load checkpoint
    print(f'=> Loading checkpoint from: {CHECKPOINT_PATH}')
    if CHECKPOINT_PATH.endswith('.pt'):
        # Load TorchScript model
        model = torch.jit.load(CHECKPOINT_PATH, map_location=device)
    else:
        # Load regular PyTorch checkpoint
        checkpoint = torch.load(CHECKPOINT_PATH, map_location=device)
        if 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            model.load_state_dict(checkpoint)
        model = model.to(device)

    # Set model to evaluation mode
    model.eval()

    # Define test samples
    print('=> Using manually defined test data')
    input_dim = MODEL_CFG['input_dim']

    # Define your samples manually - adjust values and number of samples as needed
    manual_samples = [
        [0.012542605, 1.5994414, 0.0013297237,
            5.4580851e-05],  # Sample 1: traversable
        # Sample 2: non-traversable
        [0.73471355, 65.826698, 0.065527156, 0.044856552],
        [0.010115743, 1.2940117, 0.0012869993,
            5.113686e-05],   # Sample 3: traversable
        [0.9, 10, 0.9, 0.1],                                    # Sample 4
        [0.05, 0.8, 0.05, 0.01]                                 # Sample 5
    ]

    # Convert to tensor and move to device
    features = torch.tensor(manual_samples, dtype=torch.float32).to(device)
    num_samples = features.shape[0]
    print(
        f'Using {num_samples} manually defined samples with {input_dim} features')

    # Display sample features
    print("Sample features:")
    print(features)

    # Perform inference
    print('=> Running inference...')
    with torch.no_grad():
        outputs = model(features)
        probabilities = torch.sigmoid(outputs).squeeze()
        predictions = (probabilities >= 0.5).int()

    # Display results
    print('\n=> Results:')
    for i in range(num_samples):
        print(f"Sample {i+1}:")
        print(f"  Features: {features[i].cpu().numpy()}")
        print(f"  Probability: {probabilities[i].item():.4f}")
        print(
            f"  Prediction: {'Traversable' if predictions[i].item() == 1 else 'Non-traversable'}")

    print(f'\033[32m====== Testing completed ======\033[0m')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str,
                        default='lesta_training/configs/lesta.yaml')
    parser.add_argument('--checkpoint', type=str,
                        default='checkpoints/epoch_best.pt')
    parser.add_argument('--device', type=str,
                        default='auto', choices=['auto', 'cpu', 'cuda'])

    args = parser.parse_args()
    main(args)
