"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: utils/pytorch/machine.py
"""
import torch


def device_setup():
    torch.backends.cudnn.benchmark = True


def get_device():
    device_setup()
    return torch.device(
        'cuda') if torch.cuda.is_available() else torch.device('cpu')


def to_device(data, device):
    """Recursively moves data and all PyTorch modules/tensors within it to the specified device.

    Args:
        data: Input data (can be tensor, module, list, dict, or custom object with PyTorch members).
        device: The target device ('cuda' or 'cpu').

    Returns:
        Data moved to the specified device.
    """
    if isinstance(data, torch.Tensor) or isinstance(data, torch.nn.Module):
        return data.to(device)
    elif isinstance(data, list):
        return [to_device(item, device) for item in data]
    elif isinstance(data, dict):
        return {key: to_device(value, device) for key, value in data.items()}
    elif hasattr(data, '__dict__'):
        for attr_name, attr_value in data.__dict__.items():
            if isinstance(attr_value, (torch.Tensor, torch.nn.Module, list, dict)):
                setattr(data, attr_name, to_device(attr_value, device))
        return data
    else:
        return data


if __name__ == "__main__":

    print(get_device())
    # cudnn benchmark
    print(torch.backends.cudnn.benchmark)
