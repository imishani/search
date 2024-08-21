import importlib

import torch
import numpy as np
import argparse
from scripts.models import *

def convert_pth_to_cpp(pth_file, config, example, cpp_file):
    model: VIN = VIN(config)
    model.load_state_dict(torch.load(pth_file))
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to(device)

    model.eval()
    traced_script_module = torch.jit.trace(model, example)
    traced_script_module.save(cpp_file)

    return traced_script_module

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert PyTorch model to C++')
    parser.add_argument('--pth_file', type=str, help='Path to the .pth file.')
    parser.add_argument('--config', type=str, help='Path to the config file.')
    parser.add_argument('--example', type=str, help='Example input to the model. Must be a torch.Tensor.')
    parser.add_argument('--cpp_file', type=str, help='Path to the .cpp file.')
    args = parser.parse_args()

    pth_file = args.pth_file
    import pickle
    with open(args.config, 'rb') as f:
        config = pickle.load(f)  # import the config dict from the config file
    # import the config dict from the config file
    if args.example.endswith('.pt') or args.example.endswith('.pth'):
        examples = torch.load(args.example)
    elif args.example.endswith('.npy'):
        examples = torch.from_numpy(np.load(args.example))
    elif args.example.endswith('.pkl'):
        import pickle
        examples = []
        with open(args.example, 'rb') as f:
            example = pickle.load(f)
            if isinstance(example, tuple):
                for i in range(len(example)):
                    if isinstance(example[i], torch.Tensor):
                        examples.append(example[i])
                    elif isinstance(example[i], int):
                        examples.append(torch.tensor(example[i]))
                    else:
                        raise ValueError("Example must be a torch.Tensor, .npy, .pt, or .pkl file.")
            examples = tuple(examples)
    else:
        raise ValueError("Example must be a torch.Tensor, .npy, .pt, or .pkl file.")

    cpp_file = args.cpp_file

    convert_pth_to_cpp(pth_file, config, examples, cpp_file)
