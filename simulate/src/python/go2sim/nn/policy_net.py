import torch
import torch.nn as nn
import numpy as np

from pathlib import Path


class MLP(nn.Module):
    def __init__(self, npz_path: Path, activation_fn=nn.SiLU()) -> None:
        super(MLP, self).__init__()
        data = np.load(npz_path)
        n_layers = int(data["n_layers"])

        self.mean = torch.tensor(data["mean"], dtype=torch.float32)
        self.std = torch.tensor(data["std"], dtype=torch.float32)
        self.activation = activation_fn
        self.layers = nn.ModuleList()

        for i in range(n_layers):
            w = data[f"w{i}"]
            b = data[f"b{i}"]
            layer = nn.Linear(w.shape[0], w.shape[1])
            layer.weight.data = torch.tensor(w.T, dtype=torch.float32)
            layer.bias.data = torch.tensor(b, dtype=torch.float32)
            self.layers.append(layer)

    @torch.no_grad()
    def forward(self, obs: np.ndarray) -> np.ndarray:
        x = torch.tensor(obs, dtype=torch.float32).reshape(1, -1)
        x = (x - self.mean) / self.std

        for layer in self.layers[:-1]:
            x = self.activation(layer(x))

        x = self.layers[-1](x)
        loc, _ = torch.chunk(x, 2, dim=-1)
        
        return torch.tanh(loc).squeeze(0).numpy()
