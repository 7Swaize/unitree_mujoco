import numpy as np

from numba import njit
from numpy.typing import NDArray
from pathlib import Path


@njit(fastmath=True)
def mlp_forward_jit(obs, mean, std, W0, W1, W2, W3, B0, B1, B2, B3) -> NDArray[np.float32]:
    one = np.float32(1.0)
    x = (obs - mean) / std
    x = x @ W0 + B0
    x = x / (one + np.exp(-x)) # SiLU chains
    x = x @ W1 + B1
    x = x / (one + np.exp(-x))
    x = x @ W2 + B2
    x = x / (one + np.exp(-x))
    x = x @ W3 + B3
    loc = x[: x.shape[0] // 2]
    return np.tanh(loc)


class MLP:
    def __init__(self, npz_path: Path) -> None:
        data = np.load(npz_path)
        n_layers = int(data["n_layers"])
        self._mean = data["mean"].astype(np.float32)
        self._std = data["std"].astype(np.float32)

        Ws = [data[f"w{i}"].astype(np.float32) for i in range(n_layers)]
        Bs = [data[f"b{i}"].astype(np.float32) for i in range(n_layers)]

        self._W0 = Ws[0]
        self._W1 = Ws[1]
        self._W2 = Ws[2]
        self._W3 = Ws[3]
        self._B0 = Bs[0]
        self._B1 = Bs[1]
        self._B2 = Bs[2]
        self._B3 = Bs[3]

    def __call__(self, obs: NDArray[np.float32]) -> NDArray[np.float32]:
        return mlp_forward_jit(
            obs,
            self._mean,
            self._std,
            self._W0,
            self._W1,
            self._W2,
            self._W3,
            self._B0,
            self._B1,
            self._B2,
            self._B3,
        )