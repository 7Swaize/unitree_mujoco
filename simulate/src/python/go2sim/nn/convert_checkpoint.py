import sys
import pickle
import numpy as np

from pathlib import Path

DST_NPZ_PATH_ABS: Path = Path(Path(__file__).resolve().parent / "policies_npz").resolve()

def convert(src_pickle_path_abs: str) -> None:
    with open(src_pickle_path_abs, "rb") as f:
        params = pickle.load(f)

    mean = np.asarray(params[0].mean["state"], dtype=np.float32)
    std = np.asarray(params[0].std["state"], dtype=np.float32)
    param_dict = params[1]["params"] if len(params) == 3 else params[1].policy["params"]

    weights, biases = [], []
    for layer_name in param_dict:
        weights.append(np.asarray(param_dict[layer_name]["kernel"], dtype=np.float32))
        biases.append(np.asarray(param_dict[layer_name]["bias"], dtype=np.float32))

    DST_NPZ_PATH_ABS.mkdir(parents=True, exist_ok=True)
    out_path = DST_NPZ_PATH_ABS / f"{Path(src_pickle_path_abs).name}.npz"

    np.savez(
        out_path,
        mean=mean, std=std, n_layers=len(weights),
        **{f"w{i}": w for i, w in enumerate(weights)},
        **{f"b{i}": b for i, b in enumerate(biases)},
    )


if __name__ == "__main__":
    convert(sys.argv[1])