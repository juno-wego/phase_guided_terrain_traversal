import io
import pickle
import sys
import types

import numpy as np
import torch
import torch.nn as nn


def _install_pickle_compat_modules():
    """Provide minimal Brax/JAX symbols so legacy policy pickles can load."""
    if "numpy._core" not in sys.modules:
        sys.modules["numpy._core"] = np.core
    if "numpy._core.multiarray" not in sys.modules:
        sys.modules["numpy._core.multiarray"] = np.core.multiarray

    if "brax.training.acme.running_statistics" not in sys.modules:
        brax = sys.modules.setdefault("brax", types.ModuleType("brax"))
        training = sys.modules.setdefault("brax.training", types.ModuleType("brax.training"))
        acme = sys.modules.setdefault("brax.training.acme", types.ModuleType("brax.training.acme"))
        running = types.ModuleType("brax.training.acme.running_statistics")

        class RunningStatisticsState:
            def __new__(cls, *args, **kwargs):
                return object.__new__(cls)

        running.RunningStatisticsState = RunningStatisticsState
        sys.modules["brax.training.acme.running_statistics"] = running
        brax.training = training
        training.acme = acme
        acme.running_statistics = running

    if "jax._src.array" not in sys.modules:
        jax = sys.modules.setdefault("jax", types.ModuleType("jax"))
        jax_src = sys.modules.setdefault("jax._src", types.ModuleType("jax._src"))
        jax_array = types.ModuleType("jax._src.array")

        def _reconstruct_array(reconstruct_fn, reconstruct_args, array_state, extra_state):
            del reconstruct_fn, reconstruct_args, extra_state
            _, shape, dtype, fortran_order, raw_bytes = array_state
            array = np.frombuffer(raw_bytes, dtype=dtype)
            if shape:
                array = array.reshape(shape, order="F" if fortran_order else "C")
            return array.copy()

        jax_array._reconstruct_array = _reconstruct_array
        sys.modules["jax._src.array"] = jax_array
        jax._src = jax_src
        jax_src.array = jax_array

def get_params(policy_file: str):
    _install_pickle_compat_modules()
    with open(policy_file, 'rb') as f:
        params = pickle.load(f)
    if len(params)==3:
        mean=params[0].mean["state"]
        std=params[0].std["state"]
        param_dict = params[1]["params"]

        weights = []
        biases = []
        for layer_name in param_dict:
            weights.append(param_dict[layer_name]["kernel"])
            biases.append(param_dict[layer_name]["bias"])

        return mean,std,weights, biases
    else:
        mean=params[0].mean["state"]
        std=params[0].std["state"]
        param_dict = params[1].policy['params']
        weights = []
        biases = []
        # print(params[0].mean["state"])
        # print(param_dict["hidden_0"])
        for layer_name in param_dict:
            weights.append(param_dict[layer_name]["kernel"])
            biases.append(param_dict[layer_name]["bias"])

        return mean,std,weights, biases


class MLP(nn.Module):
    def __init__(self, weights, biases, activation_fn=nn.ReLU(),mean=None,std=None):
        super(MLP, self).__init__()
        self.mean = torch.tensor(np.asarray(mean), dtype=torch.float32, requires_grad=False)
        self.std = torch.tensor(np.asarray(std), dtype=torch.float32, requires_grad=False)

        self.layers = nn.ModuleList()
        self.activation_fn = activation_fn
        
        layer_sizes = [weights[0].shape[0]] 
        for w in weights:
            layer_sizes.append(w.shape[1])  

        
        for i in range(len(layer_sizes) - 1):
            self.layers.append(nn.Linear(layer_sizes[i], layer_sizes[i + 1]))

        
        self.load_params(weights, biases)

    def forward(self, x):
        x=(x-self.mean)/self.std
        for layer in self.layers[:-1]:  
            x = self.activation_fn(layer(x))
        x = self.layers[-1](x)  

        loc, _ = torch.chunk(x, 2, dim=-1)  

        return torch.tanh(loc)
    def load_params(self, weights, biases):
        for i, (weight, bias) in enumerate(zip(weights, biases)):
            weight_tensor = torch.tensor(np.asarray(weight), dtype=torch.float32).T
            bias_tensor = torch.tensor(np.asarray(bias), dtype=torch.float32)
            
            self.layers[i].weight.data = weight_tensor
            self.layers[i].bias.data = bias_tensor

def policy_net(policy_file: str,activation_fn=nn.SiLU()):
    mean,std,weights, biases = get_params(policy_file)
    model = MLP(weights, 
                biases, 
                activation_fn,
                mean,
                std)
    return model
