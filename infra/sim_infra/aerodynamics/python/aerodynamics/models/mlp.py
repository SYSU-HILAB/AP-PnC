"""
MLP model definition using Flax.

4-layer MLP for aerodynamic coefficient prediction.
"""

from collections.abc import Callable

import flax.linen as nn
import jax.numpy as jnp


class Snake(nn.Module):
    frequency: float = 10.0  # 频率超参数，类似 SIREN 的 omega

    @nn.compact
    def __call__(self, x):
        # 公式: x + (1/a) * sin^2(ax)
        # 既保留了 sin 的周期性拟合能力，又利用 x 项保持了趋势的平滑
        return x + (1.0 / self.frequency) * jnp.square(jnp.sin(self.frequency * x))

class MLP4Layer(nn.Module):
    """
    4-layer MLP for aerodynamics coefficient prediction.

    Architecture:
        Input (2) -> Dense -> Hidden -> Dense -> Hidden -> Dense -> Hidden -> Dense -> Output (2)

    Input features: (cos_alpha, sin_alpha)
    Output targets: (cx, cz)
    """

    input_dim: int = 2  # (cos_alpha, sin_alpha)
    output_dim: int = 2  # (cx, cz)
    hidden_dim: int = 256
    activation: Callable = nn.swish

    @nn.compact
    def __call__(self, x: jnp.ndarray) -> jnp.ndarray:
        """
        Forward pass.

        Args:
            x: Input features of shape (..., 2)

        Returns:
            Output coefficients of shape (..., 2)
        """
        # Layer 1: input -> hidden
        x = nn.Dense(self.hidden_dim)(x)
        x = self.activation(x)

        shortcut = x
        # Layer 2: hidden -> hidden
        x = nn.Dense(self.hidden_dim)(x)
        x = Snake(frequency=2.0)(x)
        x = nn.Dense(self.hidden_dim)(x)
        x = self.activation(x) + shortcut


        shortcut = x
        # Layer 3: hidden -> hidden
        x = nn.Dense(self.hidden_dim)(x)
        x = Snake(frequency=4.0)(x)
        x = nn.Dense(self.hidden_dim)(x)
        x = self.activation(x) + shortcut


        x = nn.Dense(self.output_dim)(x)   # Skip connection from input to output
        return x
