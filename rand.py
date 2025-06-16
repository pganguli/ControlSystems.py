import numpy as np

def generate_normal_random_numbers(mu: float, sigma: float, num_samples: int, seed: int):
    np.random.seed(seed)
    s = np.random.normal(loc=mu, scale=sigma, size=num_samples)
    return s 