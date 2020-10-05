import numpy as np 

def quantize(z, n=11):
    bins = np.linspace(-4, 4, n)
    indxs = np.digitize(z, bins, right=False)
    return bins[indxs]


if __name__ == "__main__":
    x = np.array([-1, -0.1, 0.4, 0.1])
    print(quantize(x))