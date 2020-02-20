import numpy as np

def allWithin(a,b,threshold=0.0001) -> bool:
    return np.all(np.abs(np.array(a) - np.array(b)) < threshold)

