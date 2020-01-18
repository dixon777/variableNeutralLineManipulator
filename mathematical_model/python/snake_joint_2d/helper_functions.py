import numpy as np
import functools
            
def lenRecursive(r):
    if not isinstance(r, list):
        return 1
    return functools.reduce(lambda s,i: s+lenRecursive(i), r)
    

def crossMul2d(a:float,b:float) -> float:
    return a[0]*b[1] - a[1]*b[0]

def allWithin(a,b,threshold=0.0001) -> bool:
    return np.all(np.abs(np.array(a) - np.array(b)) < threshold)


def binarySearchCompute(func, lowerBound, upperBound, threshold=0.0001, **kwargs):
    result = (threshold + 1,)
    while abs(result[0]) > threshold:
        guess = (lowerBound+upperBound)/2
        result = func(guess, **kwargs)
        if result[0] > 0:
            lowerBound = guess
        else:
            upperBound = guess
    return result

def binarySearchIterator(func, lowerBound, upperBound, threshold=0.0001, **kwargs):
    result = (threshold + 1,)
    while abs(result[0]) > threshold:
        guess = (lowerBound+upperBound)/2
        result = func(guess, **kwargs)
        yield result[1]
        if result[0] > 0:
            lowerBound = guess
        else:
            upperBound = guess