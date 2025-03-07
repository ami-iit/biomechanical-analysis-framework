import numpy as np

def computeVectorNormalization(vector, N):
    """
    computeVectorNormalization normalizes a given vector.
    """
    normalizedVect = np.zeros((len(vector), N))
    for i in range(len(vector)):
        Y = np.linspace(1, len(vector[i]), N)
        normalizedVect[i, :] = np.interp(Y, np.arange(1, len(vector[i]) + 1), vector[i])  
    
    return normalizedVect