import numpy as np


a = np.array([[1,2,3],[4,5,6]])

a[(1, 1), (0, 1)] = [10, 20]
print(a)