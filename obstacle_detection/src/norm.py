import numpy as np


def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)


if __name__ == '__main__':
    A = np.random.randn(3, 2)
    print A.shape
    print normalized(A, 0)
    print normalized(A, 1)
