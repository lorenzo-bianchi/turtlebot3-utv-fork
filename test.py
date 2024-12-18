import numpy as np

def mySVD(a, b, c, d):
    a2 = a * a
    b2 = b * b
    c2 = c * c
    d2 = d * d

    term1 = a2 - 2 * a * d + b2 + 2 * b * c + c2 + d2
    term2 = a2 + 2 * a * d + b2 - 2 * b * c + c2 + d2
    term3 = a2 - b2 + c2 - d2
    term4 = a2 + b2 - c2 - d2
    term5 = 2*a*c + 2*b*d
    term6 = 2*a*b + 2*c*d

    # Radice dei termini
    print(term1, term2)
    sqrt_term = np.sqrt(term1 * term2)

    # Evitare divisioni per zero
    epsilon = 1e-10  # Valore piccolo per stabilità numerica

    # Calcolo di V
    v11 = 1
    v12 = term6 / (sqrt_term - (term3 + epsilon))
    v21 = -(sqrt_term + term3) / (term6 + epsilon)
    v22 = 1
    V = np.array([[v11, v12], [v21, v22]]) / np.sqrt(v11 * v22 - v12 * v21)

    # Calcolo di U
    u11 = 1
    u12 = term5 / (sqrt_term - (term4 + epsilon))
    u21 = -(sqrt_term + term4) / (term5 + epsilon)
    u22 = 1
    U = np.array([[u11, u12], [u21, u22]]) / np.sqrt(u11 * u22 - u12 * u21)

    return U, V.T
    return -U.T, V

if __name__ == '__main__':
    from timeit import timeit

    a, b, c, d = 3, 2, 3, 4

    U, V = mySVD(a, b, c, d)
    print(U @ V)

    total_time = timeit(lambda: mySVD(a, b, c, d), number=1)
    print(f"Total time: {total_time} s")

    total_time_svd = timeit(lambda: np.linalg.svd(np.array([[a, b], [c, d]])), number=1)
    print(f"Total time SVD: {total_time_svd} s")

    U, _, Vt = np.linalg.svd(np.array([[a, b], [c, d]]))
    print(U @ Vt)