import numpy as np
from scipy.linalg import block_diag


#calculate moment of intertia
r1 = 0.6
a1 = 0.02**2
m1 = r1 * a1 * 700

r2 = 0.1
a2 = np.pi * r2**2
h1 = 0.01
m2 = a2 * h1 * 7870

I1 = (1/3) * m1 * r1**2
I2 = 1/2 * m2 * r2**2

g = -9.81



a = m1 * r1**2 + 4 * m2 * r1**2 + I1
b = (m1 + I2) * g * r1


# Toestands- en invoermatrix
A = np.array([[0, 1, 0, 0],
              [-b/(a+I2), 0, 0, -I2 / (a+I2)],
              [0, 0, 0, 1],
              [0, 0, -1/I2, 0]])  # Vul in met A-matrix
B = np.array([[0],
              [0],
              [0],
              [1/I2]])  # Vul in met B-matrix

# Controleerbaarheidsmatrix berekenen
n = A.shape[0]  # Aantal toestanden
controllability_matrix = B
for i in range(1, n):
    controllability_matrix = np.hstack((controllability_matrix, np.linalg.matrix_power(A, i) @ B))

# Controleerbaarheidsmatrix berekenen
n = A.shape[0]  # Aantal toestanden
controllability_matrix = B
for i in range(1, n):
    controllability_matrix = np.hstack((controllability_matrix, np.linalg.matrix_power(A, i) @ B))

# Controleer de rang van de controleerbaarheidsmatrix
rank = np.linalg.matrix_rank(controllability_matrix)
if rank == n:
    print("Het systeem is volledig controleerbaar.")
else:
    print("Het systeem is niet volledig controleerbaar.")

# Eigenwaarden berekenen
eigenvalues, _ = np.linalg.eig(A)
print("Eigenwaarden van A:", eigenvalues)

# Stabiliseerbaarheid controleren
is_stabilizable = True
for eigval in eigenvalues:
    if eigval.real >= 0:  # Instabiele eigenwaarde
        controllability_submatrix = np.hstack((A - eigval.real * np.eye(A.shape[0]), B))
        if np.linalg.matrix_rank(controllability_submatrix) < A.shape[0]:
            is_stabilizable = False
            print(f"Instabiele eigenwaarde {eigval} is niet controleerbaar.")
            break

if is_stabilizable:
    print("Het systeem is stabiliseerbaar.")
else:
    print("Het systeem is niet stabiliseerbaar.")