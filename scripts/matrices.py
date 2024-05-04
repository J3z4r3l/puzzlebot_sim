def multiply_matrices(A, B):
    if len(A[0]) != len(B):
        raise ValueError("El numero de columnas de la matriz A debe ser igual al numero de filas de la matriz B")
    
    result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]
    
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    
    return result

def transpose_matrix(matrix):
    rows = len(matrix)
    cols = len(matrix[0])

    transposed = [[0 for _ in range(rows)] for _ in range(cols)]

    for i in range(rows):
        for j in range(cols):
            transposed[j][i] = matrix[i][j]

    return transposed
def add_matrices(A, B):
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("Las matrices deben tener la misma dimension para poder sumarse")
    
    result = [[0 for _ in range(len(A[0]))] for _ in range(len(A))]
    
    for i in range(len(A)):
        for j in range(len(A[0])):
            result[i][j] = A[i][j] + B[i][j]
    
    return result
def multiply_3x2_by_2x2(A, B):
    if len(A[0]) != len(B):
        raise ValueError("B")

    result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]

    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]

    return result

def get_covariance(H,Covariance,Qk):
        #A*B*A-1+C
        A_B=multiply_matrices(H,Covariance)
        H_t=transpose_matrix(H)
        product=multiply_matrices(A_B,H_t)
        Covariance=add_matrices(product,Qk)
        return Covariance

# Ejemplo de matrices
A = [[1, 0, 0],
     [0, 1, 0.1],
     [0, 0, 1]]

B = [[0, 0, 0],
     [0, 0, 0],
     [0, 0, 0]]

C = [[1, 0, 0],
     [0, 1, 0],
     [0, 0.1, 1]]

D= [[0.5, 0.01, 0.01],
     [0.01, 0.5, 0.01],
     [0.01, 0.01, 0.2]]
A_2 = [[1, 0, -0.01],
     [0, 1, 0.1],
     [0, 0, 1]]


# Multiplicar las matrices

cov=get_covariance(A,B,D)
cov2=get_covariance(A_2,cov,D)
print(cov2)
print("traka")
print(cov)
#A*B*A-1

