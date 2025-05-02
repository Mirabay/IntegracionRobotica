import numpy as np
import math

# Parámetros constantes
V_k = 2.0  # m/s
omega_k = 1.0  # rad/s
dt = 0.1  # s
steps = 10  # número de pasos a simular

# Matriz de covarianza del ruido del proceso
Q_k = np.array([
    [0.5, 0.01, 0.01],
    [0.01, 0.5, 0.01],
    [0.01, 0.01, 0.2]
])

# Estado inicial
mu = np.array([0.0, 0.0, 0.0])  # [s_x, s_y, s_theta]
Sigma = np.zeros((3, 3))  # Matriz de covarianza inicial

# Función para el modelo de movimiento
def motion_model(state, V, omega, dt):
    x, y, theta = state
    new_x = x + V * dt * math.cos(theta)
    new_y = y + V * dt * math.sin(theta)
    new_theta = theta + omega * dt
    return np.array([new_x, new_y, new_theta])

# Función para calcular la matriz Jacobiana G
def compute_jacobian(state, V, dt):
    theta = state[2]
    G = np.array([
        [1, 0, -V * dt * math.sin(theta)],
        [0, 1, V * dt * math.cos(theta)],
        [0, 0, 1]
    ])
    return G

# Función para formatear matrices
def format_matrix(mat):
    return np.array2string(mat, precision=3, suppress_small=True)

# Imprimir estado inicial
print(f"\nInicial (k=0):")
print(f"μ_0 = \n{format_matrix(mu)}")
print(f"Σ_0 = \n{format_matrix(Sigma)}\n")

# Bucle principal del EKF
for step in range(1, steps + 1):
    # Calcular componentes
    G = compute_jacobian(mu, V_k, dt)
    mu_pred = motion_model(mu, V_k, omega_k, dt)
    intermediate_cov = G @ Sigma @ G.T
    Sigma_pred = intermediate_cov + Q_k
    
    # Imprimir detalles del paso
    print(f"\n{'='*40}")
    print(f"Hercasian {step}")
    
    # ① Predicción del estado
    print(f"\n① μ_{step} = ")
    print(format_matrix(mu_pred.reshape(3, 1)))
    
    # ② Jacobiana
    print(f"\n② G_{step} = ")
    print(format_matrix(G))
    
    # ③ Covarianza
    print(f"\n③ Σ_{step} = G * Σ_{step-1} * G^T + Q_k")
    # print("G * Σ_prev * G^T:\n", format_matrix(intermediate_cov))
    # print("\n+ Q_k:\n", format_matrix(Q_k))
    print("\n= Σ_pred:\n", format_matrix(Sigma_pred))
    
    # Actualizar para siguiente paso
    mu = mu_pred
    Sigma = Sigma_pred

print("\n" + "="*40)