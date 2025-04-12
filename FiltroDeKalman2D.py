import numpy as np
import matplotlib.pyplot as plt

# Parámetros
dt = 0.1
total_time = 15
num_steps = int(total_time / dt)
Q = np.eye(4) * 0.1  # Matriz de covarianza del proceso
R = np.eye(2) * 5  # Matriz de covarianza de medición

# Matrices del sistema
A = np.array([
    [1, dt, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])

B = np.array([
    [0.5 * dt**2, 0],
    [dt, 0],
    [0, 0.5 * dt**2],
    [0, dt]
])

C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

# Control
u = np.array([1, 0.5]) # Aceleración en x y y

# Simulación del movimiento real
true_states = []
current_state = np.zeros(4)
true_states.append(current_state.copy())

for _ in range(num_steps):
    current_state = A @ current_state + B @ u
    true_states.append(current_state.copy())

# Mediciones ruidosas
measurements = []
std_dev = np.sqrt(R[0, 0])
for i in range(len(true_states)):
    if i % 30 == 0:  # Cada 3 segundos (30 pasos)
        x_real = true_states[i][0]
        y_real = true_states[i][2]
        z_x = x_real + np.random.normal(0, std_dev)
        z_y = y_real + np.random.normal(0, std_dev)
        measurements.append((z_x, z_y))
    else:
        measurements.append(None)

# Filtro de Kalman
mu = np.zeros(4)  # Estimación inicial
P = np.eye(4) * 0.1  # Covarianza inicial

estimates = [mu.copy()]
covariances = [P.copy()]

for step in range(1, num_steps + 1):
    # Predicción
    mu_pred = A @ mu + B @ u
    P_pred = A @ P @ A.T + Q

    # Actualización si hay medición
    if step % 30 == 0:
        z = np.array(measurements[step])
        S = C @ P_pred @ C.T + R
        K = P_pred @ C.T @ np.linalg.inv(S)
        mu = mu_pred + K @ (z - C @ mu_pred)
        P = (np.eye(4) - K @ C) @ P_pred
    else:
        mu = mu_pred
        P = P_pred

    estimates.append(mu.copy())
    covariances.append(P.copy())

estimates = np.array(estimates)
covariances = np.array(covariances)
true_states = np.array(true_states)

# Gráficos
time = np.arange(0, total_time + dt, dt)

# Trayectoria
plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 2], 'k-', label='Real')
plt.plot(estimates[:, 0], estimates[:, 2], 'b--', label='Estimada')
std_x = np.sqrt(covariances[:, 0, 0])
std_y = np.sqrt(covariances[:, 2, 2])
plt.fill_between(true_states[:, 0], true_states[:, 2] - std_y, true_states[:, 2] + std_y, alpha=0.3)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trayectoria del Robot')
plt.legend()
plt.grid(True)

# Velocidades
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time, true_states[:, 1], 'k-', label='Real Vx')
plt.plot(time, estimates[:, 1], 'b--', label='Estimada Vx')
std_vx = np.sqrt(covariances[:, 1, 1])
plt.fill_between(time, estimates[:, 1] - std_vx, estimates[:, 1] + std_vx, alpha=0.3)
plt.ylabel('Vx (m/s)')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(time, true_states[:, 3], 'k-', label='Real Vy')
plt.plot(time, estimates[:, 3], 'b--', label='Estimada Vy')
std_vy = np.sqrt(covariances[:, 3, 3])
plt.fill_between(time, estimates[:, 3] - std_vy, estimates[:, 3] + std_vy, alpha=0.3)
plt.xlabel('Tiempo (s)')
plt.ylabel('Vy (m/s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()