import numpy as np
import matplotlib.pyplot as plt

def kalman_filter():
    dt = 0.1
    a = 1.0  # m/s²
    z_measurements = {3: 0.4673, 6: 1.0198, 9: 1.5569}
    
    # Matrices del modelo
    A = np.array([[1, dt], [0, 1]])
    B = np.array([[0.005], [0.1]])
    C = np.array([[1, 0]])
    Q = np.array([[0.01, 0], [0, 0.01]])
    R = np.array([[0.001]])
    
    # Estado inicial
    mu = np.array([[0.0], [0.0]])
    Sigma = np.array([[0.1, 0.0], [0.0, 0.1]])
    
    # Almacenar resultados
    time_steps = np.arange(0, 11)
    positions = np.zeros(11)
    position_std = np.zeros(11)
    
    for k in range(0, 11):
        if k > 0:
            # Predicción
            mu_hat = A @ mu + B * a
            Sigma_hat = A @ Sigma @ A.T + Q
            
            # Corrección si hay medición
            if k in z_measurements:
                K = Sigma_hat @ C.T @ np.linalg.inv(C @ Sigma_hat @ C.T + R)
                mu = mu_hat + K * (z_measurements[k] - C @ mu_hat)
                Sigma = Sigma_hat - K @ C @ Sigma_hat
            else:
                mu = mu_hat
                Sigma = Sigma_hat
        
        positions[k] = mu[0][0]
        position_std[k] = np.sqrt(Sigma[0, 0])
    
    return time_steps, positions, position_std

# Ejecutar el filtro
time, pos, pos_std = kalman_filter()

# Crear gráfica
plt.figure(figsize=(12, 6))

# Trayectoria estimada y región de incertidumbre
plt.plot(time * 0.1, pos, 'b-', marker='o', label='Estimación de posición')
plt.fill_between(time * 0.1, 
                 pos - 2 * pos_std, 
                 pos + 2 * pos_std, 
                 color='blue', alpha=0.2, label='2σ de incertidumbre')

# Añadir valores numéricos
for k in [0,1,2, 3,4,5, 6,7,8,9, 10]:
    t = k * 0.1
    plt.annotate(f'{pos[k]:.3f}m ± {2*pos_std[k]:.3f}m', 
                 (t, pos[k]), 
                 textcoords="offset points", 
                 xytext=(0,10 if k%2==0 else -15), 
                 ha='center',
                 fontsize=9,
                 arrowprops=dict(arrowstyle="->", color='gray', alpha=0.5))

# Mediciones reales
plt.scatter([0.3, 0.6, 0.9], [0.4673, 1.0198, 1.5569], 
            color='red', zorder=3, label='Mediciones (z)')

# Personalización
plt.xlabel('Tiempo (s)', fontsize=12)
plt.ylabel('Posición (m)', fontsize=12)
plt.title('Filtro de Kalman: Estimación de Posición con Valores Numéricos', fontsize=14)
plt.xticks(np.arange(0, 1.1, 0.1))
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.tight_layout()
plt.show()