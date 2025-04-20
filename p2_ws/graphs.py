import matplotlib.pyplot as plt
import csv

# Inicializamos listas para almacenar los datos
times = []
x_true, y_true, theta_true = [], [], []
x_est, y_est, theta_est = [], [], []

# Leer datos desde el archivo CSV
with open('kalman_output.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row['time']))
        x_true.append(float(row['x_true']))
        y_true.append(float(row['y_true']))
        theta_true.append(float(row['theta_true']))
        x_est.append(float(row['x_est']))
        y_est.append(float(row['y_est']))
        theta_est.append(float(row['theta_est']))

# === Gráfica 1: Trayectoria XY ===
plt.figure(figsize=(10, 6))
plt.plot(x_true, y_true, 'r--', label='Odom (True)')
plt.plot(x_est, y_est, 'b-', label='Kalman Estimate')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trayectoria XY')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()

# === Gráfica 2: Posición X vs Tiempo ===
plt.figure()
plt.plot(times, x_true, 'r--', label='x_true')
plt.plot(times, x_est, 'b-', label='x_est')
plt.title('Comparación de X vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('X')
plt.legend()
plt.grid(True)

# === Gráfica 3: Posición Y vs Tiempo ===
plt.figure()
plt.plot(times, y_true, 'r--', label='y_true')
plt.plot(times, y_est, 'b-', label='y_est')
plt.title('Comparación de Y vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Y')
plt.legend()
plt.grid(True)

# === Gráfica 4: Theta vs Tiempo ===
plt.figure()
plt.plot(times, theta_true, 'r--', label='theta_true')
plt.plot(times, theta_est, 'b-', label='theta_est')
plt.title('Comparación de Theta vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Theta [rad]')
plt.legend()
plt.grid(True)

# Mostrar todas las gráficas
plt.show()
