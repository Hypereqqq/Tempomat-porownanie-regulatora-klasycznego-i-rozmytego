import numpy as np

# Stałe fizyczne
G = 9.81  # przyspieszenie ziemskie [m/s^2]
RHO = 1.225  # gęstość powietrza [kg/m^3]

# Definicje pojazdów
VEHICLES = {
    "osobowy":      {"mass": 1500, "A": 2.2, "Cd": 0.29, "F_max": 4500},
    "sportowy":     {"mass": 1300, "A": 1.9, "Cd": 0.27, "F_max": 7500},
    "van":          {"mass": 2200, "A": 2.8, "Cd": 0.33, "F_max": 5000},
    "ciezarowka":   {"mass": 15000, "A": 5.0, "Cd": 0.6,  "F_max": 40000},
}

def simulate_vehicle(vehicle_type, alpha_func, v_ref_func, controller_func, t_final=30, dt=0.1):
    """
    Symuluje ruch pojazdu z danym typem pojazdu i funkcjami nachylenia, prędkości zadanej oraz sterowania.
    """
    params = VEHICLES[vehicle_type]
    m = params['mass']
    A = params['A']
    Cd = params['Cd']
    F_max = params['F_max']

    # Czas
    time = np.arange(0, t_final + dt, dt)
    v = np.zeros_like(time)
    u_out = np.zeros_like(time)

    for i in range(1, len(time)):
        t = time[i]
        alpha = np.radians(alpha_func(t))  # zamiana stopni na radiany
        #print(f"t={t:.2f}, alpha={alpha_func(t):.2f}° ({alpha:.2f} rad)")
        v_ref = v_ref_func(t)
        v_curr = v[i-1]

        # Błąd
        error = v_ref - v_curr

        # Sygnał sterujący u(t) - przekazujemy dt do funkcji kontrolera
        u = controller_func(error, v_curr, t)
        u = np.clip(u, -1.0, 1.0)  # ograniczenie sygnału [-1, 1]
        u_out[i] = u

        # Siły
        F_aero = 0.5 * RHO * Cd * A * v_curr**2
        F_gravity = m * G * np.sin(alpha)
        #print(np.sin(alpha))
        F_drive = u * F_max

        # Równanie ruchu
        dv = (F_drive - F_aero - F_gravity) / m
        v[i] = v_curr + dv * dt
        v[i] = max(v[i], 0.0) 

    #print(f"v={v_curr:.2f}, v_ref={v_ref:.2f}, u={u:.2f}")
    return time, v, u_out
