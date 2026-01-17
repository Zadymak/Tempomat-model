import numpy as np


def simulate_cruise_control(
    v_set=20.0,     # prędkość zadana [m/s]
    v0=0.0,         # prędkość początkowa [m/s]
    kp=0.6,         # wzmocnienie regulatora
    Ti=6.0,         # stała całkowania [s]
    Tp=0.1,         # krok symulacji [s]
    N=1000,         # liczba próbek
    m=1400.0,       # masa pojazdu [kg]
    ku=3000.0,      # wzmocnienie napędu [N]
    c1=30.0,        # opory toczenia [kg/s]
    c2=2.5,         # opór aerodynamiczny [kg/m]
    slope=0.0       # nachylenie drogi [rad]
):
    g = 9.81

    v = np.zeros(N)
    u = np.zeros(N)
    e = np.zeros(N)

    v[0] = v0
    u_prev = 0.0
    e_prev = 0.0

    for n in range(1, N):
        # uchyb regulacji
        e[n] = v_set - v[n - 1]

        # regulator PI – algorytm przyrostowy
        du = kp * ((e[n] - e_prev) + (Tp / Ti) * e[n])
        u[n] = u_prev + du
        u[n] = np.clip(u[n], 0.0, 1.0)

        # zakłócenie – podjazd w środku symulacji
        current_slope = slope if 300 < n < 500 else 0.0

        # siły
        F_drive = ku * u[n]
        F_resist = c1 * v[n - 1] + c2 * v[n - 1] ** 2
        F_gravity = m * g * np.sin(current_slope)

        # równanie ruchu
        v[n] = v[n - 1] + (Tp / m) * (F_drive - F_resist - F_gravity)
        if v[n] < 0:
            v[n] = 0.0

        u_prev = u[n]
        e_prev = e[n]

    t = np.arange(N) * Tp
    return t, v, u, e
