import numpy as np

def simulate_cruise_control(
    v_set=20.0,     # prędkość zadana [m/s]
    v0=0.0,         # prędkość początkowa
    kp=0.8,         # regulator PI
    Ti=5.0,
    Tp=0.1,         # okres próbkowania
    m=1200.0,       # masa pojazdu [kg]
    c=50.0,         # opory ruchu
    ku=400.0,       # wzmocnienie napędu
    slope=0.0,      # nachylenie drogi [rad]
    N=300           # liczba kroków
):
    g = 9.81  # przyspieszenie ziemskie

    v = np.zeros(N)
    u = np.zeros(N)
    e = np.zeros(N)

    v[0] = v0

    for n in range(1, N):
        # uchyb regulacji
        e[n] = v_set - v[n-1]

        # regulator PI (algorytm przyrostowy)
        du = kp * (e[n] - e[n-1]) + (Tp / Ti) * e[n]
        u[n] = u[n-1] + du

        # ograniczenie sterowania (gaz 0–100%)
        u[n] = np.clip(u[n], 0.0, 1.0)

        # zakłócenie: nachylenie drogi (aktywne w środku symulacji)
        if 10 < n * Tp < 20:
            alpha = slope
        else:
            alpha = 0.0

        # model pojazdu (obiekt sterowania)
        v[n] = v[n-1] + (Tp / m) * (
            ku * u[n]
            - c * v[n-1]
            - m * g * alpha
        )

    t = np.arange(N) * Tp
    return t, v, u, e
