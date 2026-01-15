import numpy as np


def simulate_cruise_control(
    v_set=20.0,     # prędkość zadana [m/s]
    v0=0.0,         # prędkość początkowa
    kp=0.6,         # regulator PI
    Ti=6.0,
    Tp=0.1,         # okres próbkowania [s]
    N=1000,         # liczba kroków (100 s)
    m=1400.0,       # masa pojazdu [kg]
    ku=3000.0,      # "moc" napędu
    c1=30.0,        # opory toczenia (liniowe)
    c2=2.5,         # opory aerodynamiczne (kwadratowe)
    slope=0.0       # nachylenie drogi [rad]
):
    g = 9.81

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

        # zakłócenie: podjazd 30–50 s
        if 30 < n * Tp < 50:
            alpha = slope
        else:
            alpha = 0.0

        # model pojazdu (REALISTYCZNY)
        v[n] = v[n-1] + (Tp / m) * (
            ku * u[n]
            - c1 * v[n-1]
            - c2 * v[n-1]**2
            - m * g * alpha
        )

        # zabezpieczenie
        if v[n] < 0:
            v[n] = 0.0

    t = np.arange(N) * Tp
    return t, v, u, e
