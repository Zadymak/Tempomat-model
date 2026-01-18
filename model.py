import numpy as np


def simulate_cruise_control(
        v_set=20.0,  # Prędkość zadana [m/s]
        v0=0.0,  # Prędkość początkowa [m/s]
        kp=0.6,  # Wzmocnienie regulatora
        Ti=6.0,  # Stała całkowania [s]
        Tp=0.1,  # Krok symulacji [s]
        N=1000,  # Liczba próbek
        m=1400.0,  # Masa pojazdu [kg]
        ku=3000.0,  # Wzmocnienie napędu [N]
        c1=30.0,  # Opory toczenia [kg/s]
        c2=2.5,  # Opór aerodynamiczny [kg/m]
        slope=0.0  # Nachylenie drogi [rad]
):
    """
    Symulacja układu tempomatu metodą Eulera (rozwiązanie rekurencyjne).
    """
    g = 9.81

    # Inicjalizacja tablic
    v = np.zeros(N)
    u = np.zeros(N)
    e = np.zeros(N)

    # Warunki początkowe
    v[0] = v0

    # Zmienne pomocnicze dla algorytmu przyrostowego
    u_prev = 0.0
    e_prev = 0.0

    for n in range(1, N):
        # 1. Obliczenie uchybu regulacji
        e[n] = v_set - v[n - 1]

        # 2. Regulator PI – algorytm przyrostowy
        # delta_u = kp * (delta_e + (Tp/Ti)*e)
        delta_e = e[n] - e_prev
        du = kp * (delta_e + (Tp / Ti) * e[n])

        u[n] = u_prev + du

        # Ograniczenie sterowania (nasycenie 0% - 100%)
        u[n] = np.clip(u[n], 0.0, 1.0)

        # 3. Zakłócenie (Nachylenie drogi - stałe dla całego przebiegu)
        current_slope = slope

        # 4. Model fizyczny pojazdu (Bilans sił)
        F_drive = ku * u[n]
        F_resist = c1 * v[n - 1] + c2 * (v[n - 1] ** 2)
        F_gravity = m * g * np.sin(current_slope)

        # Równanie różnicowe (Metoda Eulera)
        v[n] = v[n - 1] + (Tp / m) * (F_drive - F_resist - F_gravity)

        # Zabezpieczenie przed ujemną prędkością
        if v[n] < 0:
            v[n] = 0.0

        # Zapamiętanie stanu do następnego kroku
        u_prev = u[n]
        e_prev = e[n]

    t = np.arange(N) * Tp
    return t, v, u, e