import streamlit as st
import matplotlib.pyplot as plt
from model import simulate_cruise_control

st.set_page_config(layout="wide")
st.title("🚗 Model tempomatu – układ automatycznej regulacji")

# --- PANEL BOCZNY ---
st.sidebar.header("Parametry układu")

v_set = st.sidebar.slider(
    "Prędkość zadana [m/s]",
    5.0, 45.0, 20.0, step=1.0
)

m = st.sidebar.slider(
    "Masa pojazdu [kg]",
    800, 2500, 1400, step=50
)

ku = st.sidebar.slider(
    "Wzmocnienie napędu ku [N]",
    1000, 6000, 3000, step=100
)

kp = st.sidebar.slider(
    "Wzmocnienie regulatora Kp [-]",
    0.1, 2.0, 0.6, step=0.1
)

Ti = st.sidebar.slider(
    "Stała całkowania Ti [s]",
    1.0, 15.0, 6.0, step=0.5
)

c1 = st.sidebar.slider(
    "Opory toczenia c₁ [kg/s]",
    5.0, 120.0, 30.0, step=1.0,
    help="Opory proporcjonalne do prędkości"
)

c2 = st.sidebar.slider(
    "Opór aerodynamiczny c₂ [kg/m]",
    0.5, 8.0, 2.5, step=0.1,
    help="Opór rosnący z kwadratem prędkości"
)

slope = st.sidebar.slider(
    "Nachylenie drogi [rad]",
    0.0, 0.2, 0.0, step=0.01
)

T_sim = st.sidebar.slider(
    "Czas symulacji [s]",
    20, 100, 100, step=10
)

Tp = 0.1
N = int(T_sim / Tp)

# --- SYMULACJA ---
t, v, u, e = simulate_cruise_control(
    v_set=v_set,
    kp=kp,
    Ti=Ti,
    Tp=Tp,
    N=N,
    m=m,
    ku=ku,
    c1=c1,
    c2=c2,
    slope=slope
)

# --- WIZUALIZACJA ---
col1, col2 = st.columns([2, 1])

with col1:
    fig, ax = plt.subplots()
    ax.plot(
        t, v,
        label="v(t)",
        color="tab:blue"
    )

    ax.axhline(
        v_set,
        linestyle="--",
        color="tab:red",
        linewidth=2,
        label="v_zad"
    )
    ax.set_xlabel("Czas [s]")
    ax.set_ylabel("Prędkość [m/s]")
    ax.set_title("Odpowiedź układu regulacji prędkości")
    ax.grid(True)
    ax.legend()
    st.pyplot(fig)

with col2:
    st.subheader("Informacje")

    st.metric("Prędkość ustalona [m/s]", f"{v[-1]:.2f}")
    st.metric("Sterowanie (gaz) [%]", f"{u[-1]*100:.1f}")

    F_res = c1 * v[-1] + c2 * v[-1] ** 2
    st.metric("Siła oporów [N]", f"{F_res:.0f}")

    if u[-1] >= 0.99:
        st.warning("⚠️ Nasycenie napędu – prędkość zadana nieosiągalna")

st.caption(
    "Model uwzględnia regulator PI oraz nieliniowe opory ruchu "
    "(toczenia i aerodynamiczne)."
)
