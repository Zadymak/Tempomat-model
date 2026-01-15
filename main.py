import streamlit as st
import matplotlib.pyplot as plt
from model import simulate_cruise_control

st.set_page_config(page_title="Tempomat – model realistyczny", layout="wide")
st.title("Tempomat – układ automatycznej regulacji (model realistyczny)")

# ===== PANEL STEROWANIA =====

st.sidebar.header("Sterowanie")

v_set = st.sidebar.slider("Prędkość zadana [m/s]", 5.0, 40.0, 20.0)

st.sidebar.subheader("Regulator PI")
kp = st.sidebar.slider("kp", 0.1, 3.0, 0.6)
Ti = st.sidebar.slider("Ti [s]", 1.0, 20.0, 6.0)

st.sidebar.subheader("Pojazd")
m = st.sidebar.slider("Masa pojazdu [kg]", 800, 2500, 1400)
ku = st.sidebar.slider("Moc napędu ku", 1000.0, 5000.0, 3000.0)
c1 = st.sidebar.slider("Opory toczenia c1", 10.0, 80.0, 30.0)
c2 = st.sidebar.slider("Opór aerodynamiczny c2", 0.5, 6.0, 2.5)

st.sidebar.subheader("Zakłócenie")
slope = st.sidebar.slider("Nachylenie drogi α [rad]", -0.1, 0.1, 0.0)

# ===== SYMULACJA =====

t, v, u, e = simulate_cruise_control(
    v_set=v_set,
    kp=kp,
    Ti=Ti,
    m=m,
    ku=ku,
    c1=c1,
    c2=c2,
    slope=slope
)

# ===== UKŁAD KOLUMN =====

col1, col2 = st.columns([2, 1])

# ===== WYKRES =====
with col1:
    st.subheader("Przebieg prędkości")

    fig, ax = plt.subplots()
    ax.plot(t, v, label="v(t) – prędkość")
    ax.axhline(v_set, linestyle="--", label="v* – zadana")

    ax.set_xlabel("Czas [s]")
    ax.set_ylabel("Prędkość [m/s]")
    ax.grid()
    ax.legend()

    st.pyplot(fig)

# ===== INFORMACJE =====
with col2:
    st.subheader("Parametry")
    st.write(f"Prędkość zadana: **{v_set:.1f} m/s**")
    st.write(f"Masa pojazdu: **{m} kg**")
    st.write(f"Nachylenie drogi: **{slope:.2f} rad**")

    st.subheader("Stan końcowy")
    st.write(f"Osiągnięta prędkość: **{v[-1]:.1f} m/s**")
    st.write(f"Gaz (u): **{u[-1]*100:.0f} %**")
