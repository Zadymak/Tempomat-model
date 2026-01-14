import streamlit as st
import matplotlib.pyplot as plt
from model import simulate_cruise_control

st.set_page_config(page_title="Tempomat – UAR", layout="centered")
st.title("Tempomat – układ automatycznej regulacji")

# ===== SUWAKI =====

st.subheader("Wartość zadana")
v_set = st.slider("Prędkość zadana [m/s]", 5.0, 40.0, 20.0)

st.subheader("Regulator PI")
kp = st.slider("Wzmocnienie kp", 0.1, 5.0, 0.8)
Ti = st.slider("Czas całkowania Ti [s]", 1.0, 20.0, 5.0)

st.subheader("Model pojazdu")
m = st.slider("Masa pojazdu [kg]", 800, 2500, 1200)
c = st.slider("Opory ruchu c", 10.0, 200.0, 50.0)
ku = st.slider("Wzmocnienie napędu ku", 100.0, 1000.0, 400.0)

st.subheader("Zakłócenie")
slope = st.slider("Nachylenie drogi α [rad]", -0.1, 0.1, 0.0)

# ===== SYMULACJA =====

t, v, u, e = simulate_cruise_control(
    v_set=v_set,
    kp=kp,
    Ti=Ti,
    m=m,
    c=c,
    ku=ku,
    slope=slope
)

# ===== WYKRESY =====

fig, ax = plt.subplots()
ax.plot(t, v, label="v(t) – prędkość")
ax.axhline(v_set, linestyle="--", label="v* – zadana")
ax.set_xlabel("Czas [s]")
ax.set_ylabel("Prędkość [m/s]")
ax.legend()
ax.grid()

st.pyplot(fig)
