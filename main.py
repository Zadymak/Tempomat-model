import numpy as np
from dash import Dash, html, dcc, callback, Output, Input, State
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# =============================================================================
# PRESETY POJAZDÓW
# =============================================================================
VEHICLE_PRESETS = {
    "city_car": {
        "name": "🚗 Samochód osobowy",
        "mass": 1200,  # kgdescription": "Wysokowydajny pojazd sportowy - szybka reakcja"
        "drag_coeff": 50,  # współczynnik oporów ruchu
        "max_traction": 3500,  # N
        "max_brake": 7000,  # N
        "color": "#FF6B35",  # turkusowy
    },
    "truck": {
        "name": "🚛 Ciężarówka",
        "mass": 25000,  # kg
        "drag_coeff": 300,  # współczynnik oporów ruchu [N·s/m]
        "max_traction": 40000,  # N
        "max_brake": 80000,  # N
        "color": "#FF6B35",  # pomarańczowy
    },
    "sports_car": {
        "name": "🏎️ Samochód sportowy",
        "mass": 1600,  # kg
        "drag_coeff": 80,  # współczynnik oporów ruchu
        "max_traction": 14000,  # N
        "max_brake": 28000,  # N
        "color": "#E63946",  # czerwony
    }
}


# =============================================================================
# KLASA SYMULACJI TEMPOMATU
# =============================================================================
class CruiseControlSimulator:
    """
    Symulator tempomatu z regulatorem PID i mechanizmem anti-windup.
    Model oparty na równaniu: m·dv/dt = F_trac - F_brake - b·v
    """

    def __init__(self, vehicle_params, kp, Tp, Ti, Td):
        self.mass = vehicle_params["mass"]
        self.drag_coeff = vehicle_params["drag_coeff"]
        self.max_traction = vehicle_params["max_traction"]
        self.max_brake = vehicle_params["max_brake"]
        self.kp = kp
        self.Tp = Tp
        self.Ti = Ti
        self.Td = Td

    def simulate(self, v_ref, v0, t_end):
        dt = self.Tp
        dt_sim = 0.001
        n_steps = int(t_end / dt) + 1

        t = np.linspace(0, t_end, n_steps)
        v = np.zeros(n_steps)
        e = np.zeros(n_steps)
        u = np.zeros(n_steps)
        f_trac = np.zeros(n_steps)
        f_brake = np.zeros(n_steps)
        integral = np.zeros(n_steps)
        derivative = np.zeros(n_steps)

        v[0] = v0
        integral_sum = 0.0
        v_max_ref = 50.0  # normalizacja
        e_prev = (v_ref - v0) / v_max_ref

        for i in range(1, n_steps):
            e_raw = v_ref - v[i - 1]
            e[i - 1] = e_raw / v_max_ref

            delta_e = e[i - 1] - e_prev if i > 1 else 0.0
            derivative[i - 1] = delta_e

            u_P = self.kp * e[i - 1]
            u_I = self.kp * (self.Tp / self.Ti) * integral_sum
            u_D = self.kp * (self.Td / self.Tp) * delta_e

            u_raw = u_P + u_I + u_D
            u_normalized = u_raw

            u[i - 1] = np.clip(u_normalized, -1.0, 1.0)

            if abs(u_normalized) < 1.0 or (e[i - 1] * u_normalized < 0):
                integral_sum += e[i - 1]

            integral[i - 1] = integral_sum
            e_prev = e[i - 1]

            if u[i - 1] >= 0:
                f_trac[i - 1] = u[i - 1] * self.max_traction
                f_brake[i - 1] = 0
            else:
                f_trac[i - 1] = 0
                f_brake[i - 1] = -u[i - 1] * self.max_brake

            v_current = v[i - 1]
            n_substeps = int(dt / dt_sim)
            for _ in range(n_substeps):
                f_drag = self.drag_coeff * v_current
                dv_dt = (f_trac[i - 1] - f_brake[i - 1] - f_drag) / self.mass
                v_current = v_current + dv_dt * dt_sim
                v_current = max(0, v_current)
            v[i] = v_current

        e[-1] = (v_ref - v[-1]) / v_max_ref
        u[-1] = u[-2] if len(u) > 1 else 0
        f_trac[-1] = f_trac[-2] if len(f_trac) > 1 else 0
        f_brake[-1] = f_brake[-2] if len(f_brake) > 1 else 0
        integral[-1] = integral_sum
        derivative[-1] = derivative[-2] if len(derivative) > 1 else 0

        return {
            "time": t, "velocity": v, "error": e, "control": u,
            "traction": f_trac, "brake": f_brake, "integral": integral,
            "derivative": derivative, "v_ref": v_ref
        }


# =============================================================================
# KONWERSJE I WYKRESY
# =============================================================================
def ms_to_kmh(v_ms): return v_ms * 3.6


def kmh_to_ms(v_kmh): return v_kmh / 3.6


def create_simulation_plots(results, vehicle_params, show_kmh=True, previous_results=None):
    color = vehicle_params["color"]
    t = results["time"]

    if show_kmh:
        v = ms_to_kmh(results["velocity"])
        v_ref = ms_to_kmh(results["v_ref"])
        v_unit = "km/h"
    else:
        v = results["velocity"]
        v_ref = results["v_ref"]
        v_unit = "m/s"

    fig = make_subplots(
        rows=1, cols=2, shared_xaxes=False, horizontal_spacing=0.08,
        column_widths=[0.5, 0.5],
        subplot_titles=("Prędkość pojazdu", "Siły i sygnał sterujący")
    )

    # Poprzedni przebieg
    if previous_results is not None:
        t_prev = previous_results["time"]
        v_prev = ms_to_kmh(previous_results["velocity"]) if show_kmh else previous_results["velocity"]
        fig.add_trace(go.Scatter(
            x=t_prev, y=v_prev, mode='lines', name=f'Poprzedni [{v_unit}]',
            line=dict(color='#6C757D', width=2, dash='dot'),
            hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
        ), row=1, col=1)

    # Aktualny przebieg
    fig.add_trace(go.Scatter(
        x=t, y=v, mode='lines', name=f'Prędkość [{v_unit}]',
        line=dict(color=color, width=3),
        hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=t, y=[v_ref] * len(t), mode='lines', name=f'Zadana [{v_unit}]',
        line=dict(color='#00D9A5', width=2, dash='dash'),
        hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    ), row=1, col=1)

    # Wykresy sił
    # fig.add_trace(go.Scatter(
    #     x=t, y=results["control"], mode='lines', name='Sterowanie [-1:1]',
    #     line=dict(color='#BB86FC', width=2),
    #     hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    # ), row=1, col=2)

    fig.add_trace(go.Scatter(
        x=t, y=results["traction"] / 1000, mode='lines', name='Napęd [kN]',
        line=dict(color='#03DAC6', width=2),
        hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    ), row=1, col=2)

    fig.add_trace(go.Scatter(
        x=t, y=results["brake"] / 1000, mode='lines', name='Hamowanie [kN]',
        line=dict(color='#CF6679', width=2),
        hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    ), row=1, col=2)

    f_drag = vehicle_params["drag_coeff"] * results["velocity"] / 1000
    fig.add_trace(go.Scatter(
        x=t, y=f_drag, mode='lines', name='Opory [kN]',
        line=dict(color='#FFAB40', width=2, dash='dot'),
        hovertemplate='%{y:.2f}'  # ZAOKRĄGLENIE
    ), row=1, col=2)

    fig.update_layout(
        height=500, showlegend=True, template="plotly_dark",
        paper_bgcolor='#1E1E1E', plot_bgcolor='#2D2D2D',
        title=dict(text=f"<b>Symulacja - {vehicle_params['name']}</b>", font=dict(size=20, color=color), x=0.5),
        legend=dict(orientation="h", y=-0.25, x=0.5, xanchor="center"),
        font=dict(family="Arial", color='#E0E0E0'), hovermode='x unified'
    )

    fig.update_xaxes(title_text="Czas [s]", gridcolor='#444', row=1, col=1)
    fig.update_xaxes(title_text="Czas [s]", gridcolor='#444', row=1, col=2)
    fig.update_yaxes(title_text=f"Prędkość [{v_unit}]", gridcolor='#444', row=1, col=1)
    fig.update_yaxes(title_text="Siła [kN]", gridcolor='#444', row=1, col=2)

    return fig


# =============================================================================
# APLIKACJA DASH
# =============================================================================
app = Dash(__name__)
app.title = "Symulator Tempomatu"

DARK_BG = '#121212'
DARK_CARD = '#1E1E1E'
DARK_CARD_LIGHTER = '#2D2D2D'
DARK_TEXT = '#E0E0E0'
DARK_TEXT_SECONDARY = '#A0A0A0'
ACCENT_COLOR = '#BB86FC'

app.layout = html.Div([

    # 1. SEKCJA INFORMACYJNA (NAMIARY NA AUTORÓW)
    html.Div([
        html.H1("Symulator Tempomatu", style={'color': ACCENT_COLOR, 'marginBottom': '10px'}),
        html.Div([
            html.P("Autorzy: Filip Godzich, Dawid Majdziński (Grupa Lab5)", style={'fontSize': '18px', 'fontWeight': 'bold'}),
            html.P("Nr indeksu: 166462, 166379", style={'color': DARK_TEXT_SECONDARY}),
            html.P("Projekt tempomatu - Podstawy Automatyki",
                   style={'color': DARK_TEXT_SECONDARY, 'fontStyle': 'italic'}),
        ])
    ], style={'textAlign': 'center', 'padding': '20px', 'backgroundColor': DARK_CARD, 'marginBottom': '20px'}),

    # 2. SEKCJA MODELU MATEMATYCZNEGO (ROZWIJANA)
    # html.Details([
    #     html.Summary("📐 Równowaga sił i model matematyczny (kliknij aby rozwinąć)",
    #                  style={'cursor': 'pointer', 'fontSize': '16px', 'fontWeight': 'bold', 'color': ACCENT_COLOR,
    #                         'marginBottom': '10px'}),
    #     html.Div([
    #         dcc.Markdown(r'''
    #         Model dynamiki pojazdu oparty jest na **II zasadzie dynamiki Newtona**:
    #
    #         $$m \cdot \frac{dv(t)}{dt} = F_{nap}(t) - F_{ham}(t) - F_{op}(t)$$
    #
    #         Gdzie:
    #         * $m$ - masa pojazdu [kg]
    #         * $F_{nap}$ - siła napędowa [N]
    #         * $F_{ham}$ - siła hamowania [N]
    #         * $F_{op} = b \cdot v(t)$ - siła oporów ruchu proporcjonalna do prędkości
    #         ''', mathjax=True)
    #     ], style={'padding': '20px', 'backgroundColor': DARK_CARD_LIGHTER, 'borderRadius': '5px'})
    # ], style={'backgroundColor': DARK_CARD, 'padding': '10px', 'borderRadius': '5px', 'marginBottom': '20px'}),

    # 3. GŁÓWNY INTERFEJS SYMULATORA
    html.Div([
        # LEWY PANEL - SUWAKI
        html.Div([
            html.Div([
                html.Label("🚙 Typ pojazdu:", style={'fontWeight': 'bold', 'color': DARK_TEXT}),
                dcc.Dropdown(
                    id='vehicle-dropdown',
                    options=[{'label': v['name'], 'value': k} for k, v in VEHICLE_PRESETS.items()],
                    value='city_car',
                    clearable=False,
                    style={'marginTop': '5px', 'backgroundColor': DARK_CARD_LIGHTER, 'color': '#000'}
                ),
            ], style={'marginBottom': '20px'}),

            html.Hr(style={'borderColor': '#333'}),
            html.H4("📊 Parametry", style={'color': ACCENT_COLOR}),

            html.Label("🎯 Prędkość zadana [km/h]:"),
            dcc.Slider(
                id='speed-slider', min=30, max=150, step=5, value=90,
                marks={i: f'{i}' for i in range(30, 151, 30)}
            ),

            html.Label("🚦 Prędkość początkowa [km/h]:", style={'marginTop': '15px'}),
            dcc.Slider(
                id='initial-speed-slider', min=0, max=120, step=5, value=0,
                marks={i: f'{i}' for i in range(0, 121, 30)}
            ),

            html.Label("⏱️ Czas [s]:", style={'marginTop': '15px'}),
            dcc.Slider(
                id='time-slider', min=60, max=300, step=30, value=120,
                marks={i: f'{i}' for i in range(60, 301, 60)}
            ),

            html.Hr(style={'borderColor': '#333', 'marginTop': '20px'}),
            html.H4("⚙️ Regulator PID", style={'color': ACCENT_COLOR}),

            html.Label("Kp (Wzmocnienie):"),
            dcc.Slider(
                id='kp-slider', min=1, max=50, step=1, value=15,
                marks={i: f'{i}' for i in range(0, 51, 10)}
            ),

            html.Label("Tp (Próbkowanie):", style={'marginTop': '10px'}),
            dcc.Slider(
                id='tp-slider', min=0.1, max=1.0, step=0.1, value=0.5,
                marks={0.1: '0.1', 0.5: '0.5', 1.0: '1.0'}
            ),

            html.Label("Ti (Zdwojenie):", style={'marginTop': '10px'}),
            dcc.Slider(
                id='ti-slider', min=0.1, max=10, step=0.1, value=5,
                marks={i: f'{i}' for i in range(0, 11, 2)}
            ),

            html.Label("Td (Wyprzedzenie):", style={'marginTop': '10px'}),
            dcc.Slider(
                id='td-slider', min=0, max=5, step=0.1, value=0.1,
                marks={i: f'{i}' for i in range(0, 6, 1)}
            ),

            html.Button('🚀 Uruchom symulację', id='simulate-button', n_clicks=0,
                        style={'width': '100%', 'backgroundColor': ACCENT_COLOR, 'border': 'none',
                               'padding': '15px', 'fontWeight': 'bold', 'marginTop': '20px', 'borderRadius': '5px'})
        ], style={'width': '300px', 'padding': '20px', 'backgroundColor': DARK_CARD, 'borderRadius': '10px',
                  'marginRight': '20px'}),

        # PRAWY PANEL - WYKRESY
        html.Div([
            html.Div(id='vehicle-params-display',
                     style={'padding': '15px', 'backgroundColor': DARK_CARD, 'marginBottom': '15px',
                            'borderRadius': '10px'}),
            html.Div([
                dcc.Loading(
                    type="circle", color=ACCENT_COLOR,
                    children=[dcc.Graph(id='simulation-graph', style={'height': '550px'})]
                )
            ], style={'backgroundColor': DARK_CARD, 'borderRadius': '10px', 'padding': '10px'})
        ], style={'flex': '1'})

    ], style={'display': 'flex', 'alignItems': 'flex-start'}),

    dcc.Store(id='previous-results-store'),

], style={
    'maxWidth': '100%',
    'margin': '0',
    'padding': '20px',
    'fontFamily': 'Arial, sans-serif',
    'backgroundColor': DARK_BG,
    'minHeight': '100vh',
    'boxSizing': 'border-box',
    'color': DARK_TEXT
})


@callback(Output('vehicle-params-display', 'children'), Input('vehicle-dropdown', 'value'))
def update_params(v_type):
    p = VEHICLE_PRESETS[v_type]
    return html.Div([
        html.H3(f"⚙️ {p['name']}", style={'color': p['color'], 'margin': '0 0 10px 0'}),
        html.Div([
            f"Masa: {p['mass']} kg | Opór: {p['drag_coeff']} | ",
            f"Napęd max: {p['max_traction']} N | Hamulec max: {p['max_brake']} N"
        ], style={'color': DARK_TEXT_SECONDARY})
    ])


@callback(
    Output('simulation-graph', 'figure'),
    Output('previous-results-store', 'data'),
    Input('simulate-button', 'n_clicks'),
    State('vehicle-dropdown', 'value'),
    State('speed-slider', 'value'), State('initial-speed-slider', 'value'),
    State('time-slider', 'value'), State('kp-slider', 'value'),
    State('tp-slider', 'value'), State('ti-slider', 'value'),
    State('td-slider', 'value'), State('previous-results-store', 'data')
)
def run_simulation(n, v_type, v_ref_kmh, v0_kmh, t_sim, kp, Tp, Ti, Td, prev_data):
    params = VEHICLE_PRESETS[v_type]
    v_ref = kmh_to_ms(v_ref_kmh)
    v0 = kmh_to_ms(v0_kmh)

    sim = CruiseControlSimulator(params, kp, Tp, Ti, Td)
    res = sim.simulate(v_ref, v0, t_sim)

    prev_res = None
    if prev_data:
        prev_res = {"time": np.array(prev_data["time"]), "velocity": np.array(prev_data["velocity"])}

    fig = create_simulation_plots(res, params, show_kmh=True, previous_results=prev_res)
    current_data = {"time": res["time"].tolist(), "velocity": res["velocity"].tolist()}

    return fig, current_data


if __name__ == '__main__':
    app.run(debug=True, host='127.0.0.1', port=8050)