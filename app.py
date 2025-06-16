import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import numpy as np

from vehicle_model import simulate_vehicle, VEHICLES, RHO
from classic_pi import ClassicPIController
from fuzzy_pi import FuzzyPIController

# Inicjalizacja
app = dash.Dash(__name__)

# Dostępne pojazdy
VEHICLE_OPTIONS = [
    {"label": "Osobowy", "value": "osobowy"},
    {"label": "Sportowy", "value": "sportowy"},
    {"label": "Van", "value": "van"},
    {"label": "Ciężarówka", "value": "ciezarowka"},
]

# Layout aplikacji
app.layout = html.Div(style={'display': 'flex', 'flexDirection': 'row'}, children=[
    # Kolumna lewa 
    html.Div(style={'width': '25%', 'padding': '15px', 'boxSizing': 'border-box', 'overflowY': 'auto', 'maxHeight': '100vh'}, children=[
        html.H2("Symulacja Tempomatu", style={'marginBottom': '15px', 'textAlign': 'center'}),
        
        # Wybór pojazdu 
        html.Div(style={'marginBottom': '15px'}, children=[
            html.Label("Typ pojazdu:", style={'fontWeight': 'bold'}),
            dcc.Dropdown(id="vehicle-dropdown", options=VEHICLE_OPTIONS, value="osobowy", 
                        style={'marginTop': '5px'})
        ]),
        
        # Parametry regulatorów 
        html.Div(style={'display': 'flex', 'marginBottom': '15px', 'gap': '10px'}, children=[
            # Klasyczny PI
            html.Div(style={'flex': 1, 'padding': '10px', 'border': '1px solid #ddd', 'borderRadius': '5px'}, children=[
                html.H5("Regulator klasyczny PI:", style={'marginTop': '0', 'textAlign': 'center'}),
                html.Div(style={'display': 'flex', 'alignItems': 'center', 'marginBottom': '5px'}, children=[
                    html.Label("Kp:", style={'width': '30px', 'marginRight': '5px'}),
                    dcc.Input(id="kp-value", type="number", value=1, min=0.1, step=0.1, style={'flex': 1})
                ]),
                html.Div(style={'display': 'flex', 'alignItems': 'center', 'marginBottom': '5px'}, children=[
                    html.Label("Ti:", style={'width': '30px', 'marginRight': '5px'}),
                    dcc.Input(id="ti-value", type="number", value=1, min=0.1, step=0.1, style={'flex': 1})
                ]),
                html.Div(style={'display': 'flex', 'alignItems': 'center'}, children=[
                    html.Label("tp:", style={'width': '30px', 'marginRight': '5px'}),
                    dcc.Input(id="classic-dt-value", type="number", value=0.01, min=0.001, step=0.001, style={'flex': 1})
                ]),
            ]),
            
            # Rozmyty PI
            html.Div(style={'flex': 1, 'padding': '10px', 'border': '1px solid #ddd', 'borderRadius': '5px'}, children=[
                html.H5("Regulator rozmyty PI:", style={'marginTop': '0', 'textAlign': 'center'}),
                html.Div(style={'display': 'flex', 'alignItems': 'center', 'marginTop': '22px'}), 
                html.Div(style={'display': 'flex', 'alignItems': 'center', 'marginTop': '22px'}), 
                html.Div(style={'display': 'flex', 'alignItems': 'center'}, children=[
                    html.Label("tp:", style={'width': '30px', 'marginRight': '5px'}),
                    dcc.Input(id="fuzzy-dt-value", type="number", value=0.1, min=0.001, step=0.001, style={'flex': 1})
                ]),
            ]),
        ]),
        
        # Trasa 
        html.Div(style={'marginBottom': '15px', 'border': '1px solid #ddd', 'borderRadius': '5px', 'padding': '10px'}, children=[
            html.H5("Trasa (3 sekcje):", style={'marginTop': '0', 'marginBottom': '10px', 'textAlign': 'center'}),
            
            # Pierwsza sekcja
            html.Div(style={'marginBottom': '10px', 'padding': '5px', 'backgroundColor': '#f9f9f9', 'borderRadius': '5px'}, children=[
                html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'center', 'marginBottom': '5px'}, children=[
                    html.Label("Odcinek 1:", style={'fontWeight': 'bold'}),
                    html.Div(style={'display': 'flex', 'alignItems': 'center'}, children=[
                        html.Label("czas [s]:", style={'marginRight': '5px'}),
                        dcc.Input(id="seg1-time", type="number", value=10, min=1, style={'width': '50px'})
                    ]),
                ]),
                html.Label("nachylenie [°]:"),
                dcc.Slider(
                    id="seg1-alpha", 
                    min=-30, 
                    max=30, 
                    step=1, 
                    value=0,
                    marks={-30: "-30", 0: "0", 30: "30"},
                    tooltip={"placement": "bottom", "always_visible": True}
                )
            ]),
            
            # Druga sekcja
            html.Div(style={'marginBottom': '10px', 'padding': '5px', 'backgroundColor': '#f9f9f9', 'borderRadius': '5px'}, children=[
                html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'center', 'marginBottom': '5px'}, children=[
                    html.Label("Odcinek 2:", style={'fontWeight': 'bold'}),
                    html.Div(style={'display': 'flex', 'alignItems': 'center'}, children=[
                        html.Label("czas [s]:", style={'marginRight': '5px'}),
                        dcc.Input(id="seg2-time", type="number", value=10, min=1, style={'width': '50px'})
                    ]),
                ]),
                html.Label("nachylenie [°]:"),
                dcc.Slider(
                    id="seg2-alpha", 
                    min=-30, 
                    max=30, 
                    step=1, 
                    value=10,
                    marks={-30: "-30", 0: "0", 30: "30"},
                    tooltip={"placement": "bottom", "always_visible": True}
                )
            ]),
            
            # Trzecia sekcja
            html.Div(style={'padding': '5px', 'backgroundColor': '#f9f9f9', 'borderRadius': '5px'}, children=[
                html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'center', 'marginBottom': '5px'}, children=[
                    html.Label("Odcinek 3:", style={'fontWeight': 'bold'}),
                    html.Div(style={'display': 'flex', 'alignItems': 'center'}, children=[
                        html.Label("czas [s]:", style={'marginRight': '5px'}),
                        dcc.Input(id="seg3-time", type="number", value=10, min=1, style={'width': '50px'})
                    ]),
                ]),
                html.Label("nachylenie [°]:"),
                dcc.Slider(
                    id="seg3-alpha", 
                    min=-30, 
                    max=30, 
                    step=1, 
                    value=-5,
                    marks={-30: "-30", 0: "0", 30: "30"},
                    tooltip={"placement": "bottom", "always_visible": True}
                )
            ]),
        ]),
        
        # Prędkości zadane 
        html.Div(style={'marginBottom': '15px', 'border': '1px solid #ddd', 'borderRadius': '5px', 'padding': '10px'}, children=[
            html.H5("Prędkości zadane:", style={'marginTop': '0', 'marginBottom': '10px', 'textAlign': 'center'}),
            
            # Tabela prędkości zadanych 
            html.Table(style={'width': '100%', 'borderCollapse': 'collapse'}, children=[
                html.Thead(
                    html.Tr([
                        html.Th("Odcinek", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center', 'backgroundColor': '#f2f2f2'}),
                        html.Th("Czas [s]", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center', 'backgroundColor': '#f2f2f2'}),
                        html.Th("V [km/h]", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center', 'backgroundColor': '#f2f2f2'})
                    ])
                ),
                html.Tbody([
                    html.Tr([
                        html.Td("1", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}),
                        html.Td(
                            dcc.Input(id="vseg1-time", type="number", value=10, min=1, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        ),
                        html.Td(
                            dcc.Input(id="vseg1-v", type="number", value=70, min=0, max=130, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        )
                    ]),
                    html.Tr([
                        html.Td("2", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}),
                        html.Td(
                            dcc.Input(id="vseg2-time", type="number", value=10, min=1, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        ),
                        html.Td(
                            dcc.Input(id="vseg2-v", type="number", value=50, min=0, max=130, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        )
                    ]),
                    html.Tr([
                        html.Td("3", style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}),
                        html.Td(
                            dcc.Input(id="vseg3-time", type="number", value=10, min=1, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        ),
                        html.Td(
                            dcc.Input(id="vseg3-v", type="number", value=60, min=0, max=130, style={'width': '50px', 'padding': '2px'}),
                            style={'border': '1px solid #ddd', 'padding': '5px', 'textAlign': 'center'}
                        )
                    ]),
                ])
            ]),
        ]),
        
        # Przycisk symulacji
        html.Div(style={'textAlign': 'center'}, children=[
            html.Button("Uruchom symulację", id="simulate-btn", n_clicks=0, 
                style={'padding': '10px 20px', 'backgroundColor': '#4CAF50', 'color': 'white', 'border': 'none', 'borderRadius': '5px', 
                      'cursor': 'pointer', 'fontSize': '16px', 'boxShadow': '0 2px 4px rgba(0,0,0,0.2)'})
        ])
    ]),    
    # Kolumna prawa - wykresy 
    html.Div(style={'width': '75%', 'padding': '15px', 'boxSizing': 'border-box', 'backgroundColor': '#fafafa'}, children=[
        dcc.Graph(id="velocity-graph", style={'height': '33vh', 'marginBottom': '5px', 'backgroundColor': 'white', 'borderRadius': '5px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}),
        dcc.Graph(id="classic-forces-graph", style={'height': '33vh', 'marginBottom': '5px', 'backgroundColor': 'white', 'borderRadius': '5px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}),
        dcc.Graph(id="fuzzy-forces-graph", style={'height': '33vh', 'backgroundColor': 'white', 'borderRadius': '5px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'})
    ])
])

def segment_control_colors(time, control_values):
    segments = []
    colors = ["rgba(0,255,0,0.1)" if u > 0.05 else "rgba(255,0,0,0.1)" if u < -0.05 else "rgba(0,0,255,0.1)" for u in control_values]
    start_idx = 0
    for i in range(1, len(time)):
        if colors[i] != colors[start_idx]:
            segments.append((time[start_idx], time[i], colors[start_idx]))
            start_idx = i
    segments.append((time[start_idx], time[-1], colors[start_idx]))
    return segments

# Funkcja pomocnicza: buduje funkcje nachylenia i prędkości zadanej
def build_alpha_func(segments):
    time_bounds = np.cumsum([0] + [seg["time"] for seg in segments])
    def alpha_func(t):
        for i in range(len(segments)):
            if time_bounds[i] <= t < time_bounds[i+1]:
                return segments[i]["alpha"]
        return segments[-1]["alpha"]
    return alpha_func, time_bounds, [seg["alpha"] for seg in segments]

def build_vref_func(segments):
    time_bounds = np.cumsum([0] + [seg["time"] for seg in segments])
    def v_ref_func(t):
        for i in range(len(segments)):
            if time_bounds[i] <= t < time_bounds[i+1]:
                return segments[i]["v"] / 3.6
        return segments[-1]["v"] / 3.6
    return v_ref_func

# Funkcja do obliczania sił działających na pojazd 
def calculate_forces(vehicle_type, time, v, u):
    params = VEHICLES[vehicle_type]
    
    F_drive = np.array([ui * params["F_max"] for ui in u])
    F_aero = np.array([0.5 * RHO * params["Cd"] * params["A"] * vi**2 for vi in v])
    
    return F_drive, F_aero


@app.callback(
    [Output("velocity-graph", "figure"),
     Output("classic-forces-graph", "figure"),
     Output("fuzzy-forces-graph", "figure")],
    Input("simulate-btn", "n_clicks"),
    State("vehicle-dropdown", "value"),
    State("seg1-time", "value"), State("seg1-alpha", "value"),
    State("seg2-time", "value"), State("seg2-alpha", "value"),
    State("seg3-time", "value"), State("seg3-alpha", "value"),
    State("vseg1-time", "value"), State("vseg1-v", "value"),
    State("vseg2-time", "value"), State("vseg2-v", "value"),
    State("vseg3-time", "value"), State("vseg3-v", "value"),
    State("kp-value", "value"), State("ti-value", "value"), 
    State("classic-dt-value", "value"), State("fuzzy-dt-value", "value")
)
def update_simulation(n_clicks, vehicle_type,
                      seg1_time, seg1_alpha, seg2_time, seg2_alpha, seg3_time, seg3_alpha,
                      vseg1_time, vseg1_v, vseg2_time, vseg2_v, vseg3_time, vseg3_v,
                      kp_value, ti_value, classic_dt, fuzzy_dt):
    # Tworzenie regulatorów
    classic_controller = ClassicPIController(Kp=kp_value, Ti=ti_value, output_limit=(-1, 1))
    fuzzy_controller = FuzzyPIController()
    
    
    def classic_controller_func(error, v_curr, t):
        return classic_controller.compute(error, v_curr, t, classic_dt)
    
    def fuzzy_controller_func(error, v_curr, t):
        return fuzzy_controller.compute(error, v_curr, t, fuzzy_dt)
    
    # Przygotowanie segmentów trasy
    alpha_segments = [
        {"time": seg1_time, "alpha": seg1_alpha},
        {"time": seg2_time, "alpha": seg2_alpha},
        {"time": seg3_time, "alpha": seg3_alpha},
    ]
    vref_segments = [
        {"time": vseg1_time, "v": vseg1_v},
        {"time": vseg2_time, "v": vseg2_v},
        {"time": vseg3_time, "v": vseg3_v},
    ]
    
    # Przygotowanie funkcji
    alpha_func, alpha_time_bounds, alpha_values = build_alpha_func(alpha_segments)
    v_ref_func = build_vref_func(vref_segments)
    
    # Przeprowadzenie symulacji dla obu regulatorów
    t_final = sum([s["time"] for s in alpha_segments])
    
    # Symulacja dla regulatora klasycznego
    time_classic, v_classic, u_classic = simulate_vehicle(
        vehicle_type, alpha_func, v_ref_func, classic_controller_func, t_final
    )
    
    # Symulacja dla regulatora rozmytego
    time_fuzzy, v_fuzzy, u_fuzzy = simulate_vehicle(
        vehicle_type, alpha_func, v_ref_func, fuzzy_controller_func, t_final
    )
    
    # Obliczenie prędkości w km/h i kąta nachylenia
    v_classic_kmh = v_classic * 3.6
    v_fuzzy_kmh = v_fuzzy * 3.6
    vref_kmh_classic = np.array([v_ref_func(t)*3.6 for t in time_classic])
    vref_kmh_fuzzy = np.array([v_ref_func(t)*3.6 for t in time_fuzzy])
    alpha_deg_classic = np.array([alpha_func(t) for t in time_classic])
    alpha_deg_fuzzy = np.array([alpha_func(t) for t in time_fuzzy])
    
    # Obliczenie sił dla obu regulatorów
    F_drive_classic, F_aero_classic = calculate_forces(vehicle_type, time_classic, v_classic, u_classic)
    F_drive_fuzzy, F_aero_fuzzy = calculate_forces(vehicle_type, time_fuzzy, v_fuzzy, u_fuzzy)
    
    # Przygotowanie kolorów tła dla obu regulatorów
    segments_classic = segment_control_colors(time_classic, u_classic)
    segments_fuzzy = segment_control_colors(time_fuzzy, u_fuzzy)
    
    # Przygotowanie wykresów
    # 1. Wykres prędkości (regulator klasyczny, rozmyty i zadana)
    fig_velocity = go.Figure()
    
    # Prędkości
    fig_velocity.add_trace(go.Scatter(
        x=time_classic, y=v_classic_kmh, 
        mode="lines", name="Prędkość (klasyczny) [km/h]",
        line=dict(color="royalblue")
    ))
    fig_velocity.add_trace(go.Scatter(
        x=time_fuzzy, y=v_fuzzy_kmh, 
        mode="lines", name="Prędkość (rozmyty) [km/h]",
        line=dict(color="orange")
    ))
    fig_velocity.add_trace(go.Scatter(
        x=time_classic, y=vref_kmh_classic, 
        mode="lines", name="Prędkość zadana [km/h]",
        line=dict(dash="dash", color="black")
    ))
    
    # Nachylenie
    fig_velocity.add_trace(go.Scatter(
        x=time_classic, y=alpha_deg_classic, 
        mode="lines", name="Nachylenie [°]",
        line=dict(color="green"),
        yaxis="y2"
    ))
    
    # Układ wykresu prędkości
    fig_velocity.update_layout(
        title="Porównanie prędkości dla obu regulatorów",
        xaxis_title="Czas [s]",
        yaxis=dict(title="Prędkość [km/h]", side="left"),
        yaxis2=dict(title="Kąt nachylenia [°]", overlaying="y", side="right"),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=50, r=50, t=50, b=50),
        template="plotly_white"
    )
      # 2. Wykres sił dla regulatora klasycznego (z trzema osiami Y)
    fig_classic = make_subplots(specs=[[{"secondary_y": True}]])
    
    # Siła ciągu na pierwszej (lewej) osi Y
    fig_classic.add_trace(go.Scatter(
        x=time_classic, y=F_drive_classic, 
        mode="lines", name="Siła ciągu [N]",
        line=dict(color="green")
    ), secondary_y=False)
    
    # Sygnał sterujący na drugiej osi Y (środkowej)
    fig_classic.add_trace(go.Scatter(
        x=time_classic, y=u_classic, 
        mode="lines", name="Sygnał sterujący",
        line=dict(color="blue")
    ), secondary_y=True)
    
    # Siła oporu na trzeciej osi Y (dodatkowa oś po prawej)
    fig_classic.add_trace(go.Scatter(
        x=time_classic, y=F_aero_classic, 
        mode="lines", name="Siła oporu [N]",
        line=dict(color="red"),
        yaxis="y3"
    ))
    
    # Kolorowe tło
    for x0, x1, color in segments_classic:
        fig_classic.add_vrect(x0=x0, x1=x1, fillcolor=color, layer="below", line_width=0)
      # Układ wykresu klasycznego z trzema osiami Y
    fig_classic.update_layout(
        title="Regulator klasyczny PI - siły i sterowanie",
        xaxis_title="Czas [s]",
        yaxis=dict(title="Siła ciągu [N]", side="left"),
        yaxis2=dict(title="Sygnał sterujący [-]", side="right", range=[-1.2, 1.2], anchor="x", overlaying="y"),
        yaxis3=dict(
            title=None,  
            side="right",
            overlaying="y",
            position=0.92,  
            showgrid=False,
            tickmode='auto',
            tickfont=dict(color="red")
        ),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=50, r=60, t=50, b=50),  
        template="plotly_white",
        annotations=[
            dict(
                x=1.01,  
                y=0.5,   
                xref="paper",
                yref="paper",
                text="Siła oporu [N]",
                showarrow=False,
                font=dict(color="red"),
                textangle=-90
            )
        ]
    )
    
    # Legenda kolorów tła
    fig_classic.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(0,255,0,0.5)'),
        name='Przyspieszanie',
        showlegend=True
    ))
    
    fig_classic.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(255,0,0,0.5)'),
        name='Hamowanie',
        showlegend=True
    ))
    
    fig_classic.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(0,0,255,0.5)'),
        name='Utrzymanie',
        showlegend=True
    ))
      # 3. Wykres sił dla regulatora rozmytego (z trzema osiami Y)
    fig_fuzzy = make_subplots(specs=[[{"secondary_y": True}]])
    
    # Siła ciągu na pierwszej (lewej) osi Y
    fig_fuzzy.add_trace(go.Scatter(
        x=time_fuzzy, y=F_drive_fuzzy, 
        mode="lines", name="Siła ciągu [N]",
        line=dict(color="green")
    ), secondary_y=False)
    
    # Sygnał sterujący na drugiej osi Y (środkowej)
    fig_fuzzy.add_trace(go.Scatter(
        x=time_fuzzy, y=u_fuzzy, 
        mode="lines", name="Sygnał sterujący",
        line=dict(color="blue")
    ), secondary_y=True)
    
    # Siła oporu na trzeciej osi Y (dodatkowa oś po prawej)
    fig_fuzzy.add_trace(go.Scatter(
        x=time_fuzzy, y=F_aero_fuzzy, 
        mode="lines", name="Siła oporu [N]",
        line=dict(color="red"),
        yaxis="y3"
    ))
    
    # Kolorowe tło
    for x0, x1, color in segments_fuzzy:
        fig_fuzzy.add_vrect(x0=x0, x1=x1, fillcolor=color, layer="below", line_width=0)
      # Układ wykresu rozmytego z trzema osiami Y
    fig_fuzzy.update_layout(
        title="Regulator rozmyty PI - siły i sterowanie",
        xaxis_title="Czas [s]",
        yaxis=dict(title="Siła ciągu [N]", side="left"),
        yaxis2=dict(title="Sygnał sterujący [-]", side="right", range=[-1.2, 1.2], anchor="x", overlaying="y"),
        yaxis3=dict(
            title=None,  
            side="right",
            overlaying="y",
            position=0.92,  
            showgrid=False,
            tickmode='auto',
            tickfont=dict(color="red")
        ),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=50, r=60, t=50, b=50),  
        template="plotly_white",
        annotations=[
            dict(
                x=1.01,  
                y=0.5,   
                xref="paper",
                yref="paper",
                text="Siła oporu [N]",
                showarrow=False,
                font=dict(color="red"),
                textangle=-90  
            )
        ]
    )
    
    # Legenda kolorów tła
    fig_fuzzy.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(0,255,0,0.5)'),
        name='Przyspieszanie',
        showlegend=True
    ))
    
    fig_fuzzy.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(255,0,0,0.5)'),
        name='Hamowanie',
        showlegend=True
    ))
    
    fig_fuzzy.add_trace(go.Scatter(
        x=[None], y=[None],
        mode='markers',
        marker=dict(size=15, color='rgba(0,0,255,0.5)'),
        name='Utrzymanie',
        showlegend=True
    ))
    
    return fig_velocity, fig_classic, fig_fuzzy

if __name__ == '__main__':
    app.run(debug=True)