import pandas as pd
import numpy as np
import dash
import dash_bootstrap_components as dbc
from dash import dcc, html, Input, Output
import plotly.graph_objs as go

# Load data from CSV file
# Ensure the file is in the same directory or provide the full path
filepath = "Data.csv"
raw_df = pd.read_csv(filepath, header=None, names=["Raw"], dtype=str, delimiter="\t", engine="python")
raw_df = raw_df[raw_df["Raw"].notnull() & raw_df["Raw"].str.contains(",")] # 

# Extract header
header = [h.strip() for h in raw_df.iloc[0, 0].split(',')] 
data = raw_df.iloc[1:].copy() 
data = data["Raw"].str.split(",", expand=True)
data.columns = header

# Parse timestamp 
data["Timestamp"] = data["Timestamp"].str.strip("[]")
data["Timestamp"] = pd.to_datetime(data["Timestamp"], format="%Y-%m-%d %H:%M:%S", errors="coerce")

# Convert other columns to numeric 
for col in data.columns:
    if col != "Timestamp":
        data[col] = pd.to_numeric(data[col], errors="coerce")

# Drop rows without timestamp 
data.dropna(subset=["Timestamp"], inplace=True) 
data.reset_index(drop=True, inplace=True)

# Interpolate high-frequency data between full-second timestamps
change_indices = data.index[data["Timestamp"] != data["Timestamp"].shift()].tolist()
interpolated_times = [None] * len(data)

for i in range(len(change_indices) - 1):
    start_idx = change_indices[i]
    end_idx = change_indices[i + 1] - 1
    t_start = data.loc[start_idx, "Timestamp"]
    count = end_idx - start_idx + 1
    times = pd.date_range(start=t_start, periods=count, freq=f"{int(1e6 // count)}us")
    interpolated_times[start_idx:end_idx + 1] = times.tolist()

# Interpolate the last segment
if len(change_indices) >= 1:
    start_idx = change_indices[-1]
    end_idx = len(data) - 1
    t_start = data.loc[start_idx, "Timestamp"]
    count = end_idx - start_idx + 1
    if count > 1:
        times = pd.date_range(start=t_start, periods=count, freq=f"{int(1e6 // count)}us")
    else:
        times = [t_start]
    interpolated_times[start_idx:end_idx + 1] = times

data["Interpolated Time"] = interpolated_times

# Select signal columns (excluding Timestamp and Flag)
signal_columns = [col for col in data.columns if col not in ["Timestamp", "Interpolated Time", "Flag"]]

# Initialize Dash app
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.title = "Bike Data Visualizer"

# App layout
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([
            html.H2("Bike Data Visualizer", className="mt-3"),
            html.Hr(),
            html.Label("Select Signals"),
            dbc.Checklist(
                id="signal-selector",
                options=[{"label": col, "value": col} for col in signal_columns],
                value=signal_columns[:2],
                inline=False,
                inputClassName="me-2",
                className="mb-3"
            ),
            html.Label("Options"),
            dbc.Checklist(
                options=[{"label": "Show Flag Lines", "value": "flags"}],
                value=["flags"],
                id="flag-toggle",
                switch=True,
                className="mb-3"
            ),
            html.Small("Hold Ctrl/Cmd to select multiple signals.\nRed lines = flags.")
        ], width=3, className="bg-light p-3"),

        dbc.Col([
            dcc.Graph(id="data-plot", config={"displayModeBar": True, "scrollZoom": True})
        ], width=9)
    ])
], fluid=True)

# Plot the available signals
@app.callback(
    Output("data-plot", "figure"),
    Input("signal-selector", "value"),
    Input("flag-toggle", "value")
)
def update_graph(selected_signals, show_flags):
    fig = go.Figure()

    for sig in selected_signals:
        fig.add_trace(go.Scatter(
            x=data["Interpolated Time"], y=data[sig],
            mode="lines", name=sig,
            line=dict(width=2)
        ))

        # Add flag markers if selected
        if "flags" in show_flags and "Flag" in data.columns:
            flagged = data[data["Flag"] == 1]
            fig.add_trace(go.Scatter(
                x=flagged["Interpolated Time"], y=flagged[sig],
                mode="markers", name=f"{sig} Flag",
                marker=dict(color="red", size=10, symbol="x")
            ))

    fig.update_layout(
        title="Sensor Signals Over Time",
        xaxis_title="Time",
        yaxis_title="Value",
        template="plotly_white",
        height=700,
        hovermode="x unified",
        margin=dict(l=20, r=20, t=60, b=20),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        dragmode="zoom"
    )

    return fig

# Launch the app
if __name__ == "__main__":
    app.run(debug=True)