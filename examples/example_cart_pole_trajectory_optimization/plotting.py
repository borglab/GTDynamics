"""Plotting utilities for Dash/Plotly."""

import dash
import dash_core_components as dcc
import dash_html_components as html
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots

COLUMNS = ['t', 'x', 'xdot', 'xddot', 'xtau',
           'theta', 'thetadot', 'thetaddot', 'thetatau']


def read_simulation_results(filename: str):
    """Read the simulation results."""
    df = pd.read_csv(filename)
    return df


PLAY_BUTTON = {
    "args": [None, {"frame": {"duration": 1, "redraw": False},
                    "fromcurrent": True, "transition": {"duration": 300,
                                                        "easing": "quadratic-in-out"}}],
    "label": "Play",
    "method": "animate"
}


PAUSE_BUTTON = {
    "args": [[None], {"frame": {"duration": 0, "redraw": False},
                      "mode": "immediate",
                      "transition": {"duration": 0}}],
    "label": "Pause",
    "method": "animate"
}


def create_interactive_viewer(df):
    """Display an interactive viewer with dash."""

    # Create animation for cart pole.
    # Generate curve data
    t = df.t
    N = len(df.index)
    x, y = df.x, np.zeros((N,))  # cart
    u, v = df.x + 0.3 * np.sin(df.theta), -0.3 * np.cos(df.theta)  # pole

    # Create figure.
    animation = go.Figure(
        data=[go.Scatter(x=u, y=v),
              go.Scatter(x=u, y=v, mode="markers",
                         line=dict(width=1, color="blue"))
              ],
        layout=go.Layout(
            title_text="Cart-pole animation.", hovermode="closest",
            updatemenus=[dict(type="buttons",
                              buttons=[PLAY_BUTTON, PAUSE_BUTTON])],
        ),
        frames=[go.Frame(
            data=[go.Scatter(x=[x[k], u[k]], y=[y[k], v[k]],
                             mode="lines+markers",
                             marker=dict(color="red", size=20))])
                for k in range(N)]
    )

    animation.update_layout(showlegend=False)
    animation.update_yaxes(
        scaleanchor="x",
        scaleratio=1,
    )

    # Create time series with subplots.
    fig = make_subplots(rows=4, cols=2, shared_xaxes="all",
                        subplot_titles=[COLUMNS[k] for k in [1, 5, 2, 6, 3, 7, 4, 8]])

    for i, column in enumerate(COLUMNS[1:]):
        fig.add_trace(go.Scatter(
            x=df.t,
            y=df[column],
            name=column
        ), row=1 + (i % 4), col=1 + (i // 4))

    fig.update_layout(title_text="Joint trajectories:", showlegend=False)

    # Create dash application.
    mathjax = 'https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.4/MathJax.js?config=TeX-MML-AM_CHTML'
    app = dash.Dash(__name__, external_scripts=[mathjax])

    # Create app layout.
    app.layout = html.Div([
        dcc.Graph(id='time', figure=animation),
        dcc.Graph(id='x', figure=fig),
    ])

    # Return entire app.
    return app
