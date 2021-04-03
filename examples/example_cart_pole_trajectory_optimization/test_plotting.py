"""Test plotting utilities."""

import unittest

import dash
import pandas as pd
import plotly.graph_objects as go

import plotting


def whole_enchilada(filename: str):
    """Read the simulation results and display an interactive viewer with dash."""
    df = plotting.read_simulation_results(filename)
    plotting.create_interactive_viewer(df)


class TestPlot(unittest.TestCase):
    """Test interactive plotting."""

    def setUp(self):
        """Read the data for every test."""
        filename = "traj.csv"
        self.df = plotting.read_simulation_results(filename)

    def test_read_simulation_results(self):
        """Test reading simulation results."""
        self.assertIsInstance(self.df, pd.DataFrame)
        self.assertListEqual(list(self.df.columns), plotting.COLUMNS)

    def test_create_animation(self):
        """Test creating the animation figure."""
        fig = plotting.create_animation(self.df)
        self.assertIsInstance(fig, go.Figure)

    def test_create_joint_trajectories(self):
        """Test creating the joint trajectories figure."""
        fig = plotting.create_joint_trajectories(self.df)
        self.assertIsInstance(fig, go.Figure)

    def test_create_interactive_viewer(self):
        """Test creating the viewer."""
        app = plotting.create_interactive_viewer(self.df)
        self.assertIsInstance(app, dash.dash.Dash)

    def test_whole_enchilada(self):
        """Test the top level command."""
        filename = "traj.csv"
        whole_enchilada(filename)


if __name__ == "__main__":
    unittest.main()
