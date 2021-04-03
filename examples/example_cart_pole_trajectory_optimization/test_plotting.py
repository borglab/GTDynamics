"""Test plotting utilities."""

import unittest

import dash
import pandas as pd

import plotting


def whole_enchilada(filename: str):
    """Read the simulation results and display an interactive viewer with dash."""
    df = plotting.read_simulation_results(filename)
    plotting.create_interactive_viewer(df)


class TestPlot(unittest.TestCase):
    """Test interactive plotting."""

    def test_read_simulation_results(self):
        """Test reading simulation results."""
        filename = "traj.csv"
        df = plotting.read_simulation_results(filename)
        self.assertIsInstance(df, pd.DataFrame)
        self.assertListEqual(list(df.columns), plotting.COLUMNS)

    def test_create_interactive_viewer(self):
        """Test creating the viewer."""
        filename = "traj.csv"
        df = plotting.read_simulation_results(filename)
        app = plotting.create_interactive_viewer(df)
        self.assertIsInstance(app, dash.dash.Dash)

    def test_whole_enchilada(self):
        """Test the top level command."""
        filename = "traj.csv"
        whole_enchilada(filename)


if __name__ == "__main__":
    unittest.main()
