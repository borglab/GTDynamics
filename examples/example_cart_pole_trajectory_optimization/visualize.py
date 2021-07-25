""" Visualize cart-pole trajectory.
    Run this app with `python app.py` and
    visit http://127.0.0.1:8050/ in your web browser.
"""


import argparse
from pathlib import Path

import plotting

if __name__ == "__main__":
    # Parse arguments.
    parser = argparse.ArgumentParser(
        description='Visualize Cart-pole.')
    parser.add_argument('-f', '--file', type=str,
                        default="traj.csv",
                        help='CSV file')
    args = parser.parse_args()

    # Create and run Dash app.
    df = plotting.read_simulation_results(args.file)
    app = plotting.create_interactive_viewer(df)
    app.run_server(debug=False)
