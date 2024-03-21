import pandas as pd
import os
from csv_utils import append_to_csv

def calculate_alpha_and_export():
    directory = "/home/452Lab/"

    input_filename = os.path.join(directory, "picarx_run_data.csv")
    output_filename = os.path.join(directory, "alpha_estimation.csv")
    
    # Check if the input file exists
    if not os.path.exists(input_filename):
        print(f"Input file {input_filename} does not exist.")
        return

    # Ensure the output directory exists
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    
    data = pd.read_csv(input_filename, usecols=[2, 3], names=['px_power', 'duration'], header=None, skiprows=1)
    distance = 0.9
    data['delta_dr'] = distance / data['duration']
    data['alpha'] = data['delta_dr'] / data['px_power']
    data['decade'] = (data['px_power'] // 10) * 10
    mean_alpha_per_decade = data.groupby('decade')['alpha'].mean().reset_index().sort_values('decade')

    for index, row in mean_alpha_per_decade.iterrows():
        append_to_csv([row['decade'], row['alpha']], output_filename, ["decade", "alpha"])

    print(f"Alpha values by decade have been saved to {output_filename}.")

calculate_alpha_and_export()
