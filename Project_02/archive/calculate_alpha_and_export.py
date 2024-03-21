import pandas as pd
import os
import csv  # Make sure to import the csv module

def append_to_csv(data, file_path):
    file_exists = os.path.isfile(file_path)
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["decade", "alpha"])
        writer.writerow(data)

def calculate_alpha_and_export():
    # Define the input and output file paths
    input_filename = os.path.expanduser("~/picarx_run_data.csv")
    output_filename = os.path.expanduser("~/alpha_estimation.csv")
    
    # Check if input file exists
    if not os.path.exists(input_filename):
        print(f"Input file {input_filename} does not exist.")
        return

    # Ensure the directory for the output file exists
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)

    # Load data, ignoring the first two columns (date and time)
    data = pd.read_csv(input_filename, usecols=[2, 3], names=['px_power', 'duration'], header=None, skiprows=1)
    
    # Track length in meters
    distance = 0.9
    
    # Calculate Δdr and α for each entry
    data['delta_dr'] = distance / data['duration']
    data['alpha'] = data['delta_dr'] / data['px_power']
    
    # Group px_power values by decade
    data['decade'] = (data['px_power'] // 10) * 10
    
    # Calculate mean α for each decade, and sort by decade
    mean_alpha_per_decade = data.groupby('decade')['alpha'].mean().reset_index().sort_values('decade')
    
    # Export the results, appending to the CSV or creating it if it doesn't exist
    for index, row in mean_alpha_per_decade.iterrows():
        append_to_csv([row['decade'], row['alpha']], output_filename)

    print("Alpha values by decade have been saved.")
