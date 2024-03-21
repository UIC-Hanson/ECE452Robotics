import pandas as pd
import os

from csv_utils import overwrite_csv
from csv_utils import append_to_csv

def calculate_alpha_and_export():
    distance = 0.9  # distance in meters of the line
    
    directory = "/home/452Lab/"
    input_filename = os.path.join(directory, "alpha_data.csv")
    output_filename = os.path.join(directory, "alpha_estimation.csv")
    
    if not os.path.exists(input_filename):
        print(f"Input file {input_filename} does not exist.")
        return
    
    data = pd.read_csv(input_filename, usecols=[2, 3], names=['px_power', 'duration'], header=None, skiprows=1)
    data['delta_dr'] = round(distance / data['duration'], 9)
    data['alpha'] = round(data['delta_dr'] / data['px_power'], 9)
    data['decade'] = (data['px_power'] // 10) * 10
    mean_alpha_per_decade = data.groupby('decade')['alpha'].mean().reset_index().sort_values('decade')
    
    # Overwrite the output file with the new headers and the first row of data
    overwrite_csv(mean_alpha_per_decade.iloc[0].tolist(), output_filename, ["decade", "alpha"])
    
    # Append remaining rows without headers
    for index, row in mean_alpha_per_decade.iloc[1:].iterrows():
        append_to_csv(row.tolist(), output_filename, None)  # No headers for appending
    
    # Calculate and append the overall average alpha value
    overall_avg_alpha = round(data['alpha'].mean(), 9)
    append_to_csv(["Overall", overall_avg_alpha], output_filename, None)  # No headers here as well

    print(f"Alpha values by decade have been saved to {output_filename}, including the overall average alpha.")

if __name__ == '__main__':
    calculate_alpha_and_export()