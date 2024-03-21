import pandas as pd
import os

from csv_utils import overwrite_csv, append_to_csv

def calculate_alpha_and_export():
    distance = 0.9  # distance in meters of the line
    
    directory = "/home/452Lab/"
    input_filename = os.path.join(directory, "alpha_data.csv")
    output_filename = os.path.join(directory, "alpha_estimation.csv")
    
    if not os.path.exists(input_filename):
        print(f"Input file {input_filename} does not exist.")
        return
    
    data = pd.read_csv(input_filename, usecols=[2, 3], names=['px_power', 'duration'], header=None, skiprows=1)
    data['delta_dr'] = distance / data['duration']
    data['alpha'] = data['delta_dr'] / data['px_power']

    # Apply rounding to the entire 'alpha' column
    data['alpha_rounded'] = data['alpha'].apply(lambda x: round(x, 9))

    data['decade'] = (data['px_power'] // 10) * 10
    mean_alpha_per_decade = data.groupby('decade')['alpha_rounded'].mean().reset_index().sort_values('decade')

    # Use overwrite_csv and append_to_csv as before, ensuring to pass 'alpha_rounded' for the alpha values
    
    # Ensure rounding is correctly applied to the overall average calculation
    overall_avg_alpha = round(data['alpha'].mean(), 9)
    append_to_csv(["Overall", overall_avg_alpha], output_filename, None)


    print(f"Alpha values by decade have been saved to {output_filename}, including the overall average alpha.")

if __name__ == '__main__':
    calculate_alpha_and_export()