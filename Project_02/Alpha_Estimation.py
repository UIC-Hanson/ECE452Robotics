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

    # Calculate 'alpha_rounded' to ensure alpha values are rounded to 9 decimal places
    data['alpha_rounded'] = data['alpha'].apply(lambda x: round(x, 9))

    data['px_power'] = (data['px_power'] // 10) * 10
    mean_alpha_per_decade = data.groupby('px_power')['alpha_rounded'].mean().reset_index().sort_values('px_power')
    
    # Create a dataframe with all desired decades
    all_decades = pd.DataFrame({'px_power': range(0, 101, 10)})

    # Merge with mean_alpha_per_decade, filling missing values as needed
    mean_alpha_per_decade = all_decades.merge(mean_alpha_per_decade, on='px_power', how='left').fillna(value={"alpha_rounded": 0})

    # Overwrite the file with the first row of data (headers included)
    overwrite_csv([mean_alpha_per_decade.iloc[0]['px_power'], round(mean_alpha_per_decade.iloc[0]['alpha_rounded'], 9)], output_filename, ["px_power", "alpha"])
    
    # Append the remaining rows
    for index, row in mean_alpha_per_decade.iloc[1:].iterrows():
        append_to_csv([row['px_power'], round(row['alpha_rounded'], 9)], output_filename, None)

    # Calculate and append the overall average alpha value, ensuring rounding is applied
    overall_avg_alpha = round(data['alpha'].mean(), 9)
    append_to_csv(["Overall", overall_avg_alpha], output_filename, None)

    print(f"Alpha values by power decade have been saved to {output_filename}, including the overall average alpha.")

if __name__ == '__main__':
    calculate_alpha_and_export()