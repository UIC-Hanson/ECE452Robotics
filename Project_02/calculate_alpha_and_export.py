import pandas as pd

def calculate_alpha_and_export():
    # Define the input and output file paths
    input_filename = "~/picarx_run_data.csv"
    output_filename = "~/alpha_estimation.csv"

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
    
    # Export the results, overwriting any existing data
    mean_alpha_per_decade.to_csv(output_filename, index=False)

# Example usage
calculate_alpha_and_export()

print("Alpha values by decade have been saved.")