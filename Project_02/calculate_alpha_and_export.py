import pandas as pd

def calculate_alpha_and_export(input_filename, output_filename):
    # Load data, ignoring the first two columns (date and time)
    data = pd.read_csv(input_filename, usecols=[2, 3], names=['px_power', 'duration'], header=None, skiprows=1)
    
    # Track length in meters
    distance = 0.9
    
    # Calculate Δdr and α for each entry
    data['delta_dr'] = distance / data['duration']
    data['alpha'] = data['delta_dr'] / data['px_power']
    
    # Group px_power values by decade
    data['decade'] = (data['px_power'] // 10) * 10
    
    # Calculate mean α for each decade
    mean_alpha_per_decade = data.groupby('decade')['alpha'].mean().reset_index()
    
    # Export the results
    mean_alpha_per_decade.to_csv(output_filename, index=False)

# Example usage
input_filename = "path/to/your/picarx_run_data.csv"  # Update this path
output_filename = "path/to/your/alpha_estimation.csv"  # Update this path
calculate_alpha_and_export(input_filename, output_filename)

print("Alpha values by decade have been saved.")
