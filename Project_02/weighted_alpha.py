import pandas as pd
import argparse
import os

def weighted_alpha_for_power(input_filename, power):
    try:
        # Load the CSV file into a DataFrame
        alpha_by_decade = pd.read_csv(input_filename)
        
        # Check if the 'decade' and 'alpha' columns exist in the DataFrame
        if 'decade' not in alpha_by_decade.columns or 'alpha' not in alpha_by_decade.columns:
            print("Error: The input file does not contain the required 'decade' and/or 'alpha' columns.")
            return None
        
        # If the exact decade match exists, return its alpha value
        if power % 10 == 0 and power in alpha_by_decade['decade'].values:
            return alpha_by_decade.loc[alpha_by_decade['decade'] == power, 'alpha'].values[0]
        
        # Calculate weighted alpha for powers not exactly on a decade
        lower_decade = (power // 10) * 10
        upper_decade = lower_decade + 10
        if lower_decade in alpha_by_decade['decade'].values and upper_decade in alpha_by_decade['decade'].values:
            lower_alpha = alpha_by_decade.loc[alpha_by_decade['decade'] == lower_decade, 'alpha'].values[0]
            upper_alpha = alpha_by_decade.loc[alpha_by_decade['decade'] == upper_decade, 'alpha'].values[0]
            lower_weight = (upper_decade - power) / 10
            upper_weight = (power - lower_decade) / 10
            weighted_alpha = (lower_alpha * lower_weight) + (upper_alpha * upper_weight)
            return weighted_alpha
        else:
            print("The given power does not match any decade or surrounding decades in the dataset.")
            return None
    except FileNotFoundError:
        print(f"Error: The file '{input_filename}' was not found.")
        return None
    except pd.errors.EmptyDataError:
        print(f"Error: The file '{input_filename}' is empty.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

def main(power=None):
    # Bypass argument parsing if power is provided directly (useful for testing)
    if power is None:
        parser = argparse.ArgumentParser(description="Calculate weighted alpha for a given power.")
        parser.add_argument("power", type=int, help="Power value to calculate for")
        args = parser.parse_args()
        power = args.power

    directory = "/home/452Lab/"
    input_filename = os.path.join(directory, "alpha_estimation.csv")
    weighted_alpha = weighted_alpha_for_power(input_filename, power)
    if weighted_alpha is not None:
        print(f"The weighted alpha for power {power} is: {weighted_alpha}")
    else:
        print("Failed to calculate weighted alpha due to an error or invalid power value.")

# Test function
def test_weighted_alpha():
    print("Testing weighted alpha calculation...")
    test_powers = [45, 50, 55]  # Example power values for testing
    for power in test_powers:
        print(f"Testing for power: {power}")
        main(power)

if __name__ == "__main__":
    # Determine if the script is being run directly or imported for testing
    if 'get_ipython' in globals():
        # Running in an interactive environment; you can call test_weighted_alpha() manually
        pass
    else:
        # Running as a script
        main()
