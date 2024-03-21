import pandas as pd
import argparse
import os

def weighted_alpha_for_power(input_filename, power):
    try:
        alpha_by_decade = pd.read_csv(input_filename)
        # Filtering to exclude non-numeric 'decade' values
        alpha_by_decade = alpha_by_decade[~alpha_by_decade['decade'].astype(str).str.contains('Overall', na=False)]
        print(alpha_by_decade)

        power = float(power)
        lower_decade = (power // 10) * 10
        upper_decade = lower_decade + 10.0

        if power in alpha_by_decade['decade'].values:
            direct_alpha = alpha_by_decade.loc[alpha_by_decade['decade'] == power, 'alpha'].values[0]
            return direct_alpha, f"{power:.1f}"
        elif lower_decade in alpha_by_decade['decade'].values and upper_decade in alpha_by_decade['decade'].values:
            lower_alpha = alpha_by_decade.loc[alpha_by_decade['decade'] == lower_decade, 'alpha'].values[0]
            upper_alpha = alpha_by_decade.loc[alpha_by_decade['decade'] == upper_decade, 'alpha'].values[0]
            lower_weight = (upper_decade - power) / 10.0
            upper_weight = (power - lower_decade) / 10.0
            weighted_alpha = (lower_alpha * lower_weight) + (upper_alpha * upper_weight)
            print(f"Weighted alpha between {lower_decade:.1f} and {upper_decade:.1f}: {weighted_alpha}")
            return weighted_alpha, f"{power:.1f}"
        else:
            print("The given power does not match any decade or surrounding decades in the dataset.")
            return None, f"{power:.1f}"
    except FileNotFoundError:
        print(f"Error: The file '{input_filename}' was not found.")
        return None, f"{power:.1f}"
    except pd.errors.EmptyDataError:
        print(f"Error: The file '{input_filename}' is empty.")
        return None, f"{power:.1f}"
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None, f"{power:.1f}"

def main():
    parser = argparse.ArgumentParser(description="Calculate weighted alpha for a given power.")
    parser.add_argument("power", type=int, help="Power value to calculate for")
    args = parser.parse_args()

    directory = "/home/452Lab/"
    input_filename = os.path.join(directory, "alpha_estimation.csv")
    weighted_alpha, formatted_power = weighted_alpha_for_power(input_filename, args.power)
    
    if weighted_alpha is not None:
        print(f"The weighted alpha for power {formatted_power} is: {weighted_alpha}")
    else:
        print(f"Failed to calculate weighted alpha for power {formatted_power}.")

if __name__ == "__main__":
    main()