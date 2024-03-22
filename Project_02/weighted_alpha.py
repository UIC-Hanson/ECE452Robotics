import pandas as pd
import argparse
import os
pd.options.display.float_format = '{:.4f}'.format

def weighted_alpha_for_power(input_filename, power):
    try:
        alpha_by_decade = pd.read_csv(input_filename)
        # overall_power = alpha_by_decade.loc[alpha_by_decade['px_power'] == "Overall"].values[0][1]
        # Drop overall
        alpha_by_decade = alpha_by_decade[alpha_by_decade.px_power != "Overall"]
        # Convert to int
        alpha_by_decade = alpha_by_decade.apply(pd.to_numeric, errors='coerce')
        if power < 0 or power > 100:
            print(f"Value out of range.")
            return None, f"{power}"

        if power in [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]:  # if it's on a decade
            weighted_alpha = alpha_by_decade.loc[alpha_by_decade['px_power'] == power].values[0][1]
            return weighted_alpha, f"{power}"
        else:
            # Calculate tens digit to determine the lower and upper decades
            tens_digit = (int(power) // 10) % 10

            # Calculate the upper and lower decade power values
            upper_power = (tens_digit + 1) * 10
            lower_power = tens_digit * 10

            # Retrieve alpha values for the upper and lower decades
            try:
                upper_decade_alpha = alpha_by_decade.loc[alpha_by_decade['px_power'] == upper_power, 'alpha'].values[0]
            except IndexError:
                print(f"No alpha value found for power {upper_power}")
                upper_decade_alpha = None
            try:
                lower_decade_alpha = alpha_by_decade.loc[alpha_by_decade['px_power'] == lower_power, 'alpha'].values[0]
            except IndexError:
                print(f"No alpha value found for power {lower_power}")
                lower_decade_alpha = None

            if upper_decade_alpha is not None and lower_decade_alpha is not None:
                # Calculate weights based on the ones digit of the power
                ones_digit = int(power) % 10
                upper_weight = ones_digit / 10.0
                lower_weight = 1 - upper_weight

                # Calculate and return the weighted alpha
                weighted_alpha = (upper_decade_alpha * upper_weight) + (lower_decade_alpha * lower_weight)
                return weighted_alpha, f"{power}"
            else:
                print(f"Unable to calculate weighted alpha for power {power} due to missing decade data.")
                return None, f"{power}"

    except FileNotFoundError:
        print(f"Error: The file '{input_filename}' was not found.")
        return None, f"{power}"
    except pd.errors.EmptyDataError:
        print(f"Error: The file '{input_filename}' is empty.")
        return None, f"{power}"
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None, f"{power}"


def weight_average(low_power, low_value, high_power, high_value, desired_power):
    power_diff = high_power - low_power
    desired_power_position = (desired_power - low_power) / power_diff  # Fractional position of the desired power output
    low_power_weight = low_value * (1-desired_power_position)
    high_power_weight = high_value * desired_power_position
    print(f"{desired_power_position} {low_power_weight} {high_power_weight}")
    return low_power_weight + high_power_weight


def expect_actual(expect, actual):
    if expect == actual:
        print(f"\033[32mOK {expect}\033[0m")
    else:
        print(f"\033[31mExpected:\033[0m {expect} Got {actual}")


def test():
    expect_actual(15, weight_average(10, 10, 20, 20, 15))
    expect_actual(45, weight_average(40, 40, 50, 50, 45))
    expect_actual(12, weight_average(10, 10, 20, 20, 12))


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
    #test()
    main()