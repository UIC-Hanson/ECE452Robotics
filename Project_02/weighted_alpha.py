# weighted_alpha.py
import pandas as pd
import argparse
import os

def weighted_alpha_for_power(input_filename, power):
    alpha_by_decade = pd.read_csv(input_filename)
    # Calculation logic as provided...

def main():
    parser = argparse.ArgumentParser(description="Calculate weighted alpha for a given power.")
    parser.add_argument("power", type=int, help="Power value to calculate for")
    args = parser.parse_args()

    input_filename = os.path.expanduser("~/alpha_estimation.csv")
    weighted_alpha = weighted_alpha_for_power(input_filename, args.power)
    # Output handling as provided...

if __name__ == "__main__":
    main()
