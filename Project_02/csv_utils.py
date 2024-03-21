# csv_utils.py
import csv
import os

def append_to_csv(data, file_path, headers):
    """Appends a row of data to a CSV file, adding headers if the file is new."""
    # Check if the file exists before attempting to open it
    file_exists = os.path.isfile(file_path)
    
    if not file_exists:
        # If the file doesn't exist, notify the user about the new file creation
        print(f"Creating a new file at: {file_path}")
        # Ensure the directory for the new file exists; create it if necessary
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
    
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        # If the file was not existing and headers are provided, write headers first
        if not file_exists and headers:
            writer.writerow(headers)
        writer.writerow(data)