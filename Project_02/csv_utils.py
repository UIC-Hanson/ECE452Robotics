# csv_utils.py
import csv
import os

def append_to_csv(data, file_path, headers):
    """Appends a row of data to a CSV file, adding headers if the file is new."""
    file_exists = os.path.isfile(file_path)
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists and headers:
            writer.writerow(headers)
        writer.writerow(data)
