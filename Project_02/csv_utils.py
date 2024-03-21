import csv
import os

def prepare_file(file_path, mode):
    """
    Prepares the file for writing by ensuring the directory exists and notifying the user.
    
    Args:
    - file_path: Path to the CSV file (str).
    - mode: The mode of operation ('append' or 'overwrite') (str).
    """
    #action = "Creating a new file" if mode == 'overwrite' or not os.path.isfile(file_path) else "Appending to existing file"
    #print(f"{action} at: {file_path}")
    #this was a bit excessive.
    
    # Ensure the directory exists
    os.makedirs(os.path.dirname(file_path), exist_ok=True)

def write_to_csv(data, file_path, headers=None, mode='append'):
    """
    Writes or appends a row of data to a CSV file based on the mode.
    If headers is None and mode is 'append', headers won't be written.
    
    Args:
    - data: The data to write (list).
    - file_path: Path to the CSV file (str).
    - headers: Headers for the CSV file (list or None).
    - mode: The mode of operation ('append' or 'overwrite') (str).
    """
    # Prepare the file for writing
    prepare_file(file_path, mode)
    
    # Determine the file mode for opening
    file_mode = 'w' if mode == 'overwrite' else 'a'
    
    with open(file_path, mode=file_mode, newline='') as file:
        writer = csv.writer(file)
        # Write headers if overwriting, or if the file did not exist and headers are provided
        if (mode == 'overwrite' or not os.path.isfile(file_path)) and headers is not None:
            writer.writerow(headers)
        writer.writerow(data)

def append_to_csv(data, file_path, headers=None):
    """
    Appends a row of data to a CSV file. If the file is new, adds headers if provided.
    
    Args:
    - data: The data to append (list).
    - file_path: Path to the CSV file (str).
    - headers: Headers for the CSV file, if any (list or None).
    """
    write_to_csv(data, file_path, headers, mode='append')

def overwrite_csv(data, file_path, headers=None):
    """
    Overwrites or creates a new CSV file with the provided data. Adds headers if provided.
    
    Args:
    - data: The data to write (list).
    - file_path: Path to the CSV file (str).
    - headers: Headers for the CSV file, if any (list or None).
    """
    write_to_csv(data, file_path, headers, mode='overwrite')
