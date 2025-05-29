import os
import shutil
from time import time

from concurrent.futures import ThreadPoolExecutor, Future
from threading import Event

def find_file_with_extension(base_filename, file_extension, root_dir, stop_event: Event):
    """Search for a file with a specific base filename and extension in a directory."""
    if stop_event.is_set():  # Check if the search should stop
        return None

    for root, _, files in os.walk(root_dir):
        if stop_event.is_set():  # Check again after each directory
            return None
        
        for file in files:
            file: str
            # Check if the file name matches the base filename and ends with the correct extension
            if file.startswith(base_filename) and file.endswith(file_extension):
                norm_full_path: str = os.path.normpath(os.path.join(root, file))
                norm_abs_path: str = os.path.normpath(os.path.abspath("deploy/ozone-elastic-layout.json"))

                if norm_full_path.lower() != norm_abs_path.lower():
                    return os.path.join(root, file)  # Return the full path of the file
    return None

def search_and_copy_in_dir(base_filename, file_extension, root_dir, target_dir, stop_event: Event):
    """Search for the file in a specific directory and copy it if found and modified."""
    file_path = find_file_with_extension(base_filename, file_extension, root_dir, stop_event)
    
    if file_path:
        target_file_path = os.path.join(target_dir, base_filename + file_extension)
        stop_event.set()
        
        # Check if the file should be copied (if modified)
        if is_file_modified(file_path, target_file_path):
            print(f"Found {base_filename + file_extension} in {root_dir}. Copying to {target_dir}.")
            shutil.copy(file_path, target_file_path)
            print("File copied successfully.")
            
            # Set the event to signal other threads to stop searching
        else:
            print(f"The file {base_filename + file_extension} is not modified. No need to replace.")
    else:
        print(f"File {base_filename + file_extension} not found in {root_dir}.")

def search_and_copy(filename_base, target_dir, file_extension=".json", concurrent_limit=3):
    """Search for the file (without extension) in multiple directories and copy it to the target directory."""
    # Directories to search (could be Downloads, Documents, Desktop, etc.)
    s_time = time()

    directories_to_search = [
        os.path.expanduser("~\\Downloads"),
        os.path.expanduser("~\\Documents"),
        os.path.expanduser("~\\Desktop"),
        os.path.expanduser("~\\Pictures"),  # Example additional directory
        os.path.expanduser("~\\Videos"),    # Example additional directory
        # Add more directories if needed
    ]

    if os.path.isdir(os.path.expanduser("~\\OneDrive\\Documents")):
        directories_to_search.insert(2, os.path.expanduser("~\\Onedrive\\Documents"))
        concurrent_limit += 1
    
    # Split the directories into two parts: first X for concurrent search, the rest for sequential search
    concurrent_dirs = directories_to_search[:concurrent_limit]
    sequential_dirs = directories_to_search[concurrent_limit:]
    
    # Create an Event to signal when the file is found
    stop_event = Event()

    # Use ThreadPoolExecutor to search the first 'concurrent_limit' directories in parallel
    with ThreadPoolExecutor() as executor:
        futures: list[Future] = []
        
        # Submit tasks for each of the concurrent directories
        for root_dir in concurrent_dirs:
            futures.append(executor.submit(search_and_copy_in_dir, filename_base, file_extension, root_dir, target_dir, stop_event))
        
        # Wait for all threads to complete for the concurrent directories
        for future in futures:
            future.result() # Ensure any exceptions raised in the threads are propagated
            print(f"Took {time() - s_time} seconds")

    # Now, search the remaining directories sequentially, but check the stop_event
    for root_dir in sequential_dirs:
        if stop_event.is_set():
            print("File found, stopping further search.")
            print(f"Took {time() - s_time} seconds")
            return
        search_and_copy_in_dir(filename_base, file_extension, root_dir, target_dir, stop_event)
        print(f"took {time() - s_time}")

def is_file_modified(src, dest):
    """Check if the source file is newer than the destination file."""
    if not os.path.exists(dest):
        return True  # If destination doesn't exist, treat it as modified
    
    src_mtime = os.path.getmtime(src)  # Last modified time of the source file
    dest_mtime = os.path.getmtime(dest)  # Last modified time of the target file
    
    return src_mtime > dest_mtime  # Return True if source is newer