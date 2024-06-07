# import required module
import os
# assign directory
directory = '/Users/mfrahman/Python/bluesky/scenario/ADSL/'
 
# iterate over files in
# that directory
for filename in os.listdir(directory):
    file_path = os.path.join(directory, filename)
        
    # Check if it is a file
    if os.path.isfile(file_path):
        # Open the file for reading
        with open(file_path, 'r') as file:
            # Read the file content
            content = file.read()
        
        # Replace "reso ssd" with "reso mvp" in the content
        content = content.replace("00:00:00.00> reso MVP", "00:00:00.00> reso VO")
        # content = content.replace("00:00:00.00> reso VO", "00:00:00.00> reso MVP")
        # content = content.replace("00:00:00.00> plugin load pos_logger", "00:00:00.00> plugin load CDRLOGGER")
        # content = content.replace("00:00:00.00> plugin load ssd", "00:00:00.00> plugin load vo")
        # content = content.replace("00:00:00.00> reso MVP", "00:00:00.00> reso SSD\n00:00:00.00> PRIORULES TRUE RS1")
        # content = content.replace("00:00:00.00> reso SSD\n00:00:00.00> PRIORULES TRUE RS1", "00:00:00.00> reso MVP")
        # content = content.replace("00:00:00.00> reso SSD\n00:00:00.00> PRIORULES TRUE RS6", "00:00:00.00> reso VO")
        # content = content.replace("00:00:00.00> reso SSD", "00:00:00.00> plugin load ssd\n00:00:00.00> reso SSD")
        # content = content.replace("00:00:00.00> PRIORULES TRUE RS1", "00:00:00.00> PRIORULES TRUE RS6")
        # content = content.replace("00:00:00.00> PRIORULES TRUE RS6", "00:00:00.00> PRIORULES TRUE RS2")

        # content = content.replace("SCHEDULE 00:00:45.00", "SCHEDULE 00:01:00.00")
        # content = content.replace("SCHEDULE 00:01:00.00", "SCHEDULE 00:00:45.00")
        
        # Open the file for writing
        with open(file_path, 'w') as file:
            # Write the modified content back to the file
            file.write(content)
            
        print(f"Modified content in '{filename}'")