import csv
import os

# The output track file will be a standard .csv file with column 1 = time, column 2 = dist, column 3 = radius
#---- TO DO

#---- Data required from user
print("This program takes MoTeC time, speed (m/s), and acceleration (m/s/s) data and turns it into a track file for use in the lap sim.\nWhen entering file paths, please use use forward slashes instead of backslashes.\n")

# Get the path to the MoTeC .csv that will be used to create the track, checking that the path exists
data_path = input("Enter the path you to the data you would like to turn into a track:\n") 
while not os.path.exists(data_path):
	data_path = input("Path not found. Please try again. \nEnter the path you to the data you would like to turn into a track:\n")
	
# Let the user decide where they want to save the track file, checking that the path exists, ends with /, and then adding the desired file name to the end
track_path = input("Enter the path for the directory you would like the track to be saved to: \n") 
while not os.path.exists(track_path):
	track_path = input("Path not found. Please try again. \nEnter the path you to the data you would like to turn into a track:\n")
if track_path[len(track_path)-1] != '/':
	track_path += '/'
track_name = input("Enter the name of the track: ")
track_path += track_name + ".csv"

# Figuring out which columns the necessary data are in
time_column = 0 #time is always the first column in .csv files exported by MoTeC
accel_column = int(input("Which column is the lateral acceleration data in? (the first column is column 0) \n")) #column in csv file that corresponds to the lateral acceleration data
speed_column = int(input("Which column is the speed data in? (the first column is column 0) \n")) #column in csv file that corresponds to the gps speed data
print("Creating track...\n")

# Open the data file, find the turn radius at each data point, then save the new info to a csv file
with open(data_path, 'r') as data: 
	reader = csv.reader(data)

	# Create track file, find radius
	with open(track_path, "w", newline = '') as track:
		writer = csv.writer(track, dialect = 'excel')
		d = 0.0
		for line in reader:
			try:
				t = float(line[0])
				speed = float(line[speed_column])
				accel = float(line[accel_column])
				d += speed * dt
				
				# Finding the radius of the turn. If the lateral acceleration is 0, it makes the radius a really high number
				if accel == 0.0:
					radius = 999999
				else:
					radius = speed*2 / accel
				
				writer.writerow([line[time_column], d, radius])

			# This exception skips lines of text. When it finds the line with the sampling rate in it, it uses that to calculate dt
			except ValueError as ve:
				if line[0] == "Sample Rate":
					f = float(line[1])
					dt = 1/f
					print('Found the sampling frequency in the data file, it is: ' + str(f) + ' Hz')
				else:
					print('Found a line that is text instead of data. Skipping it.')

			# This exception skips empty lines
			except IndexError as ie:
				print('Found empty line in data file. Skipping it.')
			

print('Track file successfully created and saved to ' + track_path + '\n')