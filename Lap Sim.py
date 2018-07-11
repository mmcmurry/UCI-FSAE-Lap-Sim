from math import sqrt, pi

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~Version 1.0~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TO-DO~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# - Make braking more realistic using forces rather than constant acceleration
# - Change Euler integration to RK4
# - Add power curve, gears

# Some general methods used in the Car class and in setting up testing
def quadraticFormula(a,b,c):
    # Given coefficients of a quadratic, finds x-coordinates of the zeroes and returns them in a list.
    result = []
    result.append((-b+sqrt(b**2-4*a*c))/(2*a))
    result.append((-b-sqrt(b**2-4*a*c))/(2*a))
    return result

def findVertex(a,b,c):
    x = -b/(2*a)
    y = a*x**2 + b*x + c
    return [x, y]

def readTrackFile(path):
        # Turns a .txt file into a track. Works when you copy/paste radius/angle/distance data from Excel into Notepad
        file = open(path)
        track = []
        
        for line in file:
            turn = line.split('\t')
            for i in range(3):
                turn[i] = float(turn[i])
            track.append(turn)
            
        file.close()
        return track

# The Car class. This is where all of the calculations are done. Instantiate a Car object and use Car.findDynamicTimes() to test it
class Car:
    def __init__(self, name, m, P, mu, width, a_brake, df, drag):
        self.name = name # Just the name of the instance
        self.mass = m # Mass of the car with fuel and driver, in kilograms
        self.power = P # Average power of the engine, calculated using acceleration event times, in Watts
        self.mu = mu # Coefficient of static friction
        self.a_brake = a_brake # Longitudinal acceleration under peak braking
		self.cl = cl # Coefficient of lift
		self.cd = cd # Coefficient of drag
		self.f_area = f_area # Frontal area of car 
		self.rho = 1.225 # Air density in kg/m^3
        self.width = width # Width of the car in meters
        self.dt = 0.001        
    
    def getDownforce(self, velocity):
		# Returns Newtons of downforce as a positive number
		return 0.5 * self.cl * self.f_area * self.rho * velocity**2 

	def getDrag(self, velocity):
		# Returns Newtons of drag as a positive number
		return 0.5 * self.cd * self.f_area * self.rho * velocity**2
    
    def findCorneringSpeed(self, radius):
        # Returns the maximum speed in meters per second that the car could go through a corner of given radius
        # Finds speed where centripetal force = cornering force. This formula can be reduced into a quadratic and solved

        radius = radius + 0.5*self.width
        
        combined_cl = 0.5 * self.cl * self.f_area * self.rho
		a = self.mass/radius - self.mu * combined_cl
		b = 0
		c = -1 * self.mu * self.mass * 9.81
		
		v = max(quadraticFormula(a,b,c))
		return v

    def findCorneringTime(self, radius, theta):
        # Returns time to go through a corner in seconds given the radius of the turn in meters and the angle of the turn
        # Uses Car.findCorneringSpeed() to find velocity through turn, then computes time through turn with t = d/v
        v = self.findCorneringSpeed(radius)
        radius = radius + 0.5*self.width
        d = 2*pi*radius * theta/360
        return d/v
    
    def findStraightTime(self, v_i, v_f, length):
        # Figures out how long it takes to go a specified distance starting and ending at specified speeds using numerical integration
        t = 0
        d = 0
        d_prev = 0
        v = v_i
        ke = 0.5*self.mass*v**2
        
        # While d is less than the length of the straight minus the distance required to slow down, accelerate
        while(d < length - self.findBrakingDistance(v, v_f)):
            ke += self.power*self.dt - self.getDrag(v)*(d-d_prev)
            v = sqrt(2*ke/self.mass)
            d_prev = d
            d += v*self.dt
            t += self.dt
                
        # The previous loop exited, so now it is time to slow down by subtracting the braking acceleration from velocity
        while(d<length):
            v -= self.a_brake*self.dt
            d += v*self.dt
            t += self.dt
        
        return t

    def findBrakingDistance(self, v, v_f):
        # Finds distance required to slow from v to v_f using a formula derived using some basic calculus. Contact Matt McMurry for more info
        # Enter -1 for v_f to not include braking (for use in Car.findStraightTime)
        if v_f == -1:
            return 0
        else:
            dv = v-v_f
            return (dv/self.a_brake) * ((dv/2) + v_f)
    
    def findLapTime(self, track, ax_bool=None):
        # Calculates a lap time given a track. ax_bool = true will make it so that the car accelerates to the finish, rather than slow down as if there was a turn
        # Tracks are 2d lists, where each inner list is a turn made up of [radius, degrees, distance to next turn]. The direction of the turn doesn't matter
        t = 0
        if not ax_bool:
            ax_bool = False
        # Finds corner velocities for vor every turn (for calculating straight times) and finds cornering times and adds
        # them to the total lap time
        corner_speeds = []
        for turn in track:
            corner_speeds.append(self.findCorneringSpeed(turn[0]))
            t += self.findCorneringTime(turn[0], turn[1])
            
        # Iterate through turns to find time spent in the straights
        for turn in range(len(track)):
            if turn == len(track)-1:
                next_turn = 0
            else:
                next_turn = turn + 1
            
            # If it's an autocross track, don't slow down at the end
            if ax_bool and next_turn == 0:
                t += self.findStraightTime(corner_speeds[turn], -1, track[turn][2])
            else:
                t += self.findStraightTime(corner_speeds[turn], corner_speeds[next_turn], track[turn][2])
            
        return t
                
    def findDynamicTimes(self, p_bool, autox_track, endurance_track):
        # Calculates times in acceleration, skidpad, autocross, and endurance using car parameters and points data from Lincoln 2017
        times = []
        times.append(round(self.findStraightTime(0, -1, 75), 3))
        times.append(round(self.findCorneringTime(8.12, 360), 3))
        times.append(round(self.findLapTime(autox_track, True), 3))
        times.append(round(self.findLapTime(endurance_track) * 16, 3)) # Endurance is 16 laps, so just multiply the time of one lap times 16
        if p_bool:
            print(self.name + ' dynamic event times: ')
            print('Accel: ' + str(times[0]) + '\tSkidpad: ' + str(times[1]) + '\tAutoX: ' + str(times[2]) + '\tEnduro: ' + str(times[3]) + '\n')
        return times
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END CAR CLASS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~VARIABLE SETUP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
power = 23390 # Watts
m_car = 234.0 # kg
m_driver = 75.0 # kg
m_aeroPackage = 12 # kg
width_car = 1.2192 # m
mu_s = 1.31985
a_brake17 = 1.2 * 9.81 # m/s^2
a_brake18 = 1.3 * 9.81 # m/s^2
clift = 2.961
cdrag = 1.974
farea = 1

skidpad_track = [[8.12, 360, 0]]

#!!! Change these paths to match the paths to the track files on your computer
autox_track = readTrackFile("C:\\Users\\matt\\OneDrive\\Documents\\_School\\_FSAE\\2017 autocross map.txt")
endurance_track = readTrackFile("C:\\Users\\matt\\OneDrive\\Documents\\_School\\_FSAE\\2017 endurance map.txt")


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TESTING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Mantis17 is the old car
# Mantis18 is the old car + aero. 
# Mantis18a is Mantis18 - 16 kg (so we are average weight w/o aero), + 2147 W from new exhaust
Mantis17 = Car("Mantis 17", m_car+m_driver, power, mu_s, width_car, a_brake17, clift, cdrag, farea)
Mantis18 = Car("Mantis 18", m_car+m_driver+m_aeroPackage, power, mu_s, width_car, a_brake18, clift, cdrag, farea)
Mantis18a = Car("Mantis 18a", m_car+m_driver+m_aeroPackage-16, power+2147, mu_s, width_car, a_brake18, clift, cdrag, farea)

# Finding dynamic event times for all of the cars
m17 = Mantis17.findDynamicTimes(True, autox_track, endurance_track)
m18 = Mantis18.findDynamicTimes(True, autox_track, endurance_track)
m18a = Mantis18a.findDynamicTimes(True, autox_track, endurance_track)

events = ['acceleration', 'skidpad', 'autocross', 'endurance']

# Print differences between 2017 and 2018 cars
for event in range(len(m17)):
    diff = m17[event]-m18[event]
    percent = diff/m17[event]*100
    print(str(round(percent,2)) + '% difference from 2017 to 2018 in ' + events[event])
print()

# Print differences between 2017 and 2018a cars
for event in range(len(m17)):
    diff = m17[event]-m18a[event]
    percent = diff/m17[event]*100
    print(str(round(percent,2)) + '% difference from 2017 to 2018a in ' + events[event])
    
# Calculating the accuracy of the model
model_accuracy = 0
irl_times = [4.452, 5.252, 57.268, 1485.083]
for event in range(len(m17)):
    print(str(round(abs((m17[event]-irl_times[event])/irl_times[event]*100),2)) + '% difference from irl to model in ' + events[event] + '\n')
    
    model_accuracy += abs((m17[event]-irl_times[event])/m17[event]*100)

model_accuracy /= 4
print('\nModel is ' + str(model_accuracy)[:4] + '% inaccurate on average across the four events')   
