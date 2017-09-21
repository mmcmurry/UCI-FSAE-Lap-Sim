from math import sqrt, pi

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TO-DO~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# - Make acceleration from a standstill more realistic by including some form of a clutch. The car initially accelerates very slowly at
#   first because it is at the minimum rpm, where power is very low. When starting irl, you rev the engine up for more power and then 
#   use the clutch to control how much of that power you deliver to the wheels.

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

# The Drivetrain class is used to create drivetrain objects that are passed into the constructor for a Car object. Used for calculating power, torque, force, etc.
class Drivetrain:
	def __init__(self, power_curve, gear_ratios, crank_sprocket_ratio, tire_radius):
		self.power_curve = power_curve # A list of length 2 lists that are points on the power curve
		self.min_rpm = power_curve[0][0]
		self.max_rpm = power_curve[len(power_curve)-1][0]
		self.min_power = power_curve[0][1]
		self.max_power = power_curve[len(power_curve)-1][1]
		self.gear_ratios = gear_ratios
		self.crank_sprocket_ratio = crank_sprocket_ratio
		self.tire_radius = tire_radius
		
		for i in range(len(power_curve)-1):
			rise = power_curve[i][1]-power_curve[i+1][1]
			run = power_curve[i][0]-power_curve[i+1][0]
			slope = rise/run
			power_curve[i].append(slope) # The slope from point i to point i+1
		
		self.rpm = self.min_rpm
		self.gear = 1
	
	def overrev(self):
		if self.rpm > self.max_rpm:
			return True
		else:
			return False
	
	def underrev(self):
		if self.rpm < self.min_rpm:
			return True
		else:
			return False
	
	def setRPM(self, velocity):
	# Given velocity, find RPM based on gearing. Haven't been able to get this to put out realistic numbers
		self.rpm = velocity/self.tire_radius * self.gear_ratios[self.gear-1] * self.crank_sprocket_ratio * 60 / (2*pi)
		if self.rpm < self.min_rpm and self.gear == 1: # Don't let the engine 'stall'
			self.rpm = self.min_rpm
		
		return self.rpm
	
	def selectGear(self, velocity):
	# Given velocity, select the correct gear based on minimum and maximum rpms
		self.setRPM(velocity)
		while(self.overrev() or self.underrev()):
			# Upshift if revs are too high and not in final gear
			if self.overrev():
				if self.gear != len(self.gear_ratios):
					self.gear += 1
				else:
					#print("Redline in 6th!")
					return self.gear
			# Downshift if revs are too low and not in 1st gear		
			elif self.underrev():
				if self.gear != 1:
					self.gear -= 1
				else:
					#print("Engine stalling in 1st!")
					return self.gear
			self.setRPM(velocity)
					
		print("Gear: " + str(self.gear))
		return self.gear	
	
	def getPower(self):
	# Returns power in Watts based on current rpm of the engine. Uses linear interpolation between points in power_curve
	#!!! Doesn't give any power if less than min rpm !!!
	#y=m(x-x1)+y1
		power = 0
		for i in range(len(self.power_curve)-1):
			if self.rpm >= self.power_curve[i][0] and self.rpm < self.power_curve[i+1][0]:
				power = self.power_curve[i][2] * (self.rpm - self.power_curve[i][0]) + self.power_curve[i][1]
			
		return power
	
	def getTorque(self):
	# Finds torque in N*m from power at current rpm 
		torque = self.getPower()/(self.rpm*((2*pi)/60))
		return torque 
	
	def getWheelForce(self):
	# Converts torque from the engine to force on the ground
		force = self.getTorque() * self.gear_ratios[self.gear-1] * self.crank_sprocket_ratio / self.tire_radius
		return force 

# The Car class. This is where all of the calculations are done. Instantiate a Car object and use Car.findDynamicTimes() to test it
class Car:
    def __init__(self, name, m, P, mu, width, a_brake, df, drag):
        self.name = name # Just the name of the instance
        self.mass = m # Mass of the car with fuel and driver, in kilograms
        self.power = P # Average power of the engine, calculated using acceleration event times, in Watts
        self.mu = mu # Coefficient of static friction
        self.a_brake = a_brake # Longitudinal acceleration under peak braking
        self.df = df # A length 3 array with quadratic coefficients
        self.df += findVertex(df[0],df[1],df[2])[::-1]
        self.drag = drag # Same as self.df ... [a, b, c]
        self.drag += findVertex(drag[0],drag[1],drag[2])[::-1]
        self.width = width # Width of the car in meters
        self.dt = 0.001        
    
    def getDownforce(self, velocity):
        # Returns downforce amount (positive) in Newtons given a velocity using a linear/quadratic model of downforce
        # If v is higher than vertex of downforce parabola, use quadratic model of downforce. Otherwise use linear
        if (velocity >= self.df[4]):
            return self.df[0]*velocity**2 + self.df[1]*velocity + self.df[2]
        # If v is lower than the apex, use the linear model
        else:
            return velocity*self.df[3]/self.df[4]

    def getDrag(self, velocity):
        # Returns drag force (positive) in Newtons given a velocity using a linear/quadratic model of drag
        # Basically identical to Car.getDownforce()
        if (velocity >= self.drag[4]):
            return self.drag[0]*velocity**2 - self.drag[1]*velocity + self.drag[2]
        else:
            return velocity*self.drag[3]/self.drag[4]
    
    def findCorneringSpeed(self, radius):
        # Returns the maximum speed in meters per second that the car could go through a corner of given radius
        # Finds speed where centripetal force = cornering force. This formula can be reduced into a quadratic and solved

        radius = radius + 0.5*self.width
        
        a = self.mass/radius - self.mu*self.df[0]
        b = self.mu*self.df[1]
        c = -1*self.mu*(self.mass*9.81 + self.df[2])
        
        v = max(quadraticFormula(a,b,c))

        if v > self.df[4]:
            return v
        else:
            a = 1
            b = -radius/self.mass * self.mu * self.df[3]/self.df[4]
            c = -radius * self.mass * self.mu * 9.81
            return max(quadraticFormula(a,b,c))

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
		a = 0
		v = v_i
		self.drivetrain.selectGear(v_i)
        
        # While d is less than the length of the straight minus the distance required to slow down, accelerate
        while(d < length - self.findBrakingDistance(v, v_f)):
            self.drivetrain.selectGear(v)
			
		#Velocity Verlet
			a_prev = a
			d += v*self.dt + (0.5*a_prev*self.dt**2)
			
            	# Some calculations to figure out how much throttle to use
			max_force = self.drivetrain.getWheelForce() - self.getDrag(v) # The most force that the engine can supply minus drag
			max_traction = self.mu * (self.mass*9.81 + self.getDownforce(v)) # The most friction the tires can supply without slipping
			throttle_percent = max_traction/max_force # Set the 'throttle' so that the car doesn't spin the tires
			if throttle_percent > 1: # Don't let throttle_percent go over 1 or under 0
				throttle_percent = 1
			elif throttle_percent < 0:
				throttle_percent = 0
			
			a = (max_force * throttle_percent) / self.mass
			a_avg = (a + a_prev)/2
			v += a_avg*self.dt
			t += self.dt
			
			#Print stuff for debugging
			print("Time: " + str(t))
			print("RPM: " + str(self.drivetrain.rpm) + "\tPower: " + str(self.drivetrain.getPower()) + "\tTorque: " + str(self.drivetrain.getTorque()) + "\tThrottle: " + str(throttle_percent))
			print("Distance: " + str(d) + "\tVelocity: " + str(v) + "\tAccel: " + str(a_avg) + "\n")
                
        # The previous loop exited, so now it is time to slow down by subtracting the braking acceleration from velocity
        #************This needs to be changed (probably dramatically) to switch to a force based model
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
#power = 23390 # Watts
R62006_curve = [[4125.0, 18642.0], [10000.0, 49216.0]]
drivetrain17 = Drivetrain(R62006_curve, [2.583, 2.0, 1.67, 1.444, 1.286, 1.150], 42.0/13.0, 0.2286)
m_car = 234.0 # kg
m_driver = 75.0 # kg
m_aeroPackage = 12 # kg
width_car = 1.2192 # m
mu_s = 1.31985
a_brake17 = 1.2 * 9.81 # m/s^2
a_brake18 = 1.3 * 9.81 # m/s^2
df18 = [0.2852, -3.3168, 25.136]
drag18 = [0.8715, -11.845, 42.783]
df17 = [-0.2951, 5.9005, -12.486, 0, 0]
drag17 = [0.402, -0.0397, -7.5821]

skidpad_track = [[8.12, 360, 0]]

#!!! Change these paths to match the paths to the track files on your computer
#autox_track = readTrackFile("C:\\Users\\MMcMu\\OneDrive\\Documents\\_School\\_FSAE\\2017 autocross map.txt")
#endurance_track = readTrackFile("C:\\Users\\MMcMu\\OneDrive\\Documents\\_School\\_FSAE\\2017 endurance map.txt")


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TESTING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Mantis17 is the old car
Mantis17 = Car("Mantis 17", m_car+m_driver, drivetrain17, mu_s, width_car, a_brake17, df17, drag17)

print(Mantis17.findStraightTime(0, -1, 75)) # Acceleration event time

# Mantis18 is the old car + aero. 
# Mantis18a is Mantis18 - 16 kg (so we are average weight w/o aero), + 2147 W from new exhaust
#Mantis17 = Car("Mantis 17", m_car+m_driver, power, mu_s, width_car, a_brake17, df17, drag17)
#Mantis18 = Car("Mantis 18", m_car+m_driver+m_aeroPackage, power, mu_s, width_car, a_brake18, df18, drag18)
#Mantis18a = Car("Mantis 18a", m_car+m_driver+m_aeroPackage-16, power+2147, mu_s, width_car, a_brake18, df18, drag18)

# Finding dynamic event times for all of the cars
#m17 = Mantis17.findDynamicTimes(True, autox_track, endurance_track)
#m18 = Mantis18.findDynamicTimes(True, autox_track, endurance_track)
#m18a = Mantis18a.findDynamicTimes(True, autox_track, endurance_track)

#events = ['acceleration', 'skidpad', 'autocross', 'endurance']

# Print differences between 2017 and 2018 cars
#for event in range(len(m17)):
#    diff = m17[event]-m18[event]
#    percent = diff/m17[event]*100
#    print(str(round(percent,2)) + '% difference from 2017 to 2018 in ' + events[event])
#print()

# Print differences between 2017 and 2018a cars
#for event in range(len(m17)):
#    diff = m17[event]-m18a[event]
#    percent = diff/m17[event]*100
#    print(str(round(percent,2)) + '% difference from 2017 to 2018a in ' + events[event])
    
# Calculating the accuracy of the model
#model_accuracy = 0
#irl_times = [4.452, 5.252, 57.268, 1485.083]
#for event in range(len(m17)):
#    print(str(round(abs((m17[event]-irl_times[event])/irl_times[event]*100),2)) + '% difference from irl to model in ' + events[event] + '\n')
#    
#    model_accuracy += abs((m17[event]-irl_times[event])/m17[event]*100)

#model_accuracy /= 4
#print('\nModel is ' + str(model_accuracy)[:4] + '% inaccurate on average across the four events')   
