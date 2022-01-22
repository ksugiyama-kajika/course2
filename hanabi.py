print("start HANABI")
from dronekit import LocationGlobal, LocationGlobalRelative, VehicleMode, connect
import time 
import math
from pymavlink import mavutil

all = [ 'tcp:127.0.0.1:5772',
        'tcp:127.0.0.1:5782',
        'tcp:127.0.0.1:5792',
        'tcp:127.0.0.1:5802',
        'tcp:127.0.0.1:5812',
        ]

vehicle_list = []
home_locations = [[36.040958,138.101631,759.7],
                  [36.040806,138.101943,759.8],
                  [36.040467,138.101894,759.7],
                  [36.040472,138.101449,759.6],
                  [36.040775,138.101293,759.7]]

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

i = 0
while len(all) > i:
        vehicle = connect(all[i], wait_ready=True, timeout=60)
        vehicle_list.append(vehicle)
        print(all[i])
        time.sleep(2)
        i = i + 1
print(vehicle_list)

i = 0
while len(all) > i:
        vehicle_list[i].home_location = LocationGlobal(*home_locations[i])
        vehicle_list[i].wait_for_mode("GUIDED") 
        print(str(i+1),"号","home_location: %s" % vehicle_list[i].home_location)
        i += 1

i = 0
targetAltitude = 5
while len(vehicle_list) > i:
        vehicle = vehicle_list[i]
        while not vehicle.is_armable:
                print("Waiting for vehicle to initialize...")
                time.sleep(1)
        vehicle.parameters['DISARM_DELAY'] = 30
        vehicle.arm()        
        print("Take off vehicle %s" % str(i+1))
        vehicle.simple_takeoff(targetAltitude)
        time.sleep(1)
        while True:
                print("Altitude: ", vehicle.location.global_relative_frame.alt)
                if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
                        print("Reached Target altitude")
                        break
                time.sleep(1)   
        alocation = LocationGlobalRelative(*home_locations[i]) 
        alocation.alt = 20
        vehicle.simple_goto(alocation)
        time.sleep(1)
        i += 1
time.sleep(10)
print("全機配置完了")

# launch Hanabi
print("打ち上げ開始")

direction_deg = -40
turn_left = direction_deg - 60
turn_right = direction_deg + 60

msg_left = vehicle.message_factory.set_attitude_target_encode(
    0,
    0,0,
    0b00000000 if False else 0b00000100,
    to_quaternion(0,-40,turn_left),
    0,0,math.radians(0),
    0.5)

msg_right = vehicle.message_factory.set_attitude_target_encode(
    0,
    0,0,
    0b00000000 if False else 0b00000100,
    to_quaternion(0,-40,turn_right),
    0,0,math.radians(0),
    0.5)

msg_stop = vehicle.message_factory.set_attitude_target_encode(
    0,
    0,0,
    0b00000000 if False else 0b00000100,
    to_quaternion(0,0,direction_deg),
    0,0,math.radians(0),
    0.5)

turn_left = 0 
turn_right = 0

for times in range(1,11):
    if times % 2 != 0:
        turn_left += 1
        print("斜め左移動", turn_left, "回目") 
        for x in range(0, 50):
                i = 0
                while len(vehicle_list) > i:
                        vehicle_list[i].send_mavlink(msg_left)
                        i += 1            
                time.sleep(0.1)
        continue 
    turn_right += 1
    print("斜め右移動", turn_right, "回目")
    for x in range(0, 50):
            i = 0
            while len(vehicle_list) > i:
                    vehicle_list[i].send_mavlink(msg_right)
                    i += 1
            time.sleep(0.1)

print("打ち上げ完了") 

### stop vehicle
for x in range(0, 30):
        i = 0
        while len(vehicle_list) > i:
                vehicle_list[i].send_mavlink(msg_stop)
                i += 1
        time.sleep(0.1)

### burst Hanabi
print("花火破裂")
spread_directions = [288, 0, 72, 144, 216]
msg_list = []
i = 0
while len(spread_directions) > i:
        msg_spread = vehicle.message_factory.set_attitude_target_encode(
                0,
                0,0,
                0b00000000 if False else 0b00000100,
                to_quaternion(0,-60,spread_directions[i]),
                0,0,math.radians(0),
                0.5)
        msg_list.append(msg_spread)
        i = i + 1

for x in range(0, 100):
        i = 0
        while len(vehicle_list) > i:
                vehicle_list[i].send_mavlink(msg_list[i])
                i += 1
        time.sleep(0.1)
time.sleep(10)

### Change to RTL
i=0
while len(vehicle_list) > i:
        vehicle_list[i].wait_for_mode("RTL") 
        print(str(i+1),"号帰還開始")
        i += 1
time.sleep(10)

i = 0
while len(vehicle_list) > i:
        while True:
                if (vehicle_list[i].airspeed < 0.1) & (vehicle_list[i].groundspeed < 0.1) & (vehicle_list[i].location.global_relative_frame.alt < 0.1):
                        print(str(i+1),"号帰還")
                        break
                time.sleep(1)
        i += 1

print("演目終了")