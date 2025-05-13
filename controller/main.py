import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
import cflib
import numpy as np
import multiprocessing
from multiprocessing import shared_memory
import sys

import socket


uris = ['radio://0/80/2M/A8']
        #'radio://0/80/2M/9',
        #'radio://0/80/2M/A1']
pos_dict = {}
vel_dict = {}
socket_dict = {}
global k
k = 0
def start_states_log(scf):
    log_conf = LogConfig(name='States', period_in_ms=50)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')

    log_conf.add_variable('stateEstimate.vx', 'float')
    log_conf.add_variable('stateEstimate.vy', 'float')
    log_conf.add_variable('stateEstimate.vz', 'float')
    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: log_callback(uri, timestamp, data, logconf))
    log_conf.start()

def log_callback(uri, timestamp, data, logconf):
    global k
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    pos = np.array([x, y, z])
    pos_dict[uri] = pos
    k += 1
    if k%100 == 0:
        print(x,y,z)
    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    vz = data['stateEstimate.vz']
    vel = np.array([vx, vy, vz])
    vel_dict[uri] = vel


def go_to(cf, x, y,scf):
    print(f"going to {x},{y}")
    x_cond = 0
    if pos_dict[cf.link_uri][0] < x:
        x_cond = 1
    elif pos_dict[cf.link_uri][0] > x:
        x_cond = -1
    if pos_dict[cf.link_uri][1] < y:
        y_cond = 1
    elif pos_dict[cf.link_uri][1] > y:
        y_cond = -1
    # TO TEST :
    # - Change vx, vy, vz (to see the impact (example 0.01))
    # - send_position_setpoint
    # - if don't work send_full_state_setpoint orientation (quaternions)
    # 
    cf.commander.send_full_state_setpoint([x,y,0.5],[0.5,0.5,0.5],[0,0,0],[0,0,0,1],0,0,0)
    while x_cond*pos_dict[cf.link_uri][0] < x_cond*x or y_cond*pos_dict[cf.link_uri][1] < y_cond*y:
        cf.commander.send_full_state_setpoint([x,y,0.5],[0.5,0.5,0.5],[0,0,0],[0,0,0,1],0,0,0)
        time.sleep(0.05)

    # else:
    #     print(x_cond,y_cond,pos_dict[cf.link_uri][0])
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
    
    
    


def take_off(cf, height):
    #print(swarm.get_estimated_positions())
    
    take_off_time = 2.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = height / take_off_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    print('landing...')
    landing_time = 4.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)
    print("coucou landing")
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)

def turn(cf):
    print("turning...")
    turn_time = 8.0
    sleep_time = 0.1
    steps = int(turn_time / sleep_time)
    yawrate = 360 / turn_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, 0, yawrate)
        time.sleep(sleep_time)

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(1)
    deactivate_led_bit_mask(scf)
    time.sleep(0.5)

def print_check(scf):
    print(f"Connected to {scf.cf.link_uri}")

def fly_sequence(scf):
    try:
        cf = scf.cf
        global en_cours
        global decollage
        decollage = False
        en_cours = True

        v_0 = 0.35
        mes = socket_dict[uris[0]].sendall("Connected".encode())
 
            
        float_size = np.float64().nbytes

        take_off(cf, 0.50)
        mes = socket_dict[uris[0]].recv(1024).decode()
        go_to(cf,0,0,scf)
        time.sleep(0.5)

        x =-1
        print("crash")
        while x < -0.210 or True:
            mes = socket_dict[uris[0]].recv(1024).decode()
            x,y = mes.split(";")[0].split(" ")
            x = float(x)
            y = float(y)
            print('Connecting to Crazyflies...')
            print(x,y)
            x = x/1000
            y = y/1000
            go_to(cf,x,y,scf)
            time.sleep(0.1)

        land(cf, pos_dict[scf.cf.link_uri])
        time.sleep(1)

    except Exception as e:
        land(cf, pos_dict[scf.cf.link_uri])
        time.sleep(1)
        print(e)

if __name__ == '__main__':
    while True:
        try:
            nb_drone_sim = int(input("Combien de drone dans le simulateur : "))
            break
        except ValueError:
            print("Invalid input! Please enter a valid integer.")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind(("localhost", 3080))
    serversocket.listen(5)
    for i in range(nb_drone_sim if nb_drone_sim > len(uris) else len(uris)):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn, addr = serversocket.accept()
        socket_dict[uris[i]] = conn
        

        
        
    with Swarm(uris, factory=factory) as swarm:
        
        swarm.parallel_safe(print_check)
        print('Connected to  Crazyflies')

        
        # print('Performing light check')
        # swarm.parallel_safe(light_check)
        # print('Light check done')
        swarm.parallel_safe(print_check)

        time.sleep(0.5)
        print('Reseting estimators')
        swarm.reset_estimators()
        
        print('Estimators have been reset')

        swarm.parallel_safe(start_states_log)
        print('Logging states info...')

        print('Lets fly ! Put your protection glasses on')
        swarm.parallel_safe(fly_sequence)


        