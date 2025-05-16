import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import cflib
import numpy as np
import multiprocessing
from multiprocessing import shared_memory
import sys

import socket


uris = ['radio://0/80/2M/7']
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
    print(x,y,z)
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
    elif pos_dict[cf.link_uri][0] >= x:
        x_cond = -1
    if pos_dict[cf.link_uri][1] < y:
        y_cond = 1
    elif pos_dict[cf.link_uri][1] >= y:
        y_cond = -1
    # TO TEST :
    # - Change vx, vy, vz (to see the impact (example 0.01))
    # - send_position_setpoint
    # - if don't work send_full_state_setpoint orientation (quaternions)
    # 
    z = 0.5
    if abs(pos_dict[cf.link_uri][0] - x) < 0.1:
        vx = 0
    if abs(pos_dict[cf.link_uri][1] - y) < 0.1:
        vy = 0
    vx = x_cond * 0.1 
    vy = y_cond * 0.1
    cf.commander.send_velocity_world_setpoint(vx,vy,0,0)
    print(x_cond*pos_dict[cf.link_uri][0], y_cond*pos_dict[cf.link_uri][1])
    print(x_cond*x, y_cond*y)
    print(vx,vy)
    
    sig = pos_dict[cf.link_uri][0]**2 - x**2 > 0
    sig2 = pos_dict[cf.link_uri][1]**2 - y**2 > 0
    while x_cond*pos_dict[cf.link_uri][0] < x_cond*x or y_cond*pos_dict[cf.link_uri][1] < y_cond*y:
        # print("loop")
        # print(pos_dict[cf.link_uri][0]**2 - x**2)
        # print(pos_dict[cf.link_uri][1]**2 - y**2)
        sig = pos_dict[cf.link_uri][0]**2 - x**2 > 0
        sig2 = pos_dict[cf.link_uri][1]**2 - y**2 > 0
        if pos_dict[cf.link_uri][0]**2 - x**2 < 0 == sig:
            vx = -vx
        if pos_dict[cf.link_uri][1]**2 - y**2 < 0 == sig2:
            vy = -vy
        cf.commander.send_velocity_world_setpoint(vx,vy,0,0)
        
        time.sleep(0.05)

    # else:
    #     print(x_cond,y_cond,pos_dict[cf.link_uri][0])
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
    print("end of go_to")
    
    
    
    


def take_off(cf, height):
    #print(swarm.get_estimated_positions())
    take_off_time = 2.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    print(steps)
    vz = height / take_off_time
    print(vz)
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)
    time.sleep(0.5)
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
    print("Takeoff finished")

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
        connected = True
        cf = scf.cf
        global en_cours
        global decollage
        decollage = False
        en_cours = True
        scf.cf.param.set_value('posCtlPid.xVelMax', '1')
        scf.cf.param.set_value('posCtlPid.yVelMax', '1')
        scf.cf.param.set_value('posCtlPid.zVelMax', '1')

        v_0 = 0.35
    
 
            
        float_size = np.float64().nbytes

        take_off(cf, 0.50)
        time.sleep(2)
        #mes = socket_dict[uris[0]].recv(1024).decode()
        
        go_to(cf,0,0,scf)
        
        if connected:
            print("reception")
            mes = socket_dict[uris[0]].sendall("Connected".encode())
        
            x = 0
            y = 0
            mes = socket_dict[uris[0]].recv(1024).decode()
            print(f"Just decoded : {mes}")
            mes = mes.split(";")[0].split(" ") 
            print(f"Splited : {mes}, len = {len(mes)}")
            if len(mes) == 2:
                x,y = mes
            x = float(x)
            y = float(y)
            print(f"Unpacked : x : {x},y : {y}")
            while True:
                mes = socket_dict[uris[0]].recv(1024).decode()
                r = mes
                
                mes = mes.split(";")[0].split(" ") 
                #print(f"Splited : {mes}, len = {len(mes)}")
                if len(mes) == 2:
                    x,y = mes
                x = float(x)
                y = float(y)
                if x**2 + y**2 > 2000000:
                    print(f"Just decoded : {r}")
                    print(f"Splited : {mes}, len = {len(mes)}")
                    print(f"Unpacked : x : {x},y : {y}")
                    break
                
                x = x/600
                y = y/600
                go_to(cf,x,y,scf)
                time.sleep(1)
        time.sleep(0.5)
        land(cf, pos_dict[scf.cf.link_uri])
        time.sleep(1)

    except Exception as e:
        land(cf, pos_dict[scf.cf.link_uri])
        time.sleep(1)
        print(e)

if __name__ == '__main__':
    connected = True
    while True:
        try:
            nb_drone_sim = int(input("Combien de drone dans le simulateur : "))
            break
        except ValueError:
            print("Invalid input! Please enter a valid integer.")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    if connected:
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.bind(("localhost", 3080))
        serversocket.listen(5)
        for i in range(nb_drone_sim if nb_drone_sim > len(uris) else len(uris)):
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn, addr = serversocket.accept()
            socket_dict[uris[i]] = conn


    print('Connecting to Crazyflies...')
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


        