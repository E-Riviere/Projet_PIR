import time
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.high_level_commander import HighLevelCommander
import cflib
import numpy as np
import keyboard
import socket


uris = ['radio://0/80/2M/A5']
        #'radio://0/80/2M/9',
        #'radio://0/80/2M/A1']
pos_dict = {}
vel_dict = {}
socket_dict = {}
file_dict = {}
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
    file_dict[uri].write(f"{int(time.time()*1000)};{x};{y};{z};{vx};{vy};{vz}\n")
    

def print_check(scf):
    print(f"Connected to {scf.cf.link_uri}")


def fly_sequence(scf):
    file_dict[scf.cf.link_uri] = open(scf.cf.link_uri.split("/")[-1],"w")

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
    file_dict[scf.cf.link_uri] = open(scf.cf.link_uri.split("/")[-1],"w")
    drone = HighLevelCommander(cf)
        
    float_size = np.float64().nbytes

    drone.takeoff(0.5,1)
    time.sleep(2)
    #mes = socket_dict[uris[0]].recv(1024).decode()
    
    drone.go_to(-1,0,0.5,0,1)
    time.sleep(1)
    if connected:
        print("reception")
        mes = socket_dict[uris[0]].sendall("Connected".encode())
    
        x = 0
        y = 0
        mes = socket_dict[uris[0]].recv(1024).decode()
        print(f"Just decoded : {mes}")
        mes = mes.split(";")[0].split(" ") 
        print(f"Splited : {mes}, len = {len(mes)}")
        if len(mes) == 3:
            x,y,yaw = mes
        x = float(x)
        y = float(y)
        yaw = float(yaw)
        print(f"Unpacked : x : {x},y : {y},yaw : {yaw}")
        while True:
            mes = socket_dict[uris[0]].recv(1024).decode()
            r = mes
            
            mes = mes.split(";")[0].split(" ") 
            #print(f"Splited : {mes}, len = {len(mes)}")
            if len(mes) == 3:
                x,y,yaw = mes
            x = float(x)
            y = float(y)
            yaw = float(yaw)
            if x**2 + y**2 > 2000000:
                print(f"Just decoded : {r}")
                print(f"Splited : {mes}, len = {len(mes)}")
                print(f"Unpacked : x : {x},y : {y},yaw : {yaw}")
                break
            
            #point_meter = pixel_to_meter((x,y))
            x, y = x/400, y/300
            print(x,y,yaw)
            drone.go_to(x,y,0.5,yaw,0.5)
            time.sleep(0.7)
    time.sleep(0.5)
    drone.land(0,1)
    time.sleep(1)

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


        