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

def angle_to_quaternion(yaw_rad):
    """Convertit un angle yaw en quaternion [x,y,z,w]"""
    return [0.0, 0.0, np.sin(yaw_rad), np.cos(yaw_rad)]

def go_to1(cf, yaw_radians, scf):
    """Fait tourner le drone à l'angle yaw_radians sans se déplacer"""
    print(f"Orientation à {yaw_radians}°")
    
    quaternion = angle_to_quaternion(yaw_radians)
    
    current_pos = pos_dict[cf.link_uri]
    
    # Durée de la rotation (2 secondes)
    start_time = time.time()
    while time.time() - start_time < 2.0:
        cf.commander.send_full_state_setpoint(
            [current_pos[0], current_pos[1], current_pos[2]],  # Position actuelle
            [0, 0, 0],      # Vitesse nulle
            [0, 0, 0],      # Accélération nulle
            quaternion,     # Nouvelle orientation
            0, 0, 0         # Vitesse angulaire nulle
        )
        time.sleep(0.05)
    
    # Stabilisation finale
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)

def pixel_to_meter(points_pixel):
    # Plage du simulateur (pixels)
    pixel_bounds = np.array([[-400, 400], [-300, 300]])  # [x_min, x_max], [y_min, y_max]
    
    # Plage du monde réel (mètres)
    meter_bounds = np.array([[0, 1.5], [0.5, -0.5]])  # [x_min, x_max], [y_min, y_max]
    
    # Normalisation et mise à l'échelle
    scale = (meter_bounds[:, 1] - meter_bounds[:, 0]) / (pixel_bounds[:, 1] - pixel_bounds[:, 0]) #facteur de conversion
    offset = [-1,0] #coordonnées de l'origine
    

    return points_pixel * scale + offset

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
        while keyboard.is_pressed('q') == False:
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
        serversocket.bind(("192.168.194.211", 3080))
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


        