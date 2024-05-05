import asyncio
import math
import time
from multiprocessing import Manager
from mavsdk import System
import json
from datetime import datetime
from geopy import Point
from geopy.distance import geodesic
from mavsdk.offboard import (PositionNedYaw, VelocityNedYaw, OffboardError)

def in_file(a):
    log[0] = a

def json_open(i):
    if i == 0:
        rb[0] = 'mn'
    with open('coords.plan') as f:
        file_content = f.read()
        if file_content:
            templates = json.loads(file_content)
    if templates['mission']['items'][i]['params'][4] != 0:
        print(i)
        lat = templates['mission']['items'][i]['params'][4]
        long = templates['mission']['items'][i]['params'][5]
        hight = templates['mission']['items'][i]['params'][6]
        i += 1
        read_json[0] = i
        start_pos[2] = lat
        start_pos[3] = long
        print('go', lat, long, hight, str(4))
        in_file(f'go {lat} {long} {hight} {str(4)}')

    else:
        in_file('0 0 0 0')

async def run():
    loop = asyncio.get_event_loop()
    json_open(0)
    while True:
        command = await loop.run_in_executor(None, input, )
        in_file(command + ' 0 0 0 0')

def from_file(name):
    file = open(name, 'r')
    source = file.readline()
    file.close()
    return source

async def check_gps_pos_lat(drone):
    async for pos_gps in drone.telemetry.position():
        a = pos_gps.latitude_deg
        return a

async def check_gps_pos_lon(drone):
    async for pos_gps in drone.telemetry.position():
        b = pos_gps.longitude_deg
        return b

async def straight_chech(x1,y1,x2,y2,x3,y3):
    condition = ((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))
    if -0.3<= condition <=0.3:
        print('One one straight')

    else:
        print('pryamaya =',condition)

    return condition

async def check_pos(drone):
    try:
        async for position_body in drone.telemetry.odometry():
            s = position_body.position_body
            time.sleep(0.001)
            return s
            break
    except:
        print("NENENENE")

async def compas(drone):
    async for gradus in drone.telemetry.heading():
        x = gradus.heading_deg
        return x

def in_iner(angle, distance,x_m,y_m, flag):
    x = 0
    y = 0
    ox = 1
    oy = 1
    angle_const = angle
    print('angle', angle)
    if 90 < angle < 180:
        angle = angle-90
        oy=-1

    if 180 < angle < 270:
        angle = angle-180
        ox = -1
        oy = -1
    if 270 < angle < 360:
        angle = angle - 270
        ox = -1
    y = (distance * round(math.sin(math.radians(angle)),15)+y_m)*oy
    x = (distance * round(math.cos(angle*math.pi/180),15)+x_m)*ox
    print(f'cos {angle} = {math.cos(math.radians(angle))}')
    print('coodrs', y, x, distance)
    if flag == 0:
        in_local['x'] = x
        in_local['y'] = y
        in_local['azimut'] = angle 
    if flag == 1:
        if 0 < angle_const<90:
            in_local['zero_x'] = y
            in_local['zero_y'] = x
        if 90 < angle_const < 180:
            in_local['zero_x'] = x
            in_local['zero_y'] = y

        if 180 < angle_const < 270:
            in_local['zero_x'] = y
            in_local['zero_y'] = x
        if 270 < angle_const < 360:
            in_local['zero_x'] = x
            in_local['zero_y'] = y

def vector_in(llat1, llong1, llat2, llong2, flag):
    rad = 6372795

    lat1 = llat1 * math.pi / 180.
    lat2 = llat2 * math.pi / 180.
    long1 = llong1 * math.pi / 180.
    long2 = llong2 * math.pi / 180.

    cl1 = math.cos(lat1)
    cl2 = math.cos(lat2)
    sl1 = math.sin(lat1)
    sl2 = math.sin(lat2)
    delta = long2 - long1
    cdelta = math.cos(delta)
    sdelta = math.sin(delta)

    y = math.sqrt(math.pow(cl2 * sdelta, 2) + math.pow(cl1 * sl2 - sl1 * cl2 * cdelta, 2))
    x = sl1 * sl2 + cl1 * cl2 * cdelta
    ad = math.atan2(y, x)
    dist = ad * rad

    x = (cl1 * sl2) - (sl1 * cl2 * cdelta)
    y = sdelta * cl2
    z = math.degrees(math.atan(-y / x))

    if (x < 0):
        z = z + 180.

    z2 = (z + 180.) % 360. - 180.
    z2 = - math.radians(z2)
    anglerad2 = z2 - ((2 * math.pi) * math.floor((z2 / (2 * math.pi))))
    angledeg = (anglerad2 * 180.) / math.pi
    print(round(dist))
    print(
        'Distance >> %.0f' % dist, ' [meters]')
    print(

        'Initial bearing >> ', angledeg, '[degrees]'
    )
    if flag == 1:
        lenght_to_position[0] = round(dist)
        in_file(f'rot0 {round(angledeg)}')
    if flag == 0:
        in_local[0] = round(dist)
        in_local[1] = round(angledeg)
    if flag == 2:
        in_local['dist_to_point'] = round(dist)
        in_local['angle_to_point'] = round(angledeg)

def from_local_in_gps(dist, azimut):
    distm = dist/1000
    lat1 = start_pos[4]
    lon1 = start_pos[5]

    gps = geodesic(kilometers=distm).destination(Point(lat1, lon1), azimut).format_decimal().replace(',','')
    latlong = gps.split()
    print(latlong)

def pythagoras(x,y):

    return math.sqrt((x-in_local['x_now'])**2+(y-in_local['y_now'])**2)

async def engine_work(drone):
    counter = 0

    time_to = 0
    while True:
        await asyncio.sleep(0.1)

        try:

            position = await check_pos(drone)
            local_distantion = abs(pythagoras(position.y_m, position.x_m))
            gr = await compas(drone)
            num = log[0].split()
            if num[0] == 'rot':
                await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0.4))
                print(gr)
            if  num[0] != 'off' and num[0] != 'timer' and num[0] == 'rot0':

                in_file(f'timer {num[1]}')
            if num[0] == 'timer':

                start_pos['x0'] = position.x_m
                start_pos['y0'] = position.y_m
                print('num = ',num[1])
                vector_in(start_pos[0],start_pos[1],start_pos[2],start_pos[3], 0)
                vector_in(start_pos[4],start_pos[5],start_pos[2],start_pos[3], 2)
                print(in_local['dist_to_point'], in_local['angle_to_point'])

                in_iner(in_local[1], in_local[0],position.x_m,position.y_m, 0)
                in_iner(in_local['angle_to_point'], in_local['dist_to_point'], 0, 0, 1)
                await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
                print('<><><><><>', local_distantion, in_local[0])
                in_file('off')
            if num[0] == 'off':
                num[0]='off'

                if round(position.y_m) != round(in_local['y']):
                    stard_data = to_vector[0].split()

                    print(position.x_m,position.y_m, in_local['zero_y'], in_local['zero_x'])
                    await drone.offboard.set_position_ned(PositionNedYaw(in_local['zero_y'], in_local['zero_x'], -float(stard_data[3]), in_local[1]))

                if in_local['zero_x']-1 < position.y_m < in_local['zero_x']+1:
                    print('hello1')  
                    print(position.x_m, position.y_m)
                    in_local['x_now'] = position.y_m
                    in_local['y_now'] = position.x_m
                    in_file('rb')
            if num[0] == 'rb':
                next_pos_lat = start_pos[2]
                next_pos_long = start_pos[3]
                print('next = ', next_pos_lat,' ', next_pos_long, 'now = ', start_pos[0],start_pos[1])
                start_pos[0] = next_pos_lat
                start_pos[1] = next_pos_long
                print(read_json[0])
                json_open(read_json[0])
            if num[0] == 'rb0':
                if gr > 4:
                    await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0.4))
                else:
                    in_file('0 0 0 0')
            if num[0] == 'cord':
                print(position)
                await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0))
            if num[0] == 'go':
                speed[0] = float(float(num[4]) / 10)
                pos = 'go1' + " " + str(num[1]) + " " + str(num[2]) + " " + str(num[3])
                in_file(pos)
            if num[0] == 'go1':
                to_vector[0] = log[0]
                print('go1=', start_pos[0], start_pos[1])
                vector_in(float(start_pos[0]), float(start_pos[1]), float(num[1]), float(num[2]), 1)

            if num[0] == '0' or num[0] == 'go0':
                await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0))

        except Exception as e:
            print(e)
            print("Вы не ввели длительность полёта. Введите значение в метрах")
            in_file('0 0 0 0 0')

async def connect(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

async def start():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    await connect(drone)
    await drone.manual_control.set_manual_control_input(0, 0, 0, 0)
    await asyncio.sleep(2)
    try:
        if drone.telemetry.armed():
            await drone.action.arm()

        else:
            print("arm")
    except:
        pass
    print("Arm!")
    time.sleep(2)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    print("-- Starting offboard")

    await drone.offboard.start()
    return drone

if __name__ == '__main__':
    manager = Manager()
    log = manager.dict()
    rb = manager.dict()
    to_vector = manager.dict()
    in_local = manager.dict()
    read_json = manager.dict()
    read_json[0] = 0
    strainght = manager.dict()
    start_pos = manager.dict()
    lenght_to_position = manager.dict()
    start_pos[0] = 47.3977418
    start_pos[1] = 8.5455939
    start_pos[4] = 47.3977418
    start_pos[5] = 8.5455939
    start_pos['x0'] = 0
    start_pos['y0'] = 0
    in_local['x_now'] = 0
    in_local['y_now'] = 0  
    speed = manager.dict()
    loop = asyncio.get_event_loop()
    drone = loop.run_until_complete(start())
    main_task = asyncio.wait([run(), engine_work(drone)])

    loop.run_until_complete(main_task)

    loop.close()