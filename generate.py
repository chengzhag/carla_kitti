#!/usr/bin/env python3

from __future__ import print_function

import argparse
import logging
import random
import time
import os
import math

from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line


def run_carla_client(args):
    # camera settings
    camcoor_x = 0.27
    camcoor_y = -0.06
    camcoor_z = 1.65
    cambaseline = 0.54
    resolution_w = 1240 * args.scale
    resolution_h = 376 * args.scale
    camfu = 718.856 * args.scale
    camFOV = 2 * math.atan2(resolution_w, 2 * camfu) * 180 / math.pi

    # skip n frames for vehicle to start
    num_frames_skip = 30

    # weather settings
    weathers = [1, 2, 8, 9]
    print('weathers = ')
    print(weathers)

    with make_carla_client(args.host, args.port) as client:
        print('CarlaClient connected')

        # scene = client.load_settings(new_setting())

        # Choose one player start at random.
        number_of_player_starts = 152
        if args.num_starts != 0:
            number_of_episodes = min(number_of_player_starts, args.num_starts)

        print('%d episodes will be generated...' % number_of_episodes)

        startPoints = list(range(0, number_of_player_starts))
        startPoints = random.sample(startPoints, number_of_episodes)

        # Create a CarlaSettings object. This object is a wrapper around
        # the CarlaSettings.ini file. Here we set the configuration we
        # want for the new episode.
        settings = CarlaSettings()
        settings.set(
            SynchronousMode=True,
            SendNonPlayerAgentsInfo=True,
            NumberOfVehicles=40,
            NumberOfPedestrians=0,
            WeatherId=random.choice(weathers),
            QualityLevel=args.quality_level)
        settings.randomize_seeds()

        # Now we want to add a couple of cameras to the player vehicle.
        # We will collect the images produced by these cameras every
        # frame.

        for cameraID in range(2, 4):
            camcoor_y2 = camcoor_y
            # The default camera captures RGB images of the scene.
            cameraRGB = Camera('Camera%dRGB' % cameraID, FOV=camFOV)
            # Set image resolution in pixels.
            cameraRGB.set_image_size(resolution_w, resolution_h)
            # Set its position relative to the car in meters.
            cameraRGB.set_position(camcoor_x, camcoor_y2 if cameraID == 2 else camcoor_y3, camcoor_z)
            settings.add_sensor(cameraRGB)

            # Let's add another camera producing ground-truth depth.
            cameraDepth = Camera('Camera%dDepth' % cameraID, PostProcessing='Depth', FOV=camFOV)
            cameraDepth.set_image_size(resolution_w, resolution_h)
            cameraDepth.set_position(camcoor_x, camcoor_y2 if cameraID == 2 else camcoor_y3, camcoor_z)
            settings.add_sensor(cameraDepth)

            camcoor_y3 = camcoor_y + cambaseline

        for episode, startPoint in enumerate(startPoints):
            settings.set(WeatherId = random.choice(weathers))
            # Start a new episode.
            scene = client.load_settings(settings)
            print('Starting new episode at %r...' % scene.map_name)
            client.start_episode(startPoint)

            tic = time.time()
            # Iterate every frame in the episode.
            for frame in range(0 - num_frames_skip, args.frames_per_episode):

                # Read the data produced by the server this frame.
                measurements, sensor_data = client.read_data()

                # Print some of the measurements.
                print_measurements(measurements)

                if frame >= 0:
                    # Save the images to disk.
                    for name, measurement in sensor_data.items():
                        filename = args.out_filename_format.format(episode, name, frame)
                        measurement.save_to_disk(filename, lambda depth: camfu * cambaseline / depth, 'pfm')

                control = measurements.player_measurements.autopilot_control
                control.steer += random.uniform(-0.02, 0.02)
                client.send_control(control)
                print('time left: %.2f' % ((time.time() - tic) / 3600 * (
                        (args.frames_per_episode - frame)
                        + (args.frames_per_episode + num_frames_skip) * (len(startPoints) - episode - 1)))
                      )
                tic = time.time()


def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.0f} km/h, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} non-player agents in the scene)'
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        speed=player_measurements.forward_speed * 3.6,  # m/s -> km/h
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print(message)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-o', '--output-folder',
        default='carla_kitti',
        type=str,
        help='output folder dir')
    argparser.add_argument(
        '-s', '--scale',
        default=2,
        type=float,
        help='scale of frames')
    argparser.add_argument(
        '-n', '--num_starts',
        default=0,
        type=int,
        help='number of start points (episode)')
    argparser.add_argument(
        '-f', '--frames_per_episode',
        default=100,
        type=int,
        help='number of frames per episode')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = os.path.join(args.output_folder, 'episode_{:0>4d}/{:s}/{:0>6d}')

    while True:
        try:

            run_carla_client(args)

            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
