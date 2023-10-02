

import argparse


class CommandLineArgsParser:
    """
    CommandLineArgsParser is a utility class for parsing command-line arguments.

    It provides a convenient way to define and parse command-line arguments using argparse.

    Example:
    > python main.py --manual-control --autopilot --host ...

    parser = CommandLineArgsParser()
    args = parser.parse_args()
    print(args)
    """

    def __init__(self):
        self.parser = argparse.ArgumentParser(description=__doc__)
        self.add_arguments()

    def add_arguments(self):
        self.parser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        self.parser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        self.parser.add_argument(
            '-t', '--timeout',
            metavar='T',
            default=10,
            type=int,
            help='Timeout while trying to establish connection to CARLA server')
        self.parser.add_argument(
            '--map',
            metavar='M',
            default="Town10HD_Opt",
            type=str,
            help='Initial map (default: Town10HD_Opt)')
        self.parser.add_argument(
            '-n', '--number-of-vehicles',
            metavar='N',
            default=10,
            type=int,
            help='number of vehicles (default: 10)')
        self.parser.add_argument(
            '-w', '--number-of-walkers',
            metavar='W',
            default=50,
            type=int,
            help='number of walkers (default: 50)')
        self.parser.add_argument(
            '--safe',
            action='store_true',
            help='avoid spawning vehicles prone to accidents')
        self.parser.add_argument(
            '--filterv',
            metavar='PATTERN',
            default='vehicle.*',
            help='vehicles filter (default: "vehicle.*")')
        self.parser.add_argument(
            '--filterw',
            metavar='PATTERN',
            default='walker.pedestrian.*',
            help='pedestrians filter (default: "walker.pedestrian.*")')
        self.parser.add_argument(
            '--tm-port',
            metavar='P',
            default=8000,
            type=int,
            help='port to communicate with TM (default: 8000)')
        self.parser.add_argument(
            '--sync',
            action='store_true',
            help='Synchronous mode execution')
        self.parser.add_argument(
            '--hybrid',
            action='store_true',
            help='Enanble')
        self.parser.add_argument(
            '-s', '--seed',
            metavar='S',
            type=int,
            help='Random device seed')
        self.parser.add_argument(
            '--car-lights-on',
            action='store_true',
            default=False,
            help='Enable car lights')
        self.parser.add_argument(
            '--delta-seconds',
            default=.1,
            type=float,
            help='Delta seconds between frames (default: .1)')
        self.parser.add_argument(
            '--ignore-first-n-ticks',
            default=1,
            help='Ignore first n ticks in simulation (default: 70)')
        self.parser.add_argument(
            '--number-of-ego-vehicles',
            default=1,
            type=int,
            help='The number of ego vehicles in our simulation (default: 1)')
        self.parser.add_argument(
            '--manual-control',
            default=False,
            action='store_true',
            help='Are we manully controlling the ego vehicle using Logitech G29 Racing Wheel? (default: False)')
        self.parser.add_argument(
            '-a', '--autopilot',
            action='store_true',
            help='enable autopilot')
        self.parser.add_argument(
            '--res',
            metavar='WIDTHxHEIGHT',
            default='1280x720',
            help='window resolution (default: 1280x720)')
        self.parser.add_argument(
            '-v', '--verbose',
            action='store_true',
            dest='debug',
            help='print debug information')

    def parse_args(self):
        return self.parser.parse_args()

    def int_within_range(min_val, max_val):
        def validate(value):
            int_value = int(value)
            if min_val <= int_value <= max_val:
                return int_value
            else:
                raise argparse.ArgumentTypeError(
                    f"Value must be between {min_val} and {max_val}")
        return validate
