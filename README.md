## Command-Line Arguments

The following command-line arguments are available to customize the behavior of your application:

- `--host, -H`: Specifies the IP of the host server. Default value is `127.0.0.1`.

- `--port, -p`: Specifies the TCP port to listen to. Default value is `2000`.

- `--timeout, -t`: Sets the timeout (in seconds) while trying to establish a connection to the CARLA server. Default value is `10`.

- `--map, -M`: Sets the initial map for the application. Default value is `"Town03"`.

- `--number-of-vehicles, -n`: Specifies the number of vehicles to spawn. Default value is `10`.

- `--number-of-walkers, -w`: Specifies the number of pedestrians (walkers) to spawn. Default value is `50`.

- `--safe`: When provided, this flag avoids spawning vehicles prone to accidents.

- `--filterv, --filterw`: Specifies filters for vehicles and pedestrians, respectively. Default values are `"vehicle.*"` and `"walker.pedestrian.*"`.

- `--tm-port`: Specifies the port to communicate with TM (Traffic Manager). Default value is `8000`.

- `--sync`: When provided, this flag enables synchronous mode execution.

- `--hybrid`: When provided, this flag enables a hybrid mode.

- `--seed, -s`: Sets the random device seed for reproducibility.

- `--car-lights-on`: When provided, this flag enables car lights.

- `--delta-seconds`: Sets the delta seconds between frames. Default value is `0.1`.

- `--ignore-first-n-ticks`: Ignores the first n ticks in the simulation. Default value is `1`.

- `--manual-control`: When provided, this flag indicates manual control using a Logitech G29 Racing Wheel. Default value is `False`.

- `--autopilot, -a`: When provided, this flag enables autopilot mode.

- `--res`: Sets the window resolution in the format "WIDTHxHEIGHT". Default value is `"1280x720"`.

- `--verbose, -v`: When provided, this flag enables verbose mode (printing debug information).
