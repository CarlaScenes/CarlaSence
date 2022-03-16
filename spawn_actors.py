import carla
from carla import Transform, Location, Rotation
import random

town_map = "Town02"


def main():
    #Connect and load map
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    avail_maps = client.get_available_maps()
    world = client.load_world(town_map)

    settings = world.get_settings()
    print(settings.fixed_delta_seconds)
    print(settings.synchronous_mode)
    
    #Get all required blueprints
    blueprint_library = world.get_blueprint_library()
    blueprintsVehicles = blueprint_library.filter('vehicle.*')
    vehicles_spawn_points = world.get_map().get_spawn_points()

    actor1 = world.spawn_actor(random.choice(blueprintsVehicles), random.choice(vehicles_spawn_points))
    actor1.set_autopilot(True)

    actor2 = world.spawn_actor(random.choice(blueprintsVehicles), random.choice(vehicles_spawn_points))
    actor2.set_autopilot(True)

    actor3 = world.spawn_actor(random.choice(blueprintsVehicles), random.choice(vehicles_spawn_points))
    actor3.set_autopilot(True)

    #Have spectator move to actor1, so we don't have to search for him
    spectator = world.get_spectator()
    transform = actor1.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
    carla.Rotation(pitch=-90)))

    while True:
        world.tick()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
