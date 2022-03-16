import carla
import random


def spawnVehicles(client, world, spawn_points, blueprintsVehicles, number):
    print("Spawning vehicles...")
    batch= []
    for i in range(number):
        spawn_point = random.choice(spawn_points)
        vehicle_bp = random.choice(blueprintsVehicles)
        batch.append(carla.command.SpawnActor(vehicle_bp, spawn_point).then(carla.command.SetAutopilot(carla.command.FutureActor, True)))
    results = client.apply_batch_sync(batch, True)

    all_id = [results[i].actor_id for i in range(len(results))]
    all_actors = world.get_actors(all_id)
    return all_actors, all_id

def spawnWalkers(client, world, blueprintsWalkers, number):
    print("Spawning walkers...")
    # 1. Take all the random locations to spawn
    spawn_points = []
    for i in range(number):
        spawn_point = carla.Transform()
        spawn_point.location = world.get_random_location_from_navigation()
        if (spawn_point.location != None):
            spawn_points.append(spawn_point)

    # 2. Build the batch of commands to spawn the pedestrians
    batch = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    # 2.1 apply the batch
    results = client.apply_batch_sync(batch, True)
    walkers_list=[]
    for i in range(len(results)):
        walkers_list.append({"id": results[i].actor_id})

    # 3. Spawn walker AI controllers for each walker
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

    # 3.1 apply the batch
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        walkers_list[i]["con"] = results[i].actor_id

    # 4. Put altogether the walker and controller ids
    all_id=[]
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.tick()

    # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
    for i in range(0, len(all_actors),2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # random max speed
        all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)
    return all_actors, all_id