import carla
from carla import Transform

from datadescriptor import KittiDescriptor
from camera_utils import *
# from examples.client_bounding_boxes import ClientSideBoundingBoxes

import math
import logging

OCCLUDED_VERTEX_COLOR = (255, 0, 0)
VISIBLE_VERTEX_COLOR = (0, 255, 0)
MIN_VISIBLE_VERTICES_FOR_RENDER = 4
MIN_BBOX_AREA_IN_PX = 100


class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_bounding_boxes(vehicles, camera, h,w,fov):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """
        bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box(vehicle, camera, h,w,fov) for vehicle in vehicles]
        # filter objects behind camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        return bounding_boxes

    @staticmethod
    def get_bounding_box(vehicle, camera, h,w,fov):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
        
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        
        calibration = np.identity(3)
        calibration[0, 2] = w / 2.0
        calibration[1, 2] = h / 2.0
        calibration[0, 0] = calibration[1, 1] = w / (2.0 * np.tan(fov * np.pi / 360.0))

        bbox = np.transpose(np.dot(calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox
    


    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords
    
    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix
    
    @staticmethod
    def get_bounding_boxes_parked_vehicles(bboxes, camera, h,w,fov):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """
        bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box_parked_vehicle(vehicle, camera, h,w,fov) for vehicle in bboxes]
        # filter objects behind camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        return bounding_boxes
    
    @staticmethod
    def get_bounding_box_parked_vehicle(bbox, camera, h,w,fov):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """
        bb_cords = ClientSideBoundingBoxes._bounding_box_to_world(bbox)
        cords_x_y_z = ClientSideBoundingBoxes._world_to_sensor(bb_cords, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        calibration = np.identity(3)
        calibration[0, 2] = w / 2.0
        calibration[1, 2] = h / 2.0
        calibration[0, 0] = calibration[1, 1] = w / (2.0 * np.tan(fov * np.pi / 360.0))
        bbox = np.transpose(np.dot(calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox
    
    @staticmethod
    def _bounding_box_to_world(bbox):
        extent = bbox.extent
        cords = np.zeros((8, 4))
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])

        world_matrix = ClientSideBoundingBoxes.get_matrix(bbox)

        world_cords = np.dot(world_matrix, np.transpose(cords))

        return world_cords
    
    @staticmethod
    def _create_bb_points_parked(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        if isinstance(vehicle, carla.BoundingBox):
            extent = vehicle.extent
        else:
            extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords



# TODO Make computations faster by vectorization
def calculate_occlusion_stats(image, bbox_points, depth_map, max_render_depth, draw_vertices=True):
    """ Draws each vertex in vertices_pos2d if it is in front of the camera
        The color is based on whether the object is occluded or not.
        Returns the number of visible vertices and the number of vertices outside the camera.
    """
    num_visible_vertices = 0
    num_vertices_outside_camera = 0

    for i in range(len(bbox_points)):
        x_2d = bbox_points[i, 0]
        y_2d = bbox_points[i, 1]
        point_depth = bbox_points[i, 2]

        # if the point is in front of the camera but not too far away
        if max_render_depth > point_depth > 0 and point_in_canvas((y_2d, x_2d)):
            is_occluded = point_is_occluded(
                (y_2d, x_2d), point_depth, depth_map)
            if is_occluded:
                vertex_color = OCCLUDED_VERTEX_COLOR
            else:
                num_visible_vertices += 1
                vertex_color = VISIBLE_VERTEX_COLOR
            if draw_vertices:
                draw_rect(image, (y_2d, x_2d), 4, vertex_color)
        else:
            num_vertices_outside_camera += 1
    return num_visible_vertices, num_vertices_outside_camera


def get_bounding_box_and_refpoint(agent, camera, camera_calibration):
    """
    An extended version of Carla get_bounding_box() method, where the reference point of the bbox is also
    concatenated with the bbox vertices to boost the performance as all vertices and refpoint are processed in parallel.
    Returns 3D bounding box and its reference point for a agent based on camera view.
    """
    bbox_refpoint = np.array([[0, 0, 0, 1]], dtype=np.float)
    bb_cords = ClientSideBoundingBoxes._create_bb_points(agent)
    bb_cords_and_refpoint = np.vstack((bb_cords, bbox_refpoint))

    cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords_and_refpoint, agent, camera)[:3, :]
    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
    bbox_and_refpoint = np.transpose(np.dot(camera_calibration, cords_y_minus_z_x))
    camera_bbox_refpoint = np.concatenate([bbox_and_refpoint[:, 0] / bbox_and_refpoint[:, 2], bbox_and_refpoint[:, 1] / bbox_and_refpoint[:, 2], bbox_and_refpoint[:, 2]], axis=1)

    sensor_bbox_refpoint = np.transpose(cords_x_y_z)

    camera_bbox = camera_bbox_refpoint[:-1, :]
    camera_refpoint = np.squeeze(np.asarray(camera_bbox_refpoint[-1, :]))
    sensor_bbox = sensor_bbox_refpoint[:-1, :]
    sensor_refpoint = np.squeeze(np.asarray(sensor_bbox_refpoint[-1, :]))

    return (camera_bbox, camera_refpoint), (sensor_bbox, sensor_refpoint)


def create_kitti_datapoint(agent, camera, cam_calibration, image, depth_map, player_transform, bb, max_render_depth=1000):
    """
    Calculates the bounding box of the given agent, and
    returns a KittiDescriptor which describes the object to be labeled
    """

    obj_type, agent_transform, bbox_transform, ext, location = transforms_from_agent(agent)

    if obj_type is None:
        logging.warning(
            "Could not get bounding box for agent. Object type is None")
        return image, None

    (camera_bbox, camera_refpoint), (sensor_bbox, sensor_refpoint) = get_bounding_box_and_refpoint(agent, camera, cam_calibration)
    
    num_visible_vertices, num_vertices_outside_camera = calculate_occlusion_stats(image,
                                                                                  bb,
                                                                                  depth_map,
                                                                                  max_render_depth,
                                                                                  draw_vertices=False)

    # At least N vertices has to be visible in order to draw bbox
    if num_visible_vertices >= MIN_VISIBLE_VERTICES_FOR_RENDER > num_vertices_outside_camera:

        # TODO I checked for pedestrians and it works. Test for vehicles too!
        # Visualize midpoint for agents
        # draw_rect(image, (camera_refpoint[1], camera_refpoint[0]), 4)
        uncropped_bbox_2d = calc_projected_2d_bbox(camera_bbox)
        
        # Crop vertices outside camera to image edges
        crop_boxes_in_canvas(camera_bbox)

        bbox_2d = calc_projected_2d_bbox(camera_bbox)

        area = calc_bbox2d_area(bbox_2d)
        if area < MIN_BBOX_AREA_IN_PX:
            logging.info("Filtered out bbox with too low area {}".format(area))
            return image, None, None

        occlusion = calculate_occlusion(camera_bbox, agent, depth_map)
        rotation_y = get_relative_rotation_y(agent, player_transform)
        alpha = get_alpha(agent, player_transform)
        truncation = calculate_truncation(uncropped_bbox_2d, bbox_2d)
        datapoint = KittiDescriptor()
        datapoint.set_type(obj_type)
        datapoint.set_bbox(bbox_2d)
        datapoint.set_3d_object_dimensions(ext)
        datapoint.set_3d_object_location(sensor_refpoint)
        datapoint.set_rotation_y(rotation_y)
        datapoint.set_alpha(alpha)
        datapoint.set_truncated(truncation)
        datapoint.set_occlusion(occlusion)
        datapoint.set_object_id(agent.id)

        return image, datapoint, camera_bbox
    else:
        return image, None, None


def calculate_occlusion(bbox, agent, depth_map):
    """Calculate the occlusion value of a 2D bounding box.
    Iterate through each point (pixel) in the bounding box and declare it occluded only
    if the 4 surroinding points (pixels) are closer to the camera (by using the help of depth map)
    than the actual distance to the middle of the 3D bounding boxe and some margin (the extent of the object)
    """
    bbox_3d_mid = np.mean(bbox[:,2])
    min_x, min_y, max_x, max_y = calc_projected_2d_bbox(bbox)
    height, width, length = agent.bounding_box.extent.z, agent.bounding_box.extent.x, agent.bounding_box.extent.y

    #depth_margin should depend on the rotation of the object but this solution works fine
    depth_margin = np.max([2 * width, 2 * length])
    is_occluded = []

    for x in range(int(min_x), int(max_x)):
        for y in range(int(min_y), int(max_y)):
            is_occluded.append(point_is_occluded(
                (y, x), bbox_3d_mid - depth_margin, depth_map))

    occlusion = ((float(np.sum(is_occluded))) / ((max_x-min_x) * (max_y-min_y)))

    #discretize the 0–1 occlusion value into KITTI’s {0,1,2,3} labels by equally dividing the interval into 4 parts
    occlusion = np.digitize(occlusion, bins=[0.25, 0.50, 0.75])

    return occlusion

def calculate_truncation(uncropped_bbox, cropped_bbox):
    "Calculate how much of the object’s 2D uncropped bounding box is outside the image boundary"

    area_cropped = calc_bbox2d_area(cropped_bbox)
    area_uncropped = calc_bbox2d_area(uncropped_bbox)
    truncation = 1.0 - float(area_cropped / area_uncropped)
    return truncation

def get_relative_rotation_y(agent, player_transform):
    """ Returns the relative rotation of the agent to the camera in yaw
    The relative rotation is the difference between the camera rotation (on car) and the agent rotation"""

    rot_agent = agent.get_transform().rotation.yaw
    rot_vehicle = player_transform.rotation.yaw
    #rotate by -90 to match kitti
    rel_angle = math.radians(rot_agent - rot_vehicle - 90)
    if rel_angle > math.pi:
        rel_angle = rel_angle - 2 * math.pi
    elif rel_angle < - math.pi:
        rel_angle = rel_angle + 2 * math.pi
    return rel_angle

def get_alpha(agent, player_transform):

    #heading direction of the vehicle
    forward_vector = player_transform.rotation.get_forward_vector()
    forward_vector_numpy = np.array([forward_vector.x, forward_vector.y, forward_vector.z])
    #location of vehicle
    vehicle_location = player_transform.location
    #location of agent
    agent_location = agent.get_transform().location
    #vector from vehicle to agent
    target_vector = agent_location - vehicle_location
    target_vector_numpy = np.array([target_vector.x, target_vector.y, target_vector.z])
    norm_target = np.linalg.norm(target_vector_numpy)
    #fix rounding errors of dot product (can only range between -1 and 1 for normalized vectors)
    dot_prod = np.dot(forward_vector_numpy, target_vector_numpy) / norm_target
    if dot_prod > 1:
        dot_prod = 1.0
    elif dot_prod < -1:
        dot_prod = -1.0

    # check https://github.com/traveller59/second.pytorch/issues/98
    # and https://arxiv.org/pdf/1904.12681.pdf (supplementary material), here theta = theta ray
    theta = math.degrees(math.acos(dot_prod))
    rot_agent = agent.get_transform().rotation.yaw
    rot_vehicle = player_transform.rotation.yaw
    # rotate by -90 to match kitti
    rel_angle = rot_agent - rot_vehicle - 90

    alpha = math.radians(rel_angle - theta)

    if alpha > math.pi:
        alpha = alpha - 2*math.pi
    elif alpha < - math.pi:
        alpha = alpha + 2*math.pi

    return alpha

def transforms_from_agent(agent):
    """ Returns the KITTI object type and transforms, locations and extension of the given agent """
    obj_type = None

    if 'pedestrian' in agent.type_id:
        obj_type = 'Pedestrian'
    elif 'vehicle' in agent.type_id:
        obj_type = 'Car'

    if obj_type is None:
        return None, None, None, None, None

    agent_transform = agent.get_transform()
    # TODO(farzad) what about agent.bounding_box.transform
    bbox_transform = Transform(agent.bounding_box.location)
    ext = agent.bounding_box.extent
    location = agent_transform.location

    return obj_type, agent_transform, bbox_transform, ext, location


def calc_bbox2d_area(bbox_2d):
    """ Calculate the area of the given 2d bbox
    Input is assumed to be xmin, ymin, xmax, ymax tuple 
    """
    xmin, ymin, xmax, ymax = bbox_2d
    return (ymax - ymin) * (xmax - xmin)