# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.kit.commands import execute
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.sensor import _sensor 
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.synthetic_utils import SyntheticDataHelper
from stable_baselines3 import PPO
from queue import Queue
from gym import spaces
from pxr import Gf, Sdf



import omni
import gym
import numpy as np




# from omni.cuopt.microservice.waypoint_graph_model import (
#     WaypointGraphModel,
#     load_waypoint_graph_from_file,
# )
# from omni.cuopt.visualization.generate_waypoint_graph import (
#     visualize_waypoint_graph,
# )
# import omni
# import omni.ui as ui
# import asyncio


# from omni.cuopt.microservice.transport_orders import TransportOrders
# from omni.cuopt.microservice.transport_vehicles import TransportVehicles
# from omni.cuopt.microservice.cuopt_data_proc import preprocess_cuopt_data
# from omni.cuopt.microservice.cuopt_microservice_manager import cuOptRunner
# from omni.cuopt.microservice.common import show_vehicle_routes, test_connection



from omni.cuopt.visualization.generate_orders import visualize_order_locations
from pxr import Gf


# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


class CoffeeDeliveryBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # self.set_world_settings(physics_dt=1.0 / 240.0, stage_units_in_meters=1.0, rendering_dt=1.0 / 60.0)
        return

    def setup_scene(self):
        self._my_world = self.get_world()
        self.stage = omni.usd.get_context().get_stage()

        self._skip_frame = 4
        #================= PATH =================

        # self._orders_obj = TransportOrders()
        # self._vehicles_obj = TransportVehicles()
        # self._semantics = []
        # self.waypoint_graph_node_path = "/World/WaypointGraph/Nodes"
        # self.waypoint_graph_edge_path = "/World/WaypointGraph/Edges"
        # print("Running cuOpt")
        # self._usd_context = omni.usd.get_context()
        # self._stage = self._usd_context.get_stage()
        # waypoint_graph_data_path = (
        #     "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/waypoint_graph1.json"
        # )
        # self._waypoint_graph_model = load_waypoint_graph_from_file(
        #     self._stage, waypoint_graph_data_path
        # )
        # print(self._waypoint_graph_model.node_path_map)
        # orders_path = "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/orders_data.json"
        # self._orders_obj.load_sample(orders_path)
        # vehicle_data_path = "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/vehicle_data.json"
        # self._vehicles_obj.load_sample(vehicle_data_path)
        # visualize_waypoint_graph(
        #     self._stage,
        #     self._waypoint_graph_model,
        #     self.waypoint_graph_node_path,
        #     self.waypoint_graph_edge_path,
        # )
        
        # visualize_order_locations(
        #     self._stage, self._waypoint_graph_model, self._orders_obj
        # )
        # order_inds = []
        
        # print(self._stage)
        
        # for xyz_loc in self._orders_obj.order_xyz_locations:
        #     print(Gf.Vec3d(xyz_loc[0], xyz_loc[1], xyz_loc[2]))
            
        #     min_dist = None
        #     closest_node_path = None
        #     closest_waypoint_path = None
        #     print(self._waypoint_graph_model.path_node_map)
        #     for node_path in self._waypoint_graph_model.path_node_map:
        #         node_prim = self._stage.GetPrimAtPath(node_path)
        #         node_point = get_prim_translation(node_prim)
        #         distance = (node_point - Gf.Vec3d(xyz_loc[0], xyz_loc[1], xyz_loc[2])).GetLength()
        #         if min_dist is None:
        #             min_dist = distance
        #             closest_node_path = node_path
        #         elif min_dist > distance:
        #             min_dist = distance
        #             closest_node_path = node_path
        #         closest_waypoint_path = closest_node_path
            
        #     closest_node_prim = self._stage.GetPrimAtPath(closest_waypoint_path)
        #     order_inds.append(
        #         self._waypoint_graph_model.path_node_map[closest_waypoint_path]
        #     )
        # self._orders.obj.graph_locations = order_inds
        
        # cuopt_url = self._form_cuopt_url()
        
        
        #================= ENVIRONMENT =================
        
        self.map_type = "a"
        
        if self.map_type == "simple_maze":
            map_asset_path="/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/maps/simple_maze.usd"
            execute(
                "IsaacSimSpawnPrim",
                usd_path=map_asset_path,
                prim_path="/World/env_space",
                translation=(0,0,0),
                rotation=(0,0,0,0)
            )
        elif self.map_type == "maze":
            map_asset_path="/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/maps/maze.usd"
            execute(
                "IsaacSimSpawnPrim",
                usd_path=map_asset_path,
                prim_path="/World/env_space",
                translation=(0,0,0),
                rotation=(0,0,0,0)
            )
        else:
            map_asset_path ="/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/maps/5f/5f_0919.usd"
            execute(
                "IsaacSimSpawnPrim",
                usd_path=map_asset_path,
                prim_path="/World/env_space",
                translation=(0,0,0),
                rotation=(0,90,90,0)
            )
            execute(
                "IsaacSimScalePrim",
                prim_path="/World/env_space",
                scale=(0.01,0.01,0.01),
            )
        
        agent_asset_path ="/home/hyeonsu/Documents/catkin_ws/src/scout_ros/scout_description/urdf/scout_v2/scout_v2.usd"
        
        self.agent = self._my_world.scene.add(
            WheeledRobot(
                prim_path="/World/scout_v2",
                name="my_scout_v2",
                wheel_dof_names=["front_left_wheel","rear_left_wheel","front_right_wheel","rear_right_wheel"],
                wheel_dof_indices=[0,1,2,3],
                create_robot=True,
                usd_path=agent_asset_path,
                position=np.array([1, 2, 0.4]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        )
        
        self.goal = self._my_world.scene.add(
            VisualCuboid(
                prim_path="/World/new_cube_1",
                name="visual_cube",
                position=np.array([0, 0, 0.5]),
                size=0.2,
                color=np.array([1.0, 0, 0]),
            )
        )
        
        self.path_planning = Queue()
        
        lidarPath = "/lidar"
        parent    = "/World/scout_v2/base_link"
        min_range = 0.15
        max_range = 5
                
        result, prim = execute(
            "RangeSensorCreateLidar",
            path=lidarPath,
            parent=parent,
            min_range=min_range,
            max_range=max_range,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=(360/12),
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False
        )
        
        #================= CONTROLLER =================
        
        class AdvancedController(BaseController):
            def __init__(self):
                super().__init__(name="my_cool_controller")
                self._wheel_radius = 330/100./2
                self._wheel_base = 330/100
                return

            def forward(self, command):
                joint_velocities = [0.0, 0.0, 0.0, 0.0]
                joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
                joint_velocities[1] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
                joint_velocities[2] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
                joint_velocities[3] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
                return ArticulationAction(joint_velocities=joint_velocities)
        
        self.agent_controller = AdvancedController()
        self.max_velocity = 1
        self.max_angular_velocity = np.pi
        self.reset_counter = 0
        
        #================= SENSORS =================
        
        self._sdh = SyntheticDataHelper()
        ### LiDAR
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._lidar_path = "/World/scout_v2/base_link/lidar"
        ### IMU
        self._imu = _sensor.acquire_imu_sensor_interface()
        self._imu_path = "/World/scout_v2/base_link/Imu_Sensor"
        
        execute('ChangeProperty',
            prop_path=Sdf.Path('/World/scout_v2/base_link/lidar.xformOp:translate'),
            value=Gf.Vec3d(0.0, 0.0, 0.2),
            prev=Gf.Vec3d(0.0, 0.0, 0.0))
        
        #================= POLICY =================
        
        self.action_type = "discrete"
        if self.action_type == "continous":
            self.action_space = spaces.Box(low=-8, high=15, shape=(2,), dtype=np.float32)
        else:
            self.action_space = spaces.Discrete(6)
            # self.action_space = spaces.Discrete(4)
        
        self.state_pos_space = spaces.Box(
			low=np.array([0,-np.pi], dtype=np.float32),
			high=np.array([8,np.pi], dtype=np.float32),
			dtype=np.float32
		)

        self.state_vel_space = spaces.Box(
			low=np.array([0,-np.pi/4], dtype=np.float32),
			high=np.array([10,np.pi/4], dtype=np.float32),
			dtype=np.float32
        )
        
        self.state_IR_space = spaces.Box(low=0, high=1, shape=(1,12), dtype=np.float32)
        
        self.observation_space = spaces.Dict({
            "IR_raleted":self.state_IR_space,
            "pos_raleted":self.state_pos_space,
            "vel_raleted":self.state_vel_space,
            # "lidar_min_index":self.lidar_min_index
        })
        
        policy_path = "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/policy/drive/coffeeDelivery_policy_checkpoint_280000_steps.zip"
        self.model = PPO.load(policy_path)
        return

    async def setup_post_load(self):
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return
    
    def get_observations(self):
        self._my_world.render()

        lidar_data = self._lidar.get_linear_depth_data(self._lidar_path)
        lidar_data = np.resize(lidar_data, (1, 12))
        
        # lidar_min_index = np.argmin(lidar_data)
        
        goal_world_position, _ = self.goal.get_world_pose()
        agent_position, agent_orientation = self.agent.get_world_pose()
        d = np.linalg.norm(goal_world_position - agent_position)
        
        qx = agent_orientation[0]
        qy = agent_orientation[1]
        qz = agent_orientation[2]
        qw = agent_orientation[3]
        
        y = (goal_world_position[1] - agent_position[1])
        x = (goal_world_position[0] - agent_position[0])
        
        theta = np.arctan2(2.0 *(qy * qz + qw * qx), 1.0 - 2.0 *(qw * qw + qz * qz))
        alpha = np.arctan2(y, x)
        
        if alpha < 0:
            alpha +=  2.0 * np.pi
        angle = np.arctan2(np.sin(theta - alpha), np.cos(alpha - theta))
        target_relative_to_robot_data = np.array([d, angle])
        vel = self.agent.get_linear_velocity()
        vel_x, vel_y = vel[0], vel[1]
        real_V = np.sqrt(np.power(vel_x, 2) + np.power(vel_y, 2))
        real_W = self.agent.get_angular_velocity()[2]
        
        vase_vel_data = np.array([ real_V, real_W])
        obs = {"IR_raleted" : lidar_data, "pos_raleted" : target_relative_to_robot_data, "vel_raleted" : vase_vel_data,} 
        
        return obs
    
    async def _on_move_button_event_async(self, val):
        world = self.get_world()
        if val:
            await world.play_async()
            world.add_physics_callback("sim_step", self._on_follow_move_button_simulation_step)
        else:
            world.remove_physics_callback("sim_step")
        return

    async def _on_set_queuing(self, **kwargs):
        routes = kwargs['vehicle_data']['0']['route']
        
        prim_path = "/World/WaypointGraph/Nodes/Node_"
        for i in routes:
            usd_prim = self.stage.GetPrimAtPath(prim_path + str(i))
            prim_pose = usd_prim.get_world_pose()
            self.path_planning.put(prim_pose)
    
    async def _on_follow_load_waypoint_graph_async(self):
        pass

    def _on_follow_move_button_simulation_step(self, size):
        obs = self.get_observations()
        action, _ = self.model.predict(observation=obs, deterministic=True)
        if self.action_type == "continous":
            # action forward velocity , angular velocity on [-1, 1]
            raw_forward = action[0]
            raw_angular = action[1]
        else:
            # movements = np.array([[8, 0], [-8, 0], [15, 0.785], [15, -0.785], [0, -3.142], [0, 3.142]])
            movements = np.array([[3.14, 0], [-3.14, 0], [0.785, -3.142], [0.785, 3.142], [0, -8], [0, 8]])*3
            
            #왼쪽 돌기, 오른쪽 돌기, 왼쪽 회전, 오른쪽 회전, 후진, 전진
            raw_forward = movements[action][0]
            raw_angular = movements[action][1]

        # we want to force the agent to always drive forward
        # so we transform to [0,1].  we also scale by our max velocity
        forward = (raw_forward + 1.0) / 2.0
        forward_velocity = forward * self.max_velocity

        # we scale the angular, but leave it on [-1,1] so the
        # agent can remain an ambiturner.
        angular_velocity = raw_angular * self.max_angular_velocity

        self.agent.apply_wheel_actions(
            self.agent_controller.forward(command=[forward_velocity, angular_velocity])
        )
        
        goal_world_position, _ = self.goal.get_world_pose()
        agent_position, agent_orientation = self.agent.get_world_pose()
        
        distance = np.linalg.norm(goal_world_position - agent_position)
        if distance < 1:
            if self.path_planning.qsize()>0:
                next_goal_pos = self.path_planning.get()
                self.goal.set_world_pose(np.array(next_goal_pos))
                print(self.goal.get_world_pose())
        return

    def world_cleanup(self):
        return
