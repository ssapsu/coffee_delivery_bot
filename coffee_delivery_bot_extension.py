# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import json
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.coffee_delivery_bot.coffee_delivery_bot import CoffeeDeliveryBot
from omni.isaac.ui.ui_utils import (
    state_btn_builder,
    setup_ui_headers,
    get_style,
    btn_builder,
    str_builder,
)
from omni.kit.commands import execute
from omni.cuopt.microservice.waypoint_graph_model import (
    WaypointGraphModel,
    load_waypoint_graph_from_file,
)
from omni.cuopt.visualization.generate_waypoint_graph import (
    visualize_waypoint_graph,
)
import omni
import omni.ui as ui
import asyncio


from omni.cuopt.microservice.transport_orders import TransportOrders
from omni.cuopt.microservice.transport_vehicles import TransportVehicles
from omni.cuopt.microservice.cuopt_data_proc import preprocess_cuopt_data
from omni.cuopt.microservice.cuopt_microservice_manager import cuOptRunner
from omni.cuopt.microservice.common import show_vehicle_routes, test_connection

from pxr import UsdShade, UsdGeom, Gf, Sdf
from omni.cuopt.visualization.generate_orders import visualize_order_locations


class CoffeeDeliveryBotExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._extension_data_path = f"{self._extension_path}/omni/cuopt/examples/wpgraph/extension_data/"
        
        # self._waypoint_graph_model = WaypointGraphModel()
        # self._network_ui_data.text = f"Waypoint Graph Network Loaded: {len(self._waypoint_graph_model.nodes)} nodes, {len(self._waypoint_graph_model.edges)} edges"
        self.waypoint_graph_node_path = "/World/WaypointGraph/Nodes"
        self.waypoint_graph_edge_path = "/World/WaypointGraph/Edges"


        self.waypoint_graph_config = "waypoint_graph1.json"
        self.semantic_config = "semantics_data.json"
        self.orders_config = "orders_data.json"
        self.vehicles_config = "vehicle_data.json"

        self._orders_obj = TransportOrders()
        self._vehicles_obj = TransportVehicles()
        self._semantics = []
        
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="coffeeDeliveryBot",
            title="coffeeDeliveryBot Example",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html",
            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            file_path=os.path.abspath(__file__),
            sample=CoffeeDeliveryBot(),
            number_of_extra_frames=4,
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_connect_to_cuOpt_Microservice_ui(frame)
        frame = self.get_frame(index=1)
        self.build_Optimization_Problem_Setup_ui(frame)
        frame = self.get_frame(index=2)
        self.build_Run_cuOpt_ui(frame)
        frame = self.get_frame(index=3)
        self.build_act_robot_ui(frame)

        return


    def _form_cuopt_url(self):
        cuopt_ip = self._cuopt_ip.get_value_as_string()
        cuopt_port = self._cuopt_port.get_value_as_string()
        cuopt_url = f"http://{cuopt_ip}:{cuopt_port}/cuopt/"
        return cuopt_url

    def _on_move_button_event(self, val):
        asyncio.ensure_future(self.sample._on_move_button_event_async(val))
        return
    
    def post_load_button_event(self):
        self.task_ui_elements["Move"].enabled = True
        return

    
    def post_clear_button_event(self):
        return
    
        # Test if cuopt microservice is up and running
    def _test_cuopt_connection(self):
        self._cuopt_ip_promt = "0.0.0.0"
        self._cuopt_port_promt = "5000"
        cuopt_ip = self._cuopt_ip.get_value_as_string()
        cuopt_port = self._cuopt_port.get_value_as_string()

        # if (cuopt_ip == self._cuopt_ip_promt) or (
        #     cuopt_port == self._cuopt_port_promt
        # ):
        #     self._cuopt_status_info.text = (
        #         "FAILURE: Please set both an IP and Port"
        #     )
        #     return
        self._cuopt_status_info.text = test_connection(cuopt_ip, cuopt_port)

    def _load_waypoint_graph(self):
        
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
        
        import omni.kit.commands

        print("loading waypoint graph")
        self._usd_context = omni.usd.get_context()
        
        self._stage = self._usd_context.get_stage()
        waypoint_graph_data_path = (
            f"{self._extension_data_path}{self.waypoint_graph_config}"
        )
        waypoint_graph_data_path = (
            "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/waypoint_graph1.json"
        )
        self._waypoint_graph_model = load_waypoint_graph_from_file(
            self._stage, waypoint_graph_data_path
        )
        visualize_waypoint_graph(
            self._stage,
            self._waypoint_graph_model,
            self.waypoint_graph_node_path,
            self.waypoint_graph_edge_path,
        )
        self._network_ui_data.text = f"Waypoint Graph Network Loaded: {len(self._waypoint_graph_model.nodes)} nodes, {len(self._waypoint_graph_model.edges)} edges"
        # val = True
        # asyncio.ensure_future(self.sample._on_load_waypoint_graph_button(val))

    def _load_orders(self):
        print("Loading Orders")
        orders_path = f"{self._extension_data_path}{self.orders_config}"
        orders_path = "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/orders_data.json"
        self._orders_obj.load_sample(orders_path)
        visualize_order_locations(
            self._stage, self._waypoint_graph_model, self._orders_obj
        )
        self._orders_ui_data.text = f"Orders Loaded: {len(self._orders_obj.graph_locations)} tasks at nodes {self._orders_obj.graph_locations}"

    def _load_vehicles(self):
        print("Loading Vehicles")
        vehicle_data_path = (
            f"{self._extension_data_path}{self.vehicles_config}"
        )
        vehicle_data_path = "/home/hyeonsu/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/coffee_delivery_bot/assets/waypoint/vehicle_data.json"
        self._vehicles_obj.load_sample(vehicle_data_path)
        start_locs = [locs[0] for locs in self._vehicles_obj.graph_locations]
        self._vehicle_ui_data.text = f"Vehicles Loaded: {len(self._vehicles_obj.graph_locations)} vehicles at nodes {start_locs}"

    def _run_cuopt(self):
        print("Running cuOpt")
            
        cuopt_url = self._form_cuopt_url()
        # Solver Settings
        solver_config = {
            "time_limit": 0.01,
            "number_of_climbers": 128,
            "min_vehicles": None,
            "max_slack": None,
            "solution_scope": None,
            "max_lateness_per_route": None,
        }

        # Preprocess network, fleet and task data
        waypoint_graph_data, fleet_data, task_data = preprocess_cuopt_data(
            self._waypoint_graph_model, self._orders_obj, self._vehicles_obj
        )
        print(waypoint_graph_data, fleet_data, task_data)
        cuopt_server = cuOptRunner(cuopt_url)

        # Initialize server data and call for solve
        cuopt_server.set_environment_data(
            waypoint_graph_data, "waypoint_graph"
        )
        cuopt_server.set_fleet_data(fleet_data)
        cuopt_server.set_task_data(task_data)
        cuopt_server.set_solver_config(solver_config)
        cuopt_solution = cuopt_server.solve()
        routes = cuopt_solution

        # Visualize the optimized routes
        self._waypoint_graph_model.visualization.display_routes(
            self._stage,
            self._waypoint_graph_model,
            self.waypoint_graph_edge_path,
            routes,
        )

        # Display the routes on UI
        self._routes_ui_message.text = show_vehicle_routes(routes)

    def build_act_robot_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Move"
                frame.visible = True

                dict = {
                    "label": "Move Robot",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "Connection to mobius server",
                    "on_clicked_fn": self._on_move_button_event,
                }
                self.task_ui_elements["Move"] = state_btn_builder(**dict)
                self.task_ui_elements["Move"].enabled = True
        
        # # Display the routes on UI
        # self._routes_ui_message.text = show_vehicle_routes(routes)
        # asyncio.ensure_future(self.sample._on_set_queuing(**routes))

                
    def build_connect_to_cuOpt_Microservice_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Connect to cuOpt Microservice"
                frame.visible = True
                kwargs = {
                    "label": "cuOpt IP",
                    "type": "stringfield",
                    "default_val": "0.0.0.0",
                    "tooltip": "IP for cuOpt microservice",
                    "on_clicked_fn": None,
                    "use_folder_picker": False,
                    "read_only": False,
                }
                self._cuopt_ip = str_builder(**kwargs)

                kwargs = {
                    "label": "cuOpt Port",
                    "type": "stringfield",
                    "default_val": "5000",
                    "tooltip": "Port for cuOpt microservice",
                    "on_clicked_fn": None,
                    "use_folder_picker": False,
                    "read_only": False,
                }
                self._cuopt_port = str_builder(**kwargs)

                kwargs = {
                    "label": "Test cuOpt Connection ",
                    "type": "button",
                    "text": "Test",
                    "tooltip": "Test to verify cuOpt microservice is reachable",
                    "on_clicked_fn": self._test_cuopt_connection,
                }
                btn_builder(**kwargs)
                
                self._cuopt_status_info = ui.Label(" ")
                
    def build_Optimization_Problem_Setup_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Optimization Problem Setup"
                frame.visible = True
                ui_data_style = {
                    "font_size": 14,
                    "color": 0x88888888,
                    "alignment": ui.Alignment.LEFT,
                }
                
                kwargs = {
                    "label": "Load Waypoint Graph ",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Loads a waypoint graph for the sample environment",
                    "on_clicked_fn": self._load_waypoint_graph,
                }
                btn_builder(**kwargs)
                self._network_ui_data = ui.Label(
                    "No Waypoint Graph network Loaded",
                    width=350,
                    word_wrap=True,
                    style=ui_data_style,
                )

                kwargs = {
                    "label": "Load Orders ",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Loads sample orders",
                    "on_clicked_fn": self._load_orders,
                }
                btn_builder(**kwargs)
                self._orders_ui_data = ui.Label(
                    "No Orders Loaded",
                    width=350,
                    word_wrap=True,
                    style=ui_data_style,
                )

                kwargs = {
                    "label": "Load Vehicles ",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Loads sample vehicle data",
                    "on_clicked_fn": self._load_vehicles,
                }
                btn_builder(**kwargs)
                self._vehicle_ui_data = ui.Label(
                    "No Vehicles Loaded",
                    width=350,
                    word_wrap=True,
                    style=ui_data_style,
                )

    def build_Run_cuOpt_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Run cuOpt"
                frame.visible = True
                
                ui_data_style = {
                    "font_size": 14,
                    "color": 0xBBBBBBBB,
                    "alignment": ui.Alignment.LEFT,
                }

                kwargs = {
                    "label": "Run cuOpt ",
                    "type": "button",
                    "text": "Solve",
                    "tooltip": "Run the cuOpt solver based on current data",
                    "on_clicked_fn": self._run_cuopt,
                }
                btn_builder(**kwargs)
                self._routes_ui_message = ui.Label(
                    "Run cuOpt for solution",
                    width=350,
                    word_wrap=True,
                    style=ui_data_style,
                )