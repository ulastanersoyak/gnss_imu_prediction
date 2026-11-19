"""positioning_sim against Cosys-AirSim: the simulator's ROS side
(localization_core airsim/launch/airsim.launch.py, with its autopilot flying
the figure eight) and positioning_sim, the Kalman prediction. Unreal must be
running; sim/run.sh starts both through localization_core's airsim/run.sh.

    ros2 launch positioning.launch.py settings:=.../positioning_field.json \\
        [params:="duration:=60 horizon:=0.5 radius:=15"] [autopilot:=false]
        [rviz:=true] [record:=DIR] [rain:=0.5 ...]

    params    ROS parameters, NAME:=VALUE separated by spaces: positioning_sim's
              (sim/src/positioning_sim_parameters.yaml) and the autopilot's
              (radius, altitude, altitude_swing); each node takes its own
    autopilot false: no autopilot, fly yourself (localization_core
              airsim/keyboard.sh)
    rviz      true: rviz2 with sim/rviz/positioning.rviz
    record    a rosbag2 directory: the truth (TF) and the estimates, for
              localization_core airsim/evaluate.py

The run ends when positioning_sim does (duration, Ctrl-C).
"""
import json
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRSIM = os.environ.get("LOCSIM_AIRSIM_DIR", os.path.join(ROOT, "external", "localization_core", "airsim"))
sys.path.insert(0, os.path.join(AIRSIM, "launch"))
from ros_params import ros_arguments  # noqa: E402

RVIZ = os.path.join(ROOT, "sim", "rviz", "positioning.rviz")
PARAMETERS = os.path.join(ROOT, "sim", "src", "positioning_sim_parameters.yaml")
TOPICS = ["/positioning/estimate", "/positioning/prediction", "/positioning/baseline", "/positioning/gps"]


def program(context):
    settings = LaunchConfiguration("settings").perform(context)
    # The enu frame's origin: AirSim's OriginGeopoint (its GPS reference).
    o = json.load(open(settings))["OriginGeopoint"]
    origin = {"origin_latitude": float(o["Latitude"]), "origin_longitude": float(o["Longitude"]),
              "origin_altitude": float(o["Altitude"])}
    overrides = ros_arguments(LaunchConfiguration("params").perform(context), PARAMETERS)
    sim = Node(executable=os.path.join(ROOT, "build-ros", "sim", "positioning_sim"), name="positioning_sim",
               parameters=[origin], ros_arguments=overrides, output="screen", emulate_tty=True)
    return [sim, RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("settings", description="the AirSim settings.json Unreal runs with"),
        DeclareLaunchArgument("params", default_value="", description="positioning_sim parameters, NAME:=VALUE ..."),
        DeclareLaunchArgument("record", default_value="", description="rosbag2 directory to record into"),
        DeclareLaunchArgument("autopilot", default_value="true", description="false: fly it yourself"),
        DeclareLaunchArgument("rviz", default_value="false", description="true: open rviz2"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(AIRSIM, "launch", "airsim.launch.py")),
                                 launch_arguments={"record": LaunchConfiguration("record"),
                                                   "record_topics": " ".join(TOPICS),
                                                   "autopilot": LaunchConfiguration("autopilot"),
                                                   "autopilot_params": LaunchConfiguration("params"),
                                                   "rviz": LaunchConfiguration("rviz"),
                                                   "rviz_config": RVIZ}.items()),
        OpaqueFunction(function=program),
    ])
