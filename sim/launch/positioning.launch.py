"""positioning_sim against Cosys-AirSim: the simulator's ROS side
(localization_core airsim/launch/airsim.launch.py) and positioning_sim, the
Kalman prediction. Unreal must be running; sim/run.sh starts both through
localization_core's airsim/run.sh.

    ros2 launch positioning.launch.py settings:=.../positioning_field.json \\
        [params:="duration:=60 horizon:=0.5"] [record:=DIR] [rain:=0.5 ...]

    params   positioning_sim's ROS parameters, NAME:=VALUE separated by spaces
             (`ros2 param describe /positioning_sim <name>` explains each)
    record   a rosbag2 directory: the truth (TF) and the estimates, for
             localization_core airsim/evaluate.py

The run ends when positioning_sim does (duration, Ctrl-C).
"""
import json
import os
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRSIM = os.environ.get("LOCSIM_AIRSIM_DIR", os.path.join(ROOT, "external", "localization_core", "airsim"))
TOPICS = ["/positioning/estimate", "/positioning/prediction", "/positioning/baseline", "/positioning/gps"]


def program(context):
    settings = LaunchConfiguration("settings").perform(context)
    # The enu frame's origin: AirSim's OriginGeopoint (its GPS reference).
    o = json.load(open(settings))["OriginGeopoint"]
    origin = {"origin_latitude": float(o["Latitude"]), "origin_longitude": float(o["Longitude"]),
              "origin_altitude": float(o["Altitude"])}
    overrides = [a for p in shlex.split(LaunchConfiguration("params").perform(context)) for a in ("-p", p)]
    sim = Node(executable=os.path.join(ROOT, "build-ros", "sim", "positioning_sim"), name="positioning_sim",
               parameters=[origin], ros_arguments=overrides, output="screen", emulate_tty=True)
    return [sim, RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("settings", description="the AirSim settings.json Unreal runs with"),
        DeclareLaunchArgument("params", default_value="", description="positioning_sim parameters, NAME:=VALUE ..."),
        DeclareLaunchArgument("record", default_value="", description="rosbag2 directory to record into"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(AIRSIM, "launch", "airsim.launch.py")),
                                 launch_arguments={"record": LaunchConfiguration("record"),
                                                   "record_topics": " ".join(TOPICS)}.items()),
        OpaqueFunction(function=program),
    ])
