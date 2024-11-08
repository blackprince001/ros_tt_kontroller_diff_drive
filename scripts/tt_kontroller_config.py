#!/usr/bin/env python

import rospy
import json
from pathlib import Path


def load_json_parameters(file_path: Path):
    try:
        with open(file_path, 'r') as file:
            parameters = json.load(file)
            return parameters
        
    except json.JSONDecodeError as err:
        rospy.logerr(f"Failed to parse configuration file: {err}")


def set_ros_parameters(params: dict):
    for k, v in params.items():
        rospy.set_param(k, v)
        rospy.loginfo(f"Set {k} to {v}")


def run():
    rospy.init_node("tt_kontroller_configurer")

    configuration_file_path = rospy.get_param("~ConfigFile", Path("../config.json"))
    param_configurations = load_json_parameters(configuration_file_path)

    if param_configurations is not None:
        set_ros_parameters(params=param_configurations)
    else:
        rospy.logerr("Failed to load configuration file.")

if __name__ == "__main__":
    run()