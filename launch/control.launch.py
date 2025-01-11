# usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    subscribe_to = context.launch_configurations[
        'subscribe_to']

    publish_to = context.launch_configurations[
        'publish_to']

    output_velocity = False if context.launch_configurations[
        'output_velocity'] == 'false' else True
    
    output_torque = False if context.launch_configurations[
        'output_torque'] == 'false' else True

    mc_rtc_ros_control_node = Node(
        package='mc_rtc_ros_control',
        executable='mc_rtc_ros_control',
        name='mc_rtc_ros_control',
        output='screen',
        parameters=[
            {
                'output_velocity': output_velocity,
                'output_torque': output_torque,
            }
        ],
        remappings=[
            ('subscribe_to', subscribe_to),
            ('publish_to', publish_to),
        ]
    )

    return [mc_rtc_ros_control_node]


def generate_launch_description():

    subscribe_to = DeclareLaunchArgument(
        'subscribe_to',
        default_value='/joint_states',
        description='Subscribe to'
    )

    publish_to = DeclareLaunchArgument(
        'publish_to',
        default_value='/command',
        description='Publish to'
    )

    output_velocity = DeclareLaunchArgument(
        'output_velocity', default_value='false', description='Output velocity'
    )

    output_torque = DeclareLaunchArgument(
        'output_torque', default_value='false', description='Output torque'
    )

    declare_arguments = [
        subscribe_to, publish_to, output_velocity, output_torque]

    return LaunchDescription(declare_arguments + [
        OpaqueFunction(function=launch_setup),
    ])
