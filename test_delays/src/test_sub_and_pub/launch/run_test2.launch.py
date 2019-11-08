import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


###DON'T USE!!!###
def generate_launch_description():
    node_prefix = launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names')
    msg_num = launch.actions.DeclareLaunchArgument(
        'message_number',
        default_value='500',
        description='Number of messages to be sent')
    msg_len = launch.actions.DeclareLaunchArgument(
        'message_length',
        default_value='10',
        description='Length of messages to be sent')
    descr = [
        node_prefix,
        msg_num,
        msg_len,
    ]
    for i in range(500):
        descr.append(launch_ros.actions.Node(
        package='test_sub_and_pub', node_executable='test_sub', output='screen',
        node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'sub'+ str(i)],
        arguments = ['false']))

    publisher = launch_ros.actions.Node(
            package='test_sub_and_pub', node_executable='test_pub', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'pub'],
            arguments = [launch.substitutions.LaunchConfiguration('message_length'), launch.substitutions.LaunchConfiguration('message_number')])
    descr.append(publisher)
    return launch.LaunchDescription(descr)
