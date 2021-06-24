import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():

    node = Node(package='natnet', executable='natnet_node')

    desc = LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest()
    ])

    context = {
        'node': node
    }

    return desc, context


class TestStarted(unittest.TestCase):

    def test_node(self, proc_output, node):
        proc_output.assertWaitFor(
            'Started node',
            process=node,
            timeout=30
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, node):

        launch_testing.asserts.assertExitCodes(proc_info, process=node)
