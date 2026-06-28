# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from unittest.mock import MagicMock, patch, call  # noqa: F401
import numpy as np
import pytest

# Create a fixure for the task node.
@pytest.fixture
def task_node():
    
    # patch rclpy to a mock object
    # this is to make a object that can be used to test 
    # the task node without actually running it
    rclpy_patcher = patch('loone.task.rclpy')

    # start the patcher and get the mock object
    rclpy_mock = rclpy_patcher.start()

    # Task.__init__ calls super().__init__('Task_Pub') which is rclpy.Node.__init__ 
    # we need to override the __init__ method of rclpy.Node to do nothing so that 
    # we can create a Task object without actually creating a node
    rclpy_mock.node.Node.__init__ = MagicMock(return_value=None)

    from loone.task import Task

    node = Task()

    ## patch the publisher to a mock object
    node.publisher_ = MagicMock()

    # patch logger to a mock object
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node # test runs here

    # stop the patcher
    rclpy_patcher.stop()


# Validation

class TestTask:
    def test_publish(self, task_node):
        # call the publish method
        task_node.action = 1.0
        task_node.target_heading = 0.0
        task_node.target_speed = 1.0

        task_node.publish()

        # check that the publisher was called with the correct message
        expected_msg = [task_node.action, task_node.target_heading, task_node.target_speed]
        task_node.publisher_.publish.assert_called_once()
        published_msg = task_node.publisher_.publish.call_args[0][0]
        assert published_msg.data == expected_msg

    def test_run_task(self, task_node):
        # call the run_task method
        task_node.run_task()

        # check that the action, target_heading, and target_speed are set correctly
        assert task_node.action == 1.0
        assert task_node.target_heading == 0.0
        assert task_node.target_speed == 1.0
    

    

