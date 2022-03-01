import sys
import time

from numpy import source
from helpers import getTogetherTests, compareDictionaries
import pytest
import json
from subprocess import Popen, PIPE, check_output
import os

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../scripts/')

from ros2_info import ROS2
from globals import CannotParseError, NoROScoreError


class TestROS2Parsers:

    ############ parseTopic tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS2/parseTopic", "test_outputs/ROS2/parseTopic"))
    def test_parseTopic(self, test_input, expected):
        # parse the test string
        parsed_topics = ROS2.parseTopics(test_input)
        # compare the parsed topics with the expected result
        assert parsed_topics == json.loads(expected)
# All tests here should give an exception because ROScore is not running in this class
class TestROS2CLIFailure:

    def test_getServices_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS2.getServices()

    def test_getNodes_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS2.getNodes()

    def test_getTopics_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS2.getTopics()

# All tests here should success because the ROScore is running
class TestROS2CLISuccess:

    def sourceROS2(self):
        # we need to source foxy environment, but with Popen we only open one command
        # therefore we are sourcing it in the shell, take the environment variables, and change our env variables with that
        # it is a trick that I learned from https://stackoverflow.com/a/22086176/13399661
        source_output = check_output("source /opt/ros/foxy/setup.bash; env -0", shell=True, executable="/bin/bash")
        os.environ.clear()
        # if there are already other ROS environments, when we sourced this the bash will complain:
        # "ROS_DISTRO was set to \'noetic\' before. Please make sure that the environment does not mix paths from different distributions.
        # we need to get rid of that warning in the output
        control_output = source_output.decode("utf-8").split("\\n")
        if control_output[0][2:12] == "ROS_DISTRO":
            source_output = control_output[1].encode("utf-8") # again make the source_output bytes
        
        for line in source_output.split(b'\x00')[:-1]:
            var_name, var_val = line.decode("utf-8").partition("=")[::2]
            os.environ.update(dict([(var_name, var_val)]))

    def unsourceROS2(self):
        source_output = check_output("env -i bash --norc --noprofile; env -0", shell=True, executable="/bin/bash", )
        os.environ.clear()
        control_output = source_output.decode("utf-8").split("\\n")
        if control_output[0][2:12] == "ROS_DISTRO":
            source_output = control_output[1].encode("utf-8") # again make the source_output bytes
        
        for line in source_output.split(b'\x00')[:-1]:
            var_name, var_val = line.decode("utf-8").partition("=")[::2]
            os.environ.update(dict([(var_name, var_val)]))


    # a fixture that opens "roscore" process in the shell
    @pytest.fixture(scope="class")
    def ROS2Prepare(self):
        self.sourceROS2()
        time.sleep(1.5) # there need to be some time before roscore opens and becomes functional
        yield
        self.unsourceROS2()
        time.sleep(1)

    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS2Prepare")
    def test_ROS2_getServices(roscore):
        ROS2.getServices()

    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS2Prepare")
    def test_ROS2_getNodes(roscore):
        ROS2.getNodes()
 
    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS2Prepare")
    def test_ROS2_getTopics(roscore):
        ROS2.getTopics()
