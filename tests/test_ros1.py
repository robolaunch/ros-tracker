import sys
import time

from helpers import getTogetherTests
import pytest
import json
from subprocess import Popen, PIPE, check_output
import os

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../scripts/')

from ros1_info import ROS1
from globals import CannotParseError, NoROScoreError

# this class is used to test the ROS1 class
class TestROS1Parsers:
    
    ############ parseTopicorAction tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseTopic", "test_outputs/ROS1/parseTopic"))
    def test_parseTopicAction(self, test_input, expected):
        # parse the test string
        parsed_topics = ROS1.parseTopicorAction(test_input)
        # compare the parsed topics with the expected result
        assert parsed_topics == json.loads(expected)

    @pytest.mark.parametrize("test_input, expected", [["", ""]])
    def test_parseTopicorAction_EmptyString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseTopicorAction(test_input)

    @pytest.mark.parametrize("test_input, expected", 
                            [["KAE Robolaunch", "42"], # completely different format
                            ["w i t h so m a n y s p a c e", "42"], # input given with similar spaces
                            ["Type: asd\n\n\nPublishers: * q b\n\nSubs: a b c", "42"], # Some keywords are incorrect
                            ["Type: asd\nPublishers: q b\nSubcribers: a b c", "42"], # * is missing in Publishers
                            ["Type:\n\nPublishers: * q b\n\nSubs: a b c", "42"] # type info is missing
                            ])
    def test_parseTopicorAction_CorruptedString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseTopicorAction(test_input)

    
    ############ parseNode tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseNode", "test_outputs/ROS1/parseNode"))
    def test_parseNode(self, test_input, expected):
        # parse the test string
        parsed_nodes = ROS1.parseNode(test_input)
        # compare the parsed nodes with the expected result
        assert parsed_nodes == json.loads(expected)
    
    @pytest.mark.parametrize("test_input, expected", [["", ""]])
    def test_parseNode_EmptyString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseNode(test_input)


    @pytest.mark.parametrize("test_input, expected",
                            [["rosout", "42"],  # completely different format
                            ["/rosout\n/robot\ncontroller", "42"],
                            ["/rosout robot", "42"]
                            ])     
    def test_parseNode_CorruptedString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseNode(test_input)
    
    ############ parseService tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseService", "test_outputs/ROS1/parseService"))
    def test_parseService(self, test_input, expected):
        # parse the test string
        parsed_services = ROS1.parseService(test_input)
        # compare the parsed services with the expected result
        assert parsed_services == json.loads(expected)

    @pytest.mark.parametrize("test_input, expected", [["", ""]])
    def test_parseService_EmptyString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseService(test_input)
    
    @pytest.mark.parametrize("test_input, expected",
                            [["rosout", "42"],  # completely different format
                            ["/rosout\n/robot\ncontroller", "42"],
                            ["Node: /gazebo/XX\nURI:rosrpc://alp-R2D2:44039\nType: controller_manager_msgs/LoadController\nArgs: name\n", "42"]])
    def test_parseService_CorruptedString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseService(test_input)
    

# All tests here should success because the ROScore is running
class TestROS1CLISuccess:


    def sourceROS1(self):
        # we need to be sure that ROS1 is sourced before starting roscore
        # the reason behind that in the computers there may be more than one ROS installation
        # so we need to be sure that the correct one is sourced
        # however, for that we need to find the correct ROS installation
        # there may be more than one ROS1 installation, or only one, but melodic or noetic
        # and if the current sourced distro is foxy, we do not know if we are using melodic or noetic
        # for here I will assume that there is noetic installation, and for different ROS1 installations these tests will behave same, since their CLI is very similar if not the same
        # but this is a weakness that we may need to check in the future
        
        # we need to source noetic environment, but with Popen we only open one command
        # therefore we are sourcing it in the shell, take the environment variables, and change our env variables with that
        # it is a trick that I learned from https://stackoverflow.com/a/22086176/13399661
        source_output = check_output("source /opt/ros/noetic/setup.bash; env -0", shell=True, executable="/bin/bash")
        os.environ.clear()
        # if there are already other ROS environments, when we sourced this the bash will complain:
        # "ROS_DISTRO was set to \'foxy\' before. Please make sure that the environment does not mix paths from different distributions.
        # we need to get rid of that warning in the output
        control_output = source_output.decode("utf-8").split("\\n")
        if control_output[0][2:12] == "ROS_DISTRO":
            source_output = control_output[1].encode("utf-8") # again make the source_output bytes
        
        for line in source_output.split(b'\x00')[:-1]:
            var_name, var_val = line.decode("utf-8").partition("=")[::2]
            os.environ.update(dict([(var_name, var_val)]))

    # a fixture that opens "roscore" process in the shell
    @pytest.fixture(scope="class")
    def ROS1Prepare(self):
        self.sourceROS1()
        p_ros = Popen(["roscore"], stdout=PIPE, stderr=PIPE)
        time.sleep(1.5) # there need to be some time before roscore opens and becomes functional
        yield
        p_ros.terminate()
        time.sleep(1)

    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS1Prepare")
    def test_ROS1_getServices(roscore):
        ROS1.getServices()

    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS1Prepare")
    def test_ROS1_getNodes(roscore):
        ROS1.getNodes()
 
    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("ROS1Prepare")
    def test_ROS1_getTopicsActions(roscore):
        ROS1.getTopicsActions()

# All tests here should give an exception because ROScore is not running in this class
class TestROS1CLIFailure:

    def test_getServices_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS1.getServices()

    def test_getNodes_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS1.getNodes()

    def test_getTopicActions_Exception(self):
        with pytest.raises(NoROScoreError):
            ROS1.getTopicsActions()