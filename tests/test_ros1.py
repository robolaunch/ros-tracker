import sys
import time
from helpers import getTogetherTests, compareDictionaries
import pytest
import json
from subprocess import Popen, PIPE

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../scripts/')

from ros1_info import ROS1
from globals import CannotParseError, NoROScoreError

# this class is used to test the ROS1 class
class TestROS1:
    
    ############ parseTopic tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseTopic", "test_outputs/ROS1/parseTopic"))
    def test_parseTopic(self, test_input, expected):
        # parse the test string
        parsed_topics = ROS1.parseTopic(test_input)
        print("parseTopic: " + str(parsed_topics))
        # compare the parsed topics with the expected result
        assert parsed_topics == json.loads(expected)

    @pytest.mark.parametrize("test_input, expected", [["", ""]])
    def test_parseTopic_EmptyString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseTopic(test_input)

    @pytest.mark.parametrize("test_input, expected", 
                            [["KAE Robolaunch", "42"], # completely different format
                            ["w i t h so m a n y s p a c e", "42"], # input given with similar spaces
                            ["Type: asd\n\n\nPublishers: * q b\n\nSubs: a b c", "42"], # Some keywords are incorrect
                            ["Type: asd\nPublishers: q b\nSubcribers: a b c", "42"], # * is missing in Publishers
                            ["Type:\n\nPublishers: * q b\n\nSubs: a b c", "42"] # type info is missing
                            ])
    def test_parseTopic_CorruptedString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseTopic(test_input)

    ############ getTopics tests ############

    # a fixture that opens "roscore" process in the shell
    @pytest.fixture()
    def roscore(self):
        p = Popen(["roscore"], stdout=PIPE, stderr=PIPE)
        time.sleep(2) # there needs to be some time before roscore opens and becomes functional
        yield
        p.terminate()

    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("roscore")
    def test_getTopics(roscore):
        ROS1.getTopics()

    # getTopics should raise NoROScoreError if there is no roscore process
    def test_getTopics_Exception(aa):
        with pytest.raises(NoROScoreError):
            ROS1.getTopics()

    
    ############ parseNode tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseNode", "test_outputs/ROS1/parseNode"))
    def test_parseNode(self, test_input, expected):
        # parse the test string
        parsed_nodes = ROS1.parseNode(test_input)
        print("parsed_nodes: ")
        print(parsed_nodes)
        print("expected is: ")
        print(expected)
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

    ############ getNodes tests ############
    # a test that uses roscore fixture, then checks if the getTopics method runs without no exceptions
    @pytest.mark.usefixtures("roscore")
    def test_getNodes(roscore):
        ROS1.getNodes()

    # getTopics should raise NoROScoreError if there is no roscore process
    def test_getNodes_Exception(aa):
        with pytest.raises(NoROScoreError):
            ROS1.getNodes()
    
    ############ parseService tests ############
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseService", "test_outputs/ROS1/parseService"))
    def test_parseService(self, test_input, expected):
        # parse the test string
        parsed_services = ROS1.parseService(test_input)
        print("parsed_services: ")
        print(parsed_services)
        # compare the parsed services with the expected result
        assert parsed_services == json.loads(expected)

    @pytest.mark.parametrize("test_input, expected", [["", ""]])
    def test_parseService_EmptyString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseService(test_input)
    
    """
    @pytest.mark.parametrize("test_input, expected",
                            [["rosout", "42"],  # completely different format
                            ["/rosout\n/robot\ncontroller", "42"]])
    def test_parseService_CorruptedString(self, test_input, expected):
        with pytest.raises(CannotParseError):
            # parse the test string
            ROS1.parseService(test_input)
    """