import sys
from helpers import getTogetherTests
import pytest
import json

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'scripts/')

from globals import CannotParseError
from ros1_info import ROS1

# this class is used to test the ROS1 class
class TestROS1:
    
    @pytest.mark.parametrize("test_input, expected", getTogetherTests("test_inputs/ROS1/parseTopic", "test_outputs/ROS1/parseTopic"))
    def test_parseTopic(self, test_input, expected):
        # parse the test string
        parsed_topics = ROS1.parseTopic(test_input)
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