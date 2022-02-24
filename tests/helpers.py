import os
import sys

os.chdir(sys.path[0])

# This function reads the inputs from test_inputs directory 
# and outputs from test_outputs directory, reads files and puts the content in a list
# in the output the inputs and outputs are matched and given together in an array
def getTogetherTests(input_dir, output_dir):
    # get all the files in the input directory
    input_files = os.listdir(input_dir)
    # get all the files in the output directory
    output_files = os.listdir(output_dir)
    # get the number of files in the input directory
    num_input_files = len(input_files)
    # get the number of files in the output directory
    num_output_files = len(output_files)
    # if the number of files in the input directory is not equal to the number of files in the output directory
    if num_input_files != num_output_files:
        # return an error
        return "ERROR! Number of input files and output files are not equal"
    # if the number of files in the input directory is equal to the number of files in the output directory
    else:
        # create an empty list
        together_tests = []
        # for each file in the input directory
        for i in range(num_input_files):
            # get the file name
            input_file_name = input_files[i]
            # get the file name
            output_file_name = output_files[i]
            # get the file path
            input_file_path = os.path.join(input_dir, input_file_name)
            # get the file path
            output_file_path = os.path.join(output_dir, output_file_name)
            # read the file
            input_file = open(input_file_path, "rb")
            # read the file
            output_file = open(output_file_path, "rb")
            # get the input file content
            input_file_content = input_file.read()
            # get the output file content
            output_file_content = output_file.read()
            # add the input file content and output file content to the list
            together_tests.append([input_file_content, output_file_content])
        # return the list
        return together_tests

# this function compares two dictionaries
# there are two list inputs, must_be_true and probably_false
# if the key is in the must_be_true list, it must be in the both dictionary, and the value must be equal to the value in the other dictionary
# if the key is in the probably_false list, it must be in the both dictionary, but the value must not be equal to the value in the other dictionary
# function will return true or false
def compareDictionaries(dict1, dict2, must_be_true, probably_false):
    # for each key in the first dictionary
    for key in dict1:
        # if the key is in the must_be_true list
        if key in must_be_true:
            # if the key is not in the second dictionary
            if key not in dict2:
                return False
            # if the key is in the second dictionary
            else:
                # if the value of the key in the first dictionary is not equal to the value of the key in the second dictionary
                if dict1[key] != dict2[key]:
                    return False
        # if the key is in the probably_false list
        elif key in probably_false:
            # if the key is not in the second dictionary
            if key not in dict2:
                return False
    return True

#print(getTogetherTests("test_inputs/ROS1/parseTopics", "test_outputs/ROS1/parseTopics"))
