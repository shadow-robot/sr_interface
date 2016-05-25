#!/usr/bin/env python

import glob
import yaml
import subprocess
from os.path import isdir
import signal
import psutil


class AnnotationParserBase(object):
    """
    Parses the given annotation file (yaml).
    """
    def __init__(self, path_to_annotation, path_to_data):
        self.__path_to_annotation = path_to_annotation
        self.__path_to_data = path_to_data
        self.parse()

    def parse(self):
        with open(self.__path_to_annotation, 'r') as f:
            self.__annotations = yaml.load(f)

    def play_bag(self):
        """
        Plays the associated bag file
        """
        self.__rosbag_proc = subprocess.Popen("rosbag play --clock " + self.__annotations["bag_file"],
                                              stdin=subprocess.PIPE, shell=True,
                                              cwd=self.__path_to_data)

    def stop_bag(self):
        """
        Stops the currently running rosbag process.
        """
        process = psutil.Process(self.__rosbag_proc.pid)
        for sub_process in process.get_children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        self.__rosbag_proc.wait()  # we wait for children to terminate
        try:
            self.__rosbag_proc.terminate()
        except:
            pass
            # the terminate might not be needed

    def check_results(self, results):
        """
        Compares the results with the annotations found in
        self.__annotations. This method is specific to the
        benchmarking used.
        """
        raise NotImplementedError("Implement this method in your own class.")

    def to_xml(self, results):
        """
        Saves the results as a Jenkins parsable xml file for display.
        """
        raise NotImplementedError("TODO implement me")


class BenchmarkingBase(object):
    """
    Base class for running the benchmarking.
    """
    def __init__(self, path_to_data):
        if not isdir(path_to_data):
            raise Exception("Could not find the directory " + path_to_data)

        self.__path_to_data = path_to_data

    def load_files(path_to_data):
        """
        Loads the annotation file list from the given path.

        @param path_to_data the folder containing the different annotation
               files and data.
        """

        yaml_files = glob.glob(path_to_data + "/*.yaml")

        return yaml_files
