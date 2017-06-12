"""/usr/bin/python"""

import os

import rospy
import rospkg

from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


class KWSDetection(object):
    """Class to add keyword spotting functionality"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("kws_data", String, queue_size=10)
        # Initalizing publisher for continuous ASR
        self.continuous_pub_ = rospy.Publisher(
            "jsgf_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("kws_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)
        # Initializing rospack to find package location
        rospack = rospkg.RosPack()

        # Params

        # Location of external files
        self.location = rospack.get_path('pocketsphinx') + '/demo/'
        # File containing language model
        _lm_param = "~lm"
        # Dictionary
        _dict_param = "~dict"
        # List of keywords to detect
        _kws_param = "~kws"
        """Not necessary to provide the next two if _kws_param is provided
        Single word which needs to be detected
        """
        _keyphrase_param = "~keyphrase"
        # Threshold frequency of above mentioned word
        _threshold_param = "~threshold"
        # Option for continuous
        self._option_param = "~option"

        # Variable to distinguish between kws list and keyphrase.
        # Default is keyword list
        self._list = True

        # Setting param values
        if rospy.has_param(_lm_param):
            self.class_lm = self.location + rospy.get_param(_lm_param)
            if rospy.get_param(_lm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.class_lm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find defaut model.")
                    return
        else:
            rospy.loginfo("Couldn't find lm argument")

        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            self.lexicon = self.location + rospy.get_param(_dict_param)
        else:
            rospy.logerr(
                'No dictionary found. Please add an appropriate dictionary argument.')
            return
        rospy.loginfo(rospy.get_param(_kws_param))

        if rospy.has_param(_kws_param) and rospy.get_param(_kws_param) != ":default":
            self._list = True

            self.kw_list = self.location + rospy.get_param(_kws_param)
        elif rospy.has_param(_keyphrase_param) and \
        rospy.has_param(_threshold_param) and \
        rospy.get_param(_keyphrase_param) != ":default" and \
        rospy.get_param(_threshold_param) != ":default":
            self._list = False

            self.keyphrase = rospy.get_param(_keyphrase_param)
            self.kws_threshold = rospy.get_param(_threshold_param)
        else:
            rospy.logerr(
                'kws cant run. Please add an appropriate keyword list.')
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle keyword spotting of audio"""

        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.class_lm)
        config.set_string('-dict', self.lexicon)

        if self._list:
            # Keyword list file for keyword searching
            config.set_string('-kws', self.kw_list)
        else:
            # In case keyphrase is provided
            config.set_string('-keyphrase', self.keyphrase)
            config.set_float('-kws_threshold', self.kws_threshold)

        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("sphinx_msg", String, self.process_audio)
        rospy.spin()

    def process_audio(self, data):
        """Audio processing based on decoder config"""
        # For continuous mode
        stop_output = False

        need_continuous = rospy.has_param(self._option_param)

        # Check if keyword detected
        if not stop_output:
            if self.decoder.hyp() != None:
                # Actual processing
                self.decoder.process_raw(data.data, False, False)
                rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                               for seg in self.decoder.seg()])
                rospy.loginfo("Detected keyphrase, restarting search")
                seg.word = seg.word.lower() #pylint: disable=undefined-loop-variable
                self.decoder.end_utt()
                # Publish output to a topic
                self.pub_.publish(seg.word) #pylint: disable=undefined-loop-variable
                if need_continuous:
                    stop_output = True
                else:
                    self.decoder.start_utt()
        else:
            self.continuous_pub_.publish(data.data)
    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    KWSDetection()
