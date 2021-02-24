#!/usr/bin/python

import os
import rospy
from std_msgs.msg import String, UInt8MultiArray
from pocketsphinx import pocketsphinx
from sphinxbase.sphinxbase import *
from pocketsphinx_ros.msg import DecodedPhrase

class VoiceDecoder(object):
    """Class to add keyword spotting functionality"""

    def __init__(self):
        rospy.init_node("voice_recognizer")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        self.utterance_started = False
        self.load_config()
        self.start_recognizer()

        self.pub_ = rospy.Publisher("decoded_phrase", DecodedPhrase, queue_size=10)
        rospy.Subscriber("sphinx_audio", UInt8MultiArray, self.process_audio)

        rospy.spin()

    def load_config(self):
        self._hmm = rospy.get_param("~hmm", False)
        self._dict = rospy.get_param("~dict", False)
        self._kws = rospy.get_param("~kws", False)
        self._keyphrase = rospy.get_param("~keyphrase", False)
        self._threshold = rospy.get_param("~threshold", False)
        self._gram = rospy.get_param("~gram", False)
        self._grammar = rospy.get_param("~grammar", False)
        self._rule = rospy.get_param("~rule", False)
        #self._out_topic = rospy.get_param("~out_topic", "decoded_phrase")

        rospy.logwarn("Using default hmm") if not self._hmm else rospy.loginfo("hmm file: %s", self._hmm)
        rospy.logwarn("Using default dict") if not self._dict else rospy.loginfo("Dict file: %s", self._dict)

    def start_recognizer(self):

        config = pocketsphinx.Decoder.default_config()

        # Setup decoder config
        if self._hmm : config.set_string('-hmm', self._hmm)
        if self._dict : config.set_string('-dict', self._dict)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self._hmm, "feat.params"))
        #config.set_boolean('-bestpath', True)
        if self._kws: config.set_string('-kws', self._kws)
        # else:
        #     # In case keyphrase is provided
        #     config.set_string('-keyphrase', self.keyphrase)
        #     config.set_float('-kws_threshold', self.kws_threshold)

        # Set required configuration for decoder
        self.decoder = pocketsphinx.Decoder(config)

        if self._gram and self._grammar and self._rule:
            jsgf = Jsgf(self._gram)
            rule = jsgf.get_rule(self._grammar + '.' + self._rule)
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            fsg.writefile(self._gram + '.fsg')

            self.decoder.set_fsg(self._gram, fsg)
            self.decoder.set_search(self._gram)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder is successfully started")

    def process_audio(self, msg):
        """Audio processing"""

        # Audio processing
        self.decoder.process_raw(msg.data, False, False)
        
        # by JSGF
        if self._gram:
            self.in_speech = self.decoder.get_in_speech()
            if  self.in_speech != self.utterance_started:
                self.utterance_started = self.in_speech
                if not self.utterance_started:
                    self.decoder.end_utt()
                    if self.decoder.hyp() is not None:
                        rospy.logwarn("Detected: {} with prob {} and score {}".format(self.decoder.hyp().hypstr, self.decoder.hyp().prob, self.decoder.hyp().best_score))
                        msg = DecodedPhrase()
                        msg.header.stamp = rospy.Time.now()
                        msg.phrase = self.decoder.hyp().hypstr
                        msg.score = self.decoder.hyp().best_score
                        self.pub_.publish(msg)
                    self.decoder.start_utt()
        # by Keywords
        elif self._kws:
            if self.decoder.hyp() is not None:
                for seg in self.decoder.seg():
                    rospy.loginfo("Detected key words: %s ", seg.word)
                    self.decoder.end_utt()
                    msg = DecodedPhrase()
                    msg.header.stamp = rospy.Time.now()
                    msg.phrase = seg.word.lower() 
                    msg.score = self.decoder.hyp().best_score
                    self.pub_.publish(msg)
                self.decoder.start_utt()

        elif self._kws:
            if self.decoder.hyp() is not None:
                for seg in self.decoder.seg():
                    rospy.loginfo("Detected key words: %s ", seg.word)
                    self.decoder.end_utt()
                    self.pub_.publish(seg.word.lower())
                self.decoder.start_utt()

    @staticmethod
    def shutdown():
        self.decoder.end_utt()
        rospy.loginfo("Stop Voice Decoder")
        # rospy.sleep(1)

if __name__ == "__main__":
    VoiceDecoder()

