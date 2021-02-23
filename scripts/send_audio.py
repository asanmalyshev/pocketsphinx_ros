#!/usr/bin/python
# coding: utf-8

from time import sleep
import pyaudio
import rospy

from std_msgs.msg import UInt8MultiArray

class AudioMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):
        rospy.init_node("audio_streamer")
        rospy.on_shutdown(self.shutdown)

        self.load_config()
        self.chunk_size = self.audio_chunk_size
        self.audio_msg = UInt8MultiArray()
        # self.audio_msg.layout.dim[0].label='stream_data'
        # self.audio_msg.layout.dim[0].size=self.chunk_size
        # self.audio_msg.layout.dim[0].stride = self.chunk_size
        self.delay = self.node_delay

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("sphinx_audio", UInt8MultiArray, queue_size=10)

        # All set. Publish to topic
        self.transfer_audio_msg()

    def load_config(self):
        self.audio_chunk_size = rospy.get_param('~audio_chunk_size', 1024)
        self.node_delay = rospy.get_param('~node_delay', 2)

    def transfer_audio_msg(self):
        rospy.loginfo("audio input node will start after delay of %s seconds", self.delay)
        sleep(self.delay)

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=self.chunk_size)
        stream.start_stream()

        while not rospy.is_shutdown():
            buf = stream.read(self.chunk_size)
            if buf:
                # Publish audio to topic
                self.audio_msg.data = buf
                self.pub_.publish(self.audio_msg)
            else:
                break

    @staticmethod
    def shutdown():
        pass

if __name__ == "__main__":
    AudioMessage()
