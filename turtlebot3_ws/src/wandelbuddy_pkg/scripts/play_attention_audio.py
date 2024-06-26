#!/usr/bin/env python3

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from rospkg import RosPack

# play a given audiofile with soundhandle
def play_audio(file_name):
    soundhandle = SoundClient()
    rospy.sleep(1)
    #soundhandle.stopAll()
    rp = RosPack()
    file_path = rp.get_path('wandelbuddy_pkg') + '/Sounds/' + file_name
    soundhandle.playWave(file_path)

if __name__ == '__main__':
    rospy.init_node('speaker_control_node')
    # file name of audio file to play
    file_name = "Attention_call.wav"

    play_audio(file_name)

    #rospy.spin()
