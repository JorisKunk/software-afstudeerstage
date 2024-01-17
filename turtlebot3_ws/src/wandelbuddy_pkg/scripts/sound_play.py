#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def play_audio(file_path):
    soundhandle = SoundClient()
    soundhandle.stopAll()
    rospy.sleep(1)
    soundhandle.playWave(file_path)

if __name__ == '__main__':
    rospy.init_node('speaker_control_node')

    file_path = "/../Sounds/Attention_call.wav"  # Vervang dit door het pad naar je audiobestand

    play_audio(file_path)

    rospy.spin()
