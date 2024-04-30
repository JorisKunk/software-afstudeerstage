#!/usr/bin/env python3

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# play sound from a given file path
def play_audio(file_path):
    soundhandle = SoundClient()
    soundhandle.stopAll()
    rospy.sleep(1)
    soundhandle.playWave(file_path)

if __name__ == '__main__':
    rospy.init_node('speaker_control_node')
    # File containing the audio to play
    file_path = "~/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav"  

    play_audio(file_path)

    rospy.spin()
