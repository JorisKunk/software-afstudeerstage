#!/usr/bin/env python3

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# play sound from a given audiofile over via the robot/sound topic
def play_sound(file_path):
    rospy.init_node('play_sound_node')
    soundhandle = SoundClient()
    
    # Wait until the sound_play-node is ready
    rospy.sleep(1)

    # Build the message to sound message
    sound_msg = SoundRequest()
    sound_msg.sound = SoundRequest.PLAY_FILE
    sound_msg.command = SoundRequest.PLAY_ONCE
    sound_msg.volume = 1.0
    sound_msg.arg = file_path

    # Publish the message to the /robotsound-topic
    soundhandle.playWave(file_path)

if __name__ == '__main__':
    # file path to the audio file
    file_path = '/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav'
    play_sound(file_path)
