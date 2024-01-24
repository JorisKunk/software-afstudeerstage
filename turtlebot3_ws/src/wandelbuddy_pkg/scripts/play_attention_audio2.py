#!/usr/bin/env python3

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def play_sound(file_path):
    rospy.init_node('play_sound_node')
    soundhandle = SoundClient()
    
    # Wacht tot de sound_play-node klaar is
    rospy.sleep(1)

    # Bouw het bericht
    sound_msg = SoundRequest()
    sound_msg.sound = SoundRequest.PLAY_FILE
    sound_msg.command = SoundRequest.PLAY_ONCE
    sound_msg.volume = 1.0
    sound_msg.arg = file_path

    # Publiceer het bericht naar het /robotsound-topic
    soundhandle.playWave(file_path)

if __name__ == '__main__':
    # Vervang 'file_path' door het volledige pad naar je WAV-bestand
    file_path = '/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav'
    play_sound(file_path)
