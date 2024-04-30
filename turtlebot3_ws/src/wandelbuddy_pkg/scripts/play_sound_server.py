#!/usr/bin/env python3

import rospy
from wandelbuddy_pkg.srv import PlaySound
import subprocess

def handle_play_sound(request):
    try:
        subprocess.call(["paplay", request.file_path])
        return True
    except Exception as e:
        rospy.logerr(f"Fout bij het afspelen van het geluid: {e}")
        return False

def play_sound_server():
    rospy.init_node('play_sound_server')
    service = rospy.Service('play_sound', PlaySound, handle_play_sound)
    rospy.loginfo("Service voor geluid afspelen is actief.")
    rospy.spin()

if __name__ == '__main__':
    play_sound_server()
