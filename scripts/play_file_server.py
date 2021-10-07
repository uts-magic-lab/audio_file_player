#!/usr/bin/env python

import subprocess
import tempfile
import os
import signal
import time
import rospy
from actionlib import SimpleActionServer
from audio_file_player.msg import AudioFilePlayAction, AudioFilePlayGoal, AudioFilePlayResult, AudioFilePlayFeedback
from std_msgs.msg import String


class ShellCmd:
    """Helpful class to spawn commands and keep track of them"""

    def __init__(self, cmd):
        self.retcode = None
        self.outf = tempfile.NamedTemporaryFile(mode="w")
        self.errf = tempfile.NamedTemporaryFile(mode="w")
        self.inf = tempfile.NamedTemporaryFile(mode="r")
        self.process = subprocess.Popen(cmd, shell=True, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                        preexec_fn=os.setsid, close_fds=True)

    def __del__(self):
        if not self.is_done():
            self.kill()
        self.outf.close()
        self.errf.close()
        self.inf.close()

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_retcode(self):
        """Get retcode or None if still running"""
        if self.retcode is None:
            self.retcode = self.process.poll()
        return self.retcode

    def is_done(self):
        return self.get_retcode() is not None

    def is_succeeded(self):
        """Check if the process ended with success state (retcode 0)
        If the process hasn't finished yet this will be False."""
        return self.get_retcode() == 0

    def kill(self):
        self.retcode = -1
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()


class AudioFilePlayer(object):
    def __init__(self):
        rospy.loginfo("Initializing AudioFilePlayer...")
        self.current_playing_process = None
        self.afp_as = SimpleActionServer(rospy.get_name(), AudioFilePlayAction,
                                         self.as_cb, auto_start=False)
        self.afp_sub = rospy.Subscriber('~play', String, self.topic_cb,
                                        queue_size=1)
        # By default this node plays files using the aplay command
        # Feel free to use any other command or flags
        # by using the params provided
        self.command = rospy.get_param('~command', 'play')
        self.flags = rospy.get_param('~flags', '')
        self.wrap_file_path = rospy.get_param('~wrap_file_path', True)
        self.feedback_rate = rospy.get_param('~feedback_rate', 10)
        self.afp_as.start()
        # Needs to be done after start
        self.afp_as.register_preempt_callback(self.as_preempt_cb)

        rospy.loginfo(
            "Done, playing files from action server or topic interface.")

    def as_preempt_cb(self):
        if self.current_playing_process:
            self.current_playing_process.kill()
            # Put the AS as cancelled/preempted
            res = AudioFilePlayResult()
            res.success = False
            res.reason = "Got a cancel request."
            self.afp_as.set_preempted(res, text="Cancel requested.")

    def as_cb(self, goal):
        initial_time = time.time()
        self.play_audio_file(goal.filepath)
        r = rospy.Rate(self.feedback_rate)
        while not rospy.is_shutdown() and not self.current_playing_process.is_done():
            feedback = AudioFilePlayFeedback()
            curr_time = time.time()
            feedback.elapsed_played_time = rospy.Duration(
                curr_time - initial_time)
            self.afp_as.publish_feedback(feedback)
            r.sleep()

        final_time = time.time()
        res = AudioFilePlayResult()
        if self.current_playing_process.is_succeeded():
            res.success = True
            res.total_time = rospy.Duration(final_time)
            self.afp_as.set_succeeded(res)
        else:
            if self.afp_as.is_preempt_requested():
                return
            res.success = False
            reason = "stderr: " + self.current_playing_process.get_stderr()
            reason += "\nstdout: " + self.current_playing_process.get_stdout()
            res.reason = reason
            self.afp_as.set_aborted(res)

    def topic_cb(self, data):
        if self.current_playing_process:
            if not self.current_playing_process.is_done():
                self.current_playing_process.kill()
        self.play_audio_file(data.data)

    def play_audio_file(self, audio_file_path):
        # Replace any ' or " characters with emptyness to avoid bad usage
        audio_file_path = audio_file_path.replace("'", "")
        audio_file_path = audio_file_path.replace('"', '')
        if self.wrap_file_path:
            full_command = self.command + " " + self.flags + " '" + audio_file_path + "'"
        else:
            full_command = self.command + " " + self.flags + " " + audio_file_path
        rospy.loginfo("Playing audio file: " + str(audio_file_path) +
                      " with command: " + str(full_command))
        self.current_playing_process = ShellCmd(full_command)


if __name__ == '__main__':
    rospy.init_node('audio_file_player')
    afp = AudioFilePlayer()
    rospy.spin()
