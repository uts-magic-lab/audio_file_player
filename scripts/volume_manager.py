#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
import subprocess
import tempfile
import os
import signal

"""
Little script to expose volume setting (and getting)
using amixer command.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

# Helper class


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


class VolumeManager(object):
    """
    Class to manage the volume of the speakers of
    the machine where it is running. It simply uses
    amixer to execute commands to get and set the volume
    in a 0-100 (%) scale.

    By default it publishes/listens on:
    /volume_manager/get_volume
    /volume_manager/set_volume

    Of type std_msgs/Int8
    """
    def __init__(self,
                 base_set_command='amixer -c 0 sset Master playback',
                 base_get_command='amixer -c 0 sget Master playback'):
        self.base_set_cmd = base_set_command
        self.base_get_cmd = base_get_command
        self.curr_vol = 0
        rospy.loginfo("VolumeManager using base set command: '" +
                      self.base_set_cmd + "'")
        rospy.loginfo("VolumeManager using base get command: '" +
                      self.base_get_cmd + "'")

        self.sub = rospy.Subscriber('~set_volume',
                                    Int8,
                                    self.cb,
                                    queue_size=1)
        # Get volume to start publishing
        self.curr_vol = self.get_current_volume()
        self.pub = rospy.Publisher('~get_volume',
                                   Int8,
                                   latch=True,
                                   queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.curr_vol_cb)

    def curr_vol_cb(self, event):
        self.curr_vol = self.get_current_volume()
        self.pub.publish(Int8(self.curr_vol))

    def get_current_volume(self):
        cmd = ShellCmd(self.base_get_cmd)
        while not cmd.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        output = cmd.get_stdout()
        rospy.logdebug("self.base_get_cmd output: " + str(output))
        # Output looks like:
        # Simple mixer control 'Master',0
        # Capabilities: pvolume pvolume-joined pswitch pswitch-joined
        # Playback channels: Mono
        # Limits: Playback 0 - 87
        # Mono: Playback 44 [51%] [-32.25dB] [on]

        pct_idx = output.index('%')
        # At most 3 characters of 100%
        pct_text = output[pct_idx - 3:pct_idx]
        # Remove [ if it was less than 3 chars
        pct_text = pct_text.replace('[', '')
        # Remove spaces if it was less than 2 char
        pct_text = pct_text.strip()
        curr_vol = int(pct_text)
        return curr_vol

    def cb(self, data):
        if data.data > 100:
            to_pct = 100
        elif data.data < 0:
            to_pct = 0
        else:
            to_pct = data.data
        rospy.loginfo("Changing volume to: " + str(to_pct) + "%")
        cmd_line = self.base_set_cmd + " " + str(to_pct) + "%"
        rospy.loginfo("Executing command: " + cmd_line)
        cmd = ShellCmd(cmd_line)
        while not cmd.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        output = cmd.get_stdout()
        rospy.logdebug("Set command output: " + str(output))


if __name__ == '__main__':
    rospy.init_node('volume_manager')
    # If you need to change the device (something else than master)
    # Just change the base command line
    vm = VolumeManager(base_set_command='amixer -c 0 sset Master playback',
                       base_get_command='amixer -c 0 sget Master playback')
    rospy.spin()
