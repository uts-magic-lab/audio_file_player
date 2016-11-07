# audio_file_player
audio_file_player offers a simple way to play audio files through
an action server interface and a topic interface.

By default it uses sox's `play` to play the audio files (as it plays more file types than for example `aplay`).
You may need to install:
````
sudo apt-get install sox libsox-fmt-all
````

# Example usage

You can find an example mp3 file in this package in the folder assets, called `tada.mp3`.

Launch the node:

    roslaunch audio_file_player audio_file_player.launch

Try the topic interface:

    rostopic pub /audio_file_player/play std_msgs/String `rospack find audio_file_player`/assets/tada.mp3

Try the actionlib interface:

    rosrun actionlib axclient.py /audio_file_player

# Use your own playing script
If you want to use your own command for playing the files (or even use this node for some other
purpose) just modify the launch file to use the command and flags that you want:

````xml
<launch>
    <node pkg="audio_file_player" name="audio_file_player" type="play_file_server.py" output="screen">
        <!-- Parameters to change the player to use other commands, flags and the rate (Hz) at which
        the feedback is published, this is equivalent to the default configuration -->
        <param name="command" value="play"/>
        <param name="flags" value=""/>
        <param name="feedback_rate" value="10"/>
    </node>
</launch>
````
