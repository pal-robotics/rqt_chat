rqt_chat
========

![rqt_chat screenshot](doc/screenshot.png)

A simple chat plugin for rqt, compatible with ROS4HRI.

User messages are published as `hri_msgs/msg/LiveSpeech` messages on the
`/humans/voices/anonymous_speaker/speech` topic.

TTS action call to `/tts_engine/tts` (`tts_msgs/action/TTS`) are then displayed
as robot's messages.

