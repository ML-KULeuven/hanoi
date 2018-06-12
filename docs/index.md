---
layout: default
title: "DTAI Robotics: Towers of Hanoi example"
---
# Introduction
This page shows a demonstration of the project where a Kinova Mico robot arm plays the game Towers of Hanoi.
The system uses a Kinect camera to observe the objects and is implemented in [ROS](http://www.ros.org/){:target="_blank"}.
Planning is done using the HYPE planner.

# Experiments
## Setup
The setup in the lab is shown in this figure:

![Experiment setup]({{ site.baseurl }}/assets/setup.png)

Each disk (block) has a color to identify its size in the model corresponding to this drawing (created using [this snippet](https://tex.stackexchange.com/a/268531){:target="_blank"}):

![Torens van Hanoi]({{ site.baseurl }}/assets/hanoi.png)

## Demo
{% include vimeoPlayer.html id=274675727 %}

# Code
## Hanoi
The code is available in the [KU Leuven Machine Learning repository](https://github.com/ML-KULeuven/hanoi){:target="_blank"}.
## DC Bridge
The code for the DC Bridge is available on [BitBucket](https://bitbucket.org/dtai_robotics/dc_bridge){:target="_blank"}.
