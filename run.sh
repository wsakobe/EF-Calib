#!/bin/bash

roslaunch corner_detector corner_detector.launch 

wait

roslaunch eventmap_generator generator.launch

wait

roslaunch circle_detector circle_detector.launch
