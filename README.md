# Spring-2016-Launcher-Competition
Image analysis for target(s) recognition; trajectory calculation for target(s); solution output to Launcher; execute Position, Launch, Reload ect... 

Contains cpp src for MatLab processing and solution output. Arduino and MatLab interface via serial communication. 

Analyze an image for desired color signature and locate centroid(s); Generage grid of target area with and target location(s); Send data to arduino for calculaton and back to MatLab and verify solution(s); Initate Lauch sequence: Right/Left positioning via IR strip and sensor; Trajector poisitioning on calibrated Launcher with calibrated handwound solenoid via 180 servo on calibarted handmade 4-bar assemply; Send pulse to solenoid for magnet interaction to impact ping pong ball; Position to Reloat position; Reload servo actuation timed for reload via gravity; Begin positioning sequence, launch, and relaod until all targets have been attempted.
