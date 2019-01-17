USE COMPUTER VISION TO READ MECHANICAL GAUGES
=============================================

Many people still have to get up every hour at night to read values from mechanical gauges attached to old, but expensive machinery. In the 21st century, this should not be necessary!

While I appreciate the love for old machines and have full understanding for the reluctance of buying a new one, I believe smart software can help out.

The idea is to use a standard webcam, place it in front of the gauge, and use a PC to analyze the picture, identify the gauge, the dial, the numbers and identify where the pointer is, then read its values. The tool is supposed to be configurable, so that you can decide at which point of time a camera image should be taken.

Details
-------

A little project I worked on in 2015, but unfortunately never had the time to really bring it far...

Written entirely in C++, using OpenCV, boost, tbb, WxWidgets, tinyxml2.
Also supports reading values from digital measurement gauges made by HELIOS.

Suggest to not use this code, back then I was still quite new to C++11 and most parts where I used boost should be replaced by C++11 standard library functions.

Still, maybe it is helpful for someone?