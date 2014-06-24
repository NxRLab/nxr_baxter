# NxR's Baxter Package

This package contains most of the work that has been done by the Neuroscience and Robotics Laboratory with our Baxter Research Robot (referred to internally as RJ). 

RJ has been installed in a public space at Northwestern University and is currently running a demo that allows a user to pick one of two games. The first game is the "mime" game, and the second is the "crane" game. They are described below.

Since RJ is in a public setting, his software is designed to run 24/7. There are some problems with this, primarily with the use of an Asus Xtion to interact with users. Over time, the performance of the Asus can degrade and it will often not start up properly. Since much of the code for this device is closed source and proprietary, we have developed a few work arounds which we describe below.