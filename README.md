# ros-tracker

One small note, apparently there is a bug in the roslibpy package. This is probably because of Python versions. If you are using Python 3.8 or above, you may see an error like this: builtins.TypeError: _findCaller() takes from 1 to 2 positional arguments but 3 were given.

The solution to it is complicated because it is not exactly because of a line in the roslibpy, it is how the event locks are written in the twisted package and how it is used in the roslibpy somehow. However, solving it is not hard.
Just install a version of twisted 19.7.0 or above, and you should be good to go.
pip3 install twisted==19.7.0