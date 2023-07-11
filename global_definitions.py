#!/usr/bin/python3

# variables in a shared namespace used in both treads

# threads
t1 = None                   # thread 1
kill_thread_1 = False       # set by toggle_mower_thread()

# mowing patterns
mow_traditional = 1
mow_spiral = 2
mow_random = 3

