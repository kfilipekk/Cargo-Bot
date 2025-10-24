import time
from machine import Pin

## Current Checkpoint, simple integer
##When a dead end is detected, execute a turn_around_180() function, find the line again,
# and continue in "Recovery Mode" (driving and looking for the 4-way cross).
#