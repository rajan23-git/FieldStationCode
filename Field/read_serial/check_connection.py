## Author: Adrian Muth
## Version: 1/29/23
## Description: Verifies that the connection from Arduino to Raspberry Pi is active and prints a warning if not.

import os
import subprocess

ports = subprocess.check_output("lsusb -s 1:4")
#if arduino found, check its state
if (ports != None):
  os.system("echo 'Arduino no longer connected to Raspberry Pi.'")
