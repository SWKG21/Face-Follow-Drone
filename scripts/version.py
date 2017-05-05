#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2

def main(args):
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))

if __name__ == '__main__':
    main(sys.argv)
