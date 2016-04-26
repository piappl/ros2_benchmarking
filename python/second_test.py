#!/usr/bin/python
# -*- coding: utf-8 -*-


from files import TestFile
from files import TestFiles
import glob,os,sys
import numpy as np
import matplotlib.pyplot as plt

import matplotlib.mlab as mlab
import copy
from operator import truediv,sub


if __name__ == "__main__":
        test_name="second_test_beacons"
        plot_name="Network loss"
        t=TestFiles(test_name,plot_name)

#plt.show()
