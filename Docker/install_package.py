#!/usr/bin/env python
import tensorflow as tf
tf_version='-'.join((tf.__version__).split('.'))
print(tf_version)
import os
os.system('pip3 install waymo-open-dataset-tf-{}==1.2.0 --user'.format(tf_version))