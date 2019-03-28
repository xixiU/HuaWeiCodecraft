#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-01-05 14:28:48
# @Author  : yuan
# @Version : 1.0.0
# @describe: 自动将目录下图片保存成gif

import os
import imageio
from PIL import Image
import numpy as np
import glob
def convert(imagedir,targetfilename='target.gif',filesuffix='.png',duration=1):
    """
    imagedir: set image dir path
    filesuffix: set suffix of single image., with default suffix '.png'
    duration: set signle frame duration for gif
    """
    images= glob.glob(os.path.join(imagedir,'*%s'%filesuffix))
    assert len(images)>0
    image_numpy = []

    for i in images:
        temp = Image.open(i)
        temp = np.array(temp)
        image_numpy.append(temp)

    if os.path.dirname(targetfilename)!='' and not os.path.dirname(targetfilename):
        os.makedirs(os.path.dirname(targetfilename))
    
    imageio.mimsave(targetfilename,image_numpy,duration=duration)

if __name__ == "__main__":
    convert('/media/xi/UUI/资料/jietu')
    

