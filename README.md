# Code Review of ORB-SLAM2
**Reviewers:** NLS Lab. of Shanghai Jiao Tong University

**ORB-SLAM and review**
[ORB-SLAM2](http://webdiis.unizar.es/~raulmur/orbslam/) is a classic SLAM library and has demonstrated high robustness and versatility in both indoor and outdoor environments. It has become a golden example for SLAM and Robotic researchers to study and have good usderstanding of the architecture of the SLAM systems.

This code review gives more detailed and standardized comments for most important lines of [original ORB-SLAM codes](https://github.com/raulmur/ORB_SLAM2). Most of comments are annotated in Chinese.

A Chinese documentation is presented by all reviewers with the seminar procedure going. All documentation files are published in the "document" folder. Each file gives explaination of one or more classes. The content of explaination includes: usage of the class, despriptions of important member variables, despriptions of important functions and diagrams of related algorithms.

**Useful links** (All links are in Chinese):
[泡泡机器人, SLAM公开课](http://rosclub.cn/post-505.html), 
[CSDN, ORB-SLAM详解](http://blog.csdn.net/u010128736/article/details/53157605), 
[泡泡机器人, g2o公开课，高翔](http://rosclub.cn/post-245.html)
[CSDN, g2o非线性优化](http://blog.csdn.net/stihy/article/details/55254756)


## Procedure
**20170512** Presentations: KeyFrames by Rongzhi, g2o by Zheng.
遗留问题：
#1 KeyFrames的建立，删除的标准是什么，在什么情况执行。地图三位点的增删标准。
#2 KeyFrames的Covisibility Graph以及Essicial Graph的作用是什么。

**20170505** Presentations: System by Zheng, Frame by Qiang.
遗留问题：
#1 KeyPoint的去畸变原理（关于Zhengyou Zhang的work的理解，为什么左右两个Camera可以使用同一个参数矩阵）。

**20170428** Start and warming up

## Related papers
[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *ArXiv preprint arXiv:1610.06475* **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

## Problems
...

