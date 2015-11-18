#ifndef _SEG_CONTROL_H_
#define _SEG_CONTROL_H_
// sys
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
// tools - just using Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "threeDiff.h"
#include "contourTrack.h"
#include "psoBook.h"
#include "segmisc.h"
#include "vectorspace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace cv;
using namespace Vector_Space;

namespace Seg_Three
{

class SegControl
{    
// GET All instance we need:
// 1.read frames and dispatch frames to proper member;
// 2.simple optical flow should hold one class with direction/boundary part;
// 3.three diff do locate;
// 4.border do object size detect;
// 5.psoseg do background and foreground preprocess;
public:
    SegControl();
    int init(const int width, const int height);
    // read frame in and deliver to proper members
    int processFrame(const cv::Mat & in, cv::Mat & out);
    // when no new frames, we flush out cached frames
    int flushFrame(cv::Mat & out);

private:
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    cv::Mat bookResult;    
    // key members    
    ThreeDiff threeDiff;
    BoundaryScan boundaryScan;    
    PsoBook psoBook;
};

}//namespace

#endif // _SEG_CONTROL_H_
