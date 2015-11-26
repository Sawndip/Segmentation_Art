#ifndef _SEG_CONTROL_H_
#define _SEG_CONTROL_H_
// sys
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segMisc.h"
#include "vectorSpace.h"
#include "threeDiff.h"
#include "psoBook.h"
#include "boundaryScan.h"
#include "contourTrack.h"
#include "VarFlowWA.h"
// namespace
using :: std :: string;
using :: std :: vector;
using namespace cv;
using namespace Vector_Space;
using namespace Var_FlowWA;
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
    ~SegControl();
    int init(const int width, const int height,
             const int skipTB, const int skipLR,
             const int scanBorderSizeTB, const int scanBorderSizeLR);
    // read frame in and deliver to proper members
    int processFrame(const cv::Mat & in,
                     vector<SegResults> & segResults,
                     cv::Mat & bgResult);
    // when no new frames, we flush out cached frames
    int flushFrame(vector<SegResults> & segResults);

private:
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    int m_skipTB;
    int m_skipLR;
    int m_scanBorderSizeTB;
    int m_scanBorderSizeLR;
    
    // key members    
    ThreeDiff m_threeDiff;
    BoundaryScan m_boundaryScan;    
    VarFlowWA m_segBg;
};

}//namespace

#endif // _SEG_CONTROL_H_
