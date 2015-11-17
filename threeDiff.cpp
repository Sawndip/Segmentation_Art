#include "ThreeDiff.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ThreeDiff :: ThreeDiff()
{
    m_bInit = false;
    return;
}

ThreeDiff :: ~ThreeDiff()
{
    return;        
}

int ThreeDiff :: init(const int width, const int height)
{
    if (m_bInit == false)
    {
        // general
        m_imgWidth = width;
        m_imgHeight = height;
        m_inputFrames = 0;
        // cache part
        m_curFrontIdx = 0;
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_cacheFrames[k].create(width, height, CV_8UC3); // rgb
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_bookResults[k].create(width, height, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES - 1; k++)
            m_diffResults[k].create(width, height, CV_8UC1); // gray
        // contour
        m_objIdx = 0;        
        m_bInit = true;
    }
    return 0;    
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
// |><| ************************************************************************
// processFrame:
//     1. Do diff(OR operation) of two frames, store the 'diffResults';
//     2. Do AND operation of 'diffResults' to get the overlap;
//     3. using crossLines(boundary) to emit new contourTrack;
//     4. using existing 'contourTrack' with 'diffResults' to update contourTrack;
// args:
//     outs: output each contourTrack's object rectangle;
//     bookResult: psoBook's result of background/foreground;
//     lines: possible new objects cross boundary lines got from 'BoundaryScan';
// return:
//     = 0, won't output frames;
//     > 0, output one frame;
//     < 0, process error;
// *****************************************************************************
int ThreeDiff :: processFrame(const cv::Mat & in,
                              const cv::Mat & bookResult,
                              const vector<std:tuple<TDPoint, TDPoint> > & lines,
                              vector<cv::Rect> & outs)
{
    m_inputFrames++;
    if (m_inputFrames <= M_THREE_DIFF_CACHE_FRAMES)
    {
        in.copyTo(m_cacheFrames[m_inputFrames-1]);
        bookResults.copyTo(m_bookResults[m_inputFrames-1]);
        m_crossLines[m_inputFrames-1] = lines;
        if (m_inputFrames > 1)
        {   
            doRgbDiff(m_cacheFrames[m_inputFrames-1], m_cacheFrames[m_inputFrames-2]);
            m_curFrontIdx++;
            if (m_curFrontIdx % M_THREE_DIFF_CACHE_FRAMES == 0)
                m_curFrontIdx = 0;
        }
        return 0;
    }
    // 1. do diff in RGB for Contour's using.
    doRgbDiff(in, m_cacheFrames[m_curFrontIdx]);
    // fill the outs with 
    doUpdateContourTracking(outs);
    
    // 2. do boundary check for creating new Contour.
    doCreateNewContourTrack(outs);

    // 3. do update internal cache/status
    doUpdateThreeDiffAfterOneFrameProcess(in, bookResult, lines);
    return 1; // output 1 frame.
}

int ThreeDiff :: flushFrame(cv::Mat & out)
{
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    

int ThreeDiff :: doRgbDiff(const cv::Mat & first, const cv::Mat & second)
{
    for (int k = 0; k < m_imgHeight; k++)
    {
        for (int j = 0; j < m_imgWidth; j++)
        {
            vector<uchar> input;
            const cv::Vec3b & firstIntensity = first.at<cv::Vec3b>(k, j);
            const cv::Vec3b & secondIntensity = second.at<cv::Vec3b>(k, j);
            if (rgbEulerDiff(firstIntensity, secondIntensity) == true)
                m_diffResults[m_curFrontIdx].at<uchar>(k, j) = 255;
            else
                m_diffResults[m_curFrontIdx].at<uchar>(k, j) = 0;
        }
    }
    
    return 0;
}

bool ThreeDiff :: rgbEulerDiff(const cv::Vec3b & first, const cv::Vec3b & second)
{
    assert(vs1.components().size() == 3 && vs2.components().size() == 3);
    int meanRed = (first[0] + second[0]) / 2;
    int r =  first[0] - second[0];
    int g =  first[1] - second[1];
    int b =  first[2] - second[2];
    // TODO: 20?
    return sqrt((((512 + meanRed)*r*r)>>8) + 4*g*g + (((767-meanRed)*b*b)>>8)) > 20;
}

int ThreeDiff :: doUpdateThreeDiffAfterOneFrameProcess(
    const cv::Mat in, const cv::Mat & bookResult, lines);
    
} // namespace Seg_Three
