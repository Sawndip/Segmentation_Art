#include <algorithm> // std::sort
#include "ThreeDiff.h"

using namespace cv;

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
                              cv::Mat & bookResult, // also, it is the out binary frame.
                              const vector<std:tuple<TDPoint, TDPoint> > & curLines,
                              vector<cv::Rect> & rects)
{
    m_inputFrames++;
    if (m_inputFrames <= M_THREE_DIFF_CACHE_FRAMES)
    {
        in.copyTo(m_cacheFrames[m_inputFrames-1]);
        bookResults.copyTo(m_bookResults[m_inputFrames-1]);
        m_crossLines[m_inputFrames-1] = curLines;
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

    // 2. fill the 'outs' with 'lines', 'diffResult', 'simplified optical flow' 
    doUpdateContourTracking(bookResult, curLines, rects);
    
    // 3. do boundary check for creating new Contour.
    doCreateNewContourTrack(rects);

    // 4. do update internal cache/status
    doUpdateThreeDiffAfterOneFrameProcess(in, bookResult, curLines);
    return 1; // output 1 frame.
}

int ThreeDiff :: flushFrame(cv::Mat & out)
{
    // for the cache frames.
    
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers
/****************************
Lines, BookResults, DiffResults, CacheFrames: all these parameters are used for catch
the objects' moving locations in every frame and their estimated size.

When object enter or exit the screeen, location and size are special. 
So we use CrossLines to do some special processing: namely tell its enter/exist 
together with ContourTrack's status.

1. the purpose of Lines:
   1) to create new contour (m_directionIn);
   2) to get the largest size of the objects (m_bAllIn);
   3) to know the vanishing of the objects in screen (m_directionOut);

2. the purpose of DiffResults:
   1) extract features from the diff overlap, for instance center color, edge points
      search it as using optical flow, then we can get MV (Using MV for next estimate 
      searching), to finally know whether the object still in the screen and its 
      estimate size.

3. the purpose of BookResults:
   to verify the Simplified Optical Flow's result. If they are quit different, we would
   like to discard SOF's results.

4. the purpose of CacheFrame:
   1) provides RGB feature extraction;
   2) others?   

****************************/

// |><| ************************************************************************
// doCreateNewContourTrack:
//     1. Using m_crossLines to check new coming in objects
//     2. line1's(oldest line) TDPoints can be removed in this call
// args:
//     outs: output new contourTrack's object rectangle;
//     curLines: this frames ?? 
// return:
//     >= 0, process ok;
//     < 0, process error;
// *****************************************************************************        
int ThreeDiff :: doUpdateContourTracking(cv::Mat & out,
                                         vector<std:tuple<TDPoint, TDPoint> > & curLines)
{
    if (m_tracks.size() == 0)
        return 0;
    // how can we do tracking together with boundary changing!
        
    return 0;
}
    
// |><| ************************************************************************
// doCreateNewContourTrack:
//     1. Using m_crossLines to check new coming in objects
//     2. line1's(oldest line) TDPoints can be removed in this call
//     3. only deal with enter objects. Exit objects handle in 'doUpdateContourTracking'.
// args:
//     outs: output new contourTrack's object rectangle;
// return:
//     >= 0, process ok;
//     < 0, process error;
// *****************************************************************************    
int ThreeDiff :: doCreateNewContourTrack(cv::Mat & out,
                                         const vector<std:tuple<TDPoint, TDPoint> > & lines3)
{
    // check the three lines, do 'AND' operation
    vector<vector<std:tuple<TDPoint, TDPoint> > > & lines1 = m_crossLines[0];
    vector<vector<std:tuple<TDPoint, TDPoint> > > & lines2 = m_crossLines[1];
    assert(lines1.size() == lines2.size() && lines1.size() == lines3.size());
    for (int k=0; k < (int)lines1.size(); k++)
    {
        auto it1 = lines1[k].begin();
        auto it2 = lines2[k].begin();
        auto it3 = lines3[k].begin();    
        // 1. first we kick out unlikely Points in lines1
        vector<int> starts;
        vector<int> ends;        
        for (;it1 != lines1[k].end(); it1++)
        {
            starts.push_back(std::get<0>(*t1).y);
            starts.push_back(std::get<1>(*t1).y);            
        }           
        for (;it2 != lines2[k].end(); it2++)
        {
            starts.push_back(std::get<0>(*t2).y);
            starts.push_back(std::get<1>(*t2).y);            
        }           
        for (;it3 != lines3[k].end(); it3++)
        {
            starts.push_back(std::get<0>(*t3).y);
            starts.push_back(std::get<1>(*t3).y);            
        }           
        std::sort(starts.begin(), starts.end());
        std::sort(ends.begin(), ends.end());

        vector<std:tuple<TDPoint, TDPoint> > newEnters;
        int startBegint = 1;
        for (int i = 0; i < (int)ends.size(); i++)
        {
            TDPoint start, end;
            for (int j = startBegin; j < (int)starts.size(); j++)
            {
                if (starts[j] >= ends[i] && starts[j-1] < ends[i])
                {
                    startBegin = j;
                    start.x = 0;
                    start.y = starts[j-1];
                    end.x = 0;
                    end.y = ends[i];
                    newEnters.push_back(make_tuple(start, end));
                    break;
                }
            }
            // we get possible newEnters, so just create new objects.
            for (int m = 0; m < (int)newEnters.size(); m++)
            {
                int h = 0;
                for (h = 0; h < (int)lines1.size(); h++)
                {   // newEnters[m] is part of lines1[h]
                    if (isYContainedBy(newEnters[m], lines1[h] == true))
                        break;
                }
                if (h == (int)lines1.size()) // not part of lines1
                    continue;
                    
                const int length = std::get<1>(newEnters[m]).y - ;
                // we calculate the lux/luy, possible width/height
                int lux = 0, luy = 0, possibleWidth = 0;
                const int possibleHeight = 6; // fixed value, 6 pixels when first craeted
                switch(k)
                {
                case 0: // top
                    lux = std::get<0>(newEnters[m]).y; // start point
                    luy = 0;
                    possibleWidth = std::get<1>(newEnters[m]).y - lux;
                    break;                    
                case 1: // bottom
                    lux = std::get<0>(newEnters[m]).y;
                    luy = m_imgHeight - possibleHeight;
                    possibleWidth = std::get<1>(newEnters[m]).y - lux;
                    break;                    
                case 2: // left
                    lux = 0;
                    luy = std::get<0>(newEnters[m]).y;
                    possibleWidth = std::get<1>(newEnters[m]).y - luy;
                    break;                    
                case 3: // right
                    lux = m_imgWidth - possibleWidth;
                    luy = std::get<0>(newEnters[m]).y;;
                    possibleWidth = std::get<1>(newEnters[m]).y - luy;                    
                    break;
                }
                ContourTrack *pTrack = new ContourTrack(m_imgWidth, m_imgHeight,
                                                        k, lux, luy,
                                                        possibleWidth, possibleHeight);
                m_tracks.push_back();
                out.push_back();
            }
        }
    }
    return 0;
}
    
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
    // TODO: 20? what else values ?
    return sqrt((((512 + meanRed)*r*r)>>8) + 4*g*g + (((767-meanRed)*b*b)>>8)) > 20;
}
    
int ThreeDiff :: doUpdateThreeDiffAfterOneFrameProcess(
                         const cv::Mat in, const cv::Mat & bookResult,
                         const vector<std:tuple<TDPoint, TDPoint> > & lines3)
{


}
    
} // namespace Seg_Three
