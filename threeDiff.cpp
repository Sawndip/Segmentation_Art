#include <algorithm> // std::sort
#include "threeDiff.h"

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
            m_bgResults[k].create(width, height, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffAndResults[k].create(width, height, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffOrResults[k].create(width, height, CV_8UC1); // gray
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
//     bgResult: psoBg's result of background/foreground;
//     lines: possible new objects cross boundary lines got from 'BoundaryScan';
// return:
//     = 0, won't output frames;
//     > 0, output one frame;
//     < 0, process error;
// *****************************************************************************
int ThreeDiff :: processFrame(const cv::Mat & in,
                              cv::Mat & bgResult, // also, it is the out binary frame.
                              FourBorders & curFourLines,
                              vector<SegResults> & segResults)
{
    m_inputFrames++;
    // 0. do preprocess: cache frames
    if (m_inputFrames <= M_THREE_DIFF_CACHE_FRAMES)
    {
        bgResult.copyTo(m_bgResults[m_inputFrames-1]);
        m_crossLines[m_inputFrames-1] = curFourLines;
        if (m_inputFrames > 1)
        {   
            doBgDiff(m_bgResults[m_inputFrames-1], m_bgResults[m_inputFrames-2]);
            m_curFrontIdx++;
            if (m_curFrontIdx % M_THREE_DIFF_CACHE_FRAMES == 0)
                m_curFrontIdx = 0;
        }
        return 0;
    }
    
    // 1. do diff in RGB for Contour's using.
    doBgDiff(bgResult, m_bgResults[m_curFrontIdx]);
    // 2. fill the 'outs' with 'lines', 'diffResult', 'simplified optical flow' 
    doUpdateContourTracking(in, bgResult, curFourLines, segResults);   
    // 3. do boundary check for creating new Contour.
    doCreateNewContourTrack(in, bgResult, curFourLines, segResults);
    // 4. do update internal cache/status
    updateAfterOneFrameProcess(in, bgResult, curFourLines);
    
    // output 1 frame
    return 1;
}

int ThreeDiff :: flushFrame(vector<SegResults> & segResults, cv::Mat & out)
{
    // for the cache frames.
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers

// |><| ************************************************************************
// doUpdateContourTracking:
//     1. Using m_crossLines to check new coming in objects
//     2. line1's(oldest line) TDPoints can be removed in this call
// args:
//     outs: output new contourTrack's object rectangle;
//     curFourLines: this frames ?? 
// return:
//     >= 0, process ok;
//     < 0, process error;
// *****************************************************************************        
int ThreeDiff :: doUpdateContourTracking(const cv::Mat in, cv::Mat & bgResult,
                                         FourBorders & curFourLines,
                                         vector<SegResults> & segResults)
{
    if (m_trackers.size() == 0)
        return 0;
    // 1. leaving boundary check
    // 2. new curBox amendment.
    // 3. do boundary points kicking
    for (auto it = m_trackers.begin(); it != m_trackers.end(); /*No it++, do it inside loop*/)
    {
        SegResults sr;
        // re-calc the curBox, calculate the boundary cross part.
        int ret = (*it)->processFrame(in, bgResult, m_diffAndResults[m_curFrontIdx],
                                      m_diffOrResults[m_curFrontIdx]);
        if (ret < 0)
            LogW("Process failed.\n");
        else if (ret == 1) // all out
        {
            // TODO: other updates??
            delete *it; // delete this ContourTrack
            m_trackers.erase(it); // erase it from the vector.
        }
        else // ok, just do post update
        {
            sr.m_objIdx = (*it)->getIdx();
            sr.m_bOutForRecognize = (*it)->canOutputRegion();
            sr.m_curBox = (*it)->getCurBox();
            // kick the points
            if ((*it)->isAllIn() == false)
                kickOverlapPoints(sr.m_curBox, curFourLines, (*it)->getInDirection());
            it++; // increse here.
        }
        segResults.push_back(sr);
    }
        
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
int ThreeDiff :: doCreateNewContourTrack(const cv::Mat & in, cv::Mat & bgResult,
                                         vector<vector<tuple<TDPoint, TDPoint> > > & lines3,
                                         vector<SegResults> & segResults)
{
    // check the three lines, do 'AND' operation
    vector<vector<tuple<TDPoint, TDPoint> > > & lines1 = m_crossLines[0];
    vector<vector<tuple<TDPoint, TDPoint> > > & lines2 = m_crossLines[1];
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
            starts.push_back(std::get<0>(*it1).x);
            starts.push_back(std::get<1>(*it1).x);            
        }           
        for (;it2 != lines2[k].end(); it2++)
        {
            starts.push_back(std::get<0>(*it2).x);
            starts.push_back(std::get<1>(*it2).x);            
        }           
        for (;it3 != lines3[k].end(); it3++)
        {
            starts.push_back(std::get<0>(*it3).x);
            starts.push_back(std::get<1>(*it3).x);            
        }
        std::sort(starts.begin(), starts.end());
        std::sort(ends.begin(), ends.end());

        vector<tuple<TDPoint, TDPoint> > newEnters;
        int startBegin = 1;
        for (int i = 0; i < (int)ends.size(); i++)
        {
            TDPoint start, end;
            for (int j = startBegin; j < (int)starts.size(); j++)
            {
                if (starts[j] >= ends[i] && starts[j-1] < ends[i])
                {
                    startBegin = j;
                    start.x = starts[j-1];
                    start.y = 0;
                    end.x = ends[i];
                    end.y = 0;
                    newEnters.push_back(std::make_tuple(start, end));
                    break;
                }
            }
            // we get possible newEnters, so just create new objects.
            for (int m = 0; m < (int)newEnters.size(); m++)
            {
                int h = 0;
                for (h = 0; h < (int)lines1[k].size(); h++)
                {   // newEnters[m] is part of lines1[h]
                    if (isXContainedBy(newEnters[m], lines1[k][h]) == true)
                        break;
                }
                if (h == (int)lines1[k].size()) // not part of lines1
                    continue;
                    
                // we calculate the lux/luy, possible width/height
                int lux = 0, luy = 0, possibleWidth = 0;
                // fixed value, 8 pixels when first craeted, a little bigger than actual 6.
                const int possibleHeight = 8; 
                switch(k)
                {
                case 0: // top
                    lux = std::get<0>(newEnters[m]).x; // start point
                    luy = 0;
                    possibleWidth = std::get<1>(newEnters[m]).x - lux;
                    break;                    
                case 1: // bottom
                    lux = std::get<0>(newEnters[m]).x;
                    luy = m_imgHeight - possibleHeight;
                    possibleWidth = std::get<1>(newEnters[m]).x - lux;
                    break;                    
                case 2: // left
                    lux = 0;
                    luy = std::get<0>(newEnters[m]).x;
                    possibleWidth = std::get<1>(newEnters[m]).x - luy;
                    break;                    
                case 3: // right
                    lux = m_imgWidth - possibleWidth;
                    luy = std::get<0>(newEnters[m]).x;;
                    possibleWidth = std::get<1>(newEnters[m]).x - luy;                    
                    break;
                }
                ContourTrack *pTrack = new ContourTrack(m_objIdx, in,
                                                        m_imgWidth, m_imgHeight,
                                                        k, lux, luy,
                                                        possibleWidth, possibleHeight);
                m_trackers.push_back(pTrack);
                SegResults sr;
                sr.m_objIdx = m_objIdx;
                sr.m_bOutForRecognize = false;
                sr.m_curBox = cv::Rect(lux, luy, possibleWidth, possibleHeight);
                m_objIdx++;
                segResults.push_back(sr);
            }
        }
    }
    return 0;
}

int ThreeDiff :: updateAfterOneFrameProcess(const cv::Mat in,                               
           const cv::Mat & bgResult, const vector<vector<tuple<TDPoint, TDPoint> > > & lines3)
{ 
    // diff, in, bgResult, crossLines
    m_curFrontIdx++;
    m_curFrontIdx = m_curFrontIdx % M_THREE_DIFF_CACHE_FRAMES == 0 ? 0 : m_curFrontIdx;
    bgResult.copyTo(m_bgResults[m_curFrontIdx]);
    m_crossLines[m_curFrontIdx] = lines3;
    return 0;
}

//// other helpers
int ThreeDiff :: doBgDiff(const cv::Mat & first, const cv::Mat & second)
{
    for (int k = 0; k < m_imgHeight; k++)
    {
        for (int j = 0; j < m_imgWidth; j++)
        {
            m_diffAndResults[m_curFrontIdx].at<uchar>(k, j) =
                first.at<uchar>(k, j) & second.at<uchar>(k, j);
            m_diffOrResults[m_curFrontIdx].at<uchar>(k, j) =
                first.at<uchar>(k, j) | second.at<uchar>(k, j);
        }
    }    
    return 0;
}

// if an object is just entering the border, we should kick out boundary scan's points
// that inside object's area.
int ThreeDiff :: kickOverlapPoints(const cv::Rect & box,
                                   FourBorders & curFourLines, const DIRECTION direction)
{
    TDPoint start;
    TDPoint end;
    switch(direction)
    {
    case TOP:
    case BOTTOM:        
        start.x = box.x;
        start.y = 0;
        end.x = box.x + box.width;
        end.y = 0;        
        break;
    case LEFT:
    case RIGHT:        
        start.x = box.y;
        start.y = 0;
        end.x = box.y + box.height;
        end.y = 0;
        break;
    default:
        LogW("We don't know the direction, cannot kick out points.");        
        return -1;
    }

    // now, change or kick out the points between [start, end]
    vector<tuple<TDPoint, TDPoint> > & oneBorder = curFourLines[direction];
    for (auto it = oneBorder.begin(); it != oneBorder.end(); /*it++ no increment*/)
    {
        if (std::get<1>(*it).x < start.x || std::get<0>(*it).x >= end.x)
            it++;
        else if (std::get<0>(*it).x < start.x && std::get<1>(*it).x > end.x)
        {
            TDPoint end0 = start;
            TDPoint start1 = end;
            oneBorder.insert(it, std::make_tuple(std::get<0>(*it), end0));
            oneBorder.insert(it, std::make_tuple(start1, std::get<1>(*it)));
            oneBorder.erase(it); // insert two, remove one.
            // no need it++, for we already do erase.
        }
        else if (std::get<0>(*it).x < start.x) // change it.
        {
            std::get<1>(*it) = start; // end point as the start.
            it++;
        }
        else if (std::get<1>(*it).x > end.x) // change it.
        {
            std::get<0>(*it) = end; // end point as the start.
            it++;
        }
        else // inside [start, end]
            oneBorder.erase(it);
    }

    return 0;
}
    
} // namespace Seg_Three    
////////////////////////////// End of File //////////////////////////////////////////




/*    
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
    int meanRed = (first[0] + second[0]) / 2;
    int r =  first[0] - second[0];
    int g =  first[1] - second[1];
    int b =  first[2] - second[2];
    // TODO: 20? what else values ?
    return sqrt((((512 + meanRed)*r*r)>>8) + 4*g*g + (((767-meanRed)*b*b)>>8)) > 20;
}    
*/    
