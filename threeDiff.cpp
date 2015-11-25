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
            m_bgResults[k].create(height, width, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffAndResults[k].create(height, width, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffOrResults[k].create(height, width, CV_8UC1); // gray
        // crossLines won't be init here.
        // ContourTrack
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
                              const cv::Mat & bgResult,
                              FourBorders & curFourLines,
                              vector<SegResults> & segResults)
{
    m_inputFrames++;
    // 0. do preprocess: cache frames
    if (m_inputFrames <= M_THREE_DIFF_CACHE_FRAMES)
    {
        bgResult.copyTo(m_bgResults[m_inputFrames-1]);
        m_crossLines[m_inputFrames-1] = curFourLines;        
        //copyLines(curFourLines, m_crossLines[m_inputFrames-1]);
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

int ThreeDiff :: flushFrame(vector<SegResults> & segResults)
{
    // for the cache frames.
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers

// |><| **********************************************************************************
// doUpdateContourTracking:
//     1. Using m_crossLines to check new coming in objects
//     2. line1's(oldest line) TDPoints can be removed in this call
// args:
//     outs: output new contourTrack's object rectangle;
//     curFourLines: this frames ?? 
// return:
//     >= 0, process ok;
//     < 0, process error;
// ***************************************************************************************
int ThreeDiff :: doUpdateContourTracking(const cv::Mat in, const cv::Mat & bgResult,
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
//     2. only deal with enter objects. Exit objects handle in 'doUpdateContourTracking'.
// args:
//     outs: output new contourTrack's object rectangle;
// return:
//     >= 0, process ok;
//     < 0, process error;
// *****************************************************************************    
int ThreeDiff :: doCreateNewContourTrack(const cv::Mat & in,
                                         const cv::Mat & bgResult,
                                         FourBorders & lines3,
                                         vector<SegResults> & segResults)
{
    // check the three lines, do 'AND' operation
    const int oldIdx = (m_curFrontIdx - 1) < 0 ?
                          M_THREE_DIFF_CACHE_FRAMES - 1 : m_curFrontIdx - 1;
    FourBorders & lines1 = m_crossLines[oldIdx];
    FourBorders & lines2 = m_crossLines[m_curFrontIdx];
    
    assert(lines1.m_widthTB == lines2.m_widthTB && lines2.m_widthTB == lines3.m_widthTB);
    for (int k=0; k < 4; k++)
    {
        // 1. first we kick out unlikely Points in lines1
        vector<int> starts;
        vector<int> ends;
        for (int j = 0; j < (int)lines1.m_lines[k].size(); j++)
        {
            starts.push_back(lines1.m_lines[k][j].a.x);
            ends.push_back(lines1.m_lines[k][j].b.x);            
        }           
        for (int j = 0; j < (int)lines2.m_lines[k].size(); j++)
        {
            starts.push_back(lines2.m_lines[k][j].a.x);
            ends.push_back(lines2.m_lines[k][j].b.x);            
        }           
        for (int j = 0; j < (int)lines3.m_lines[k].size(); j++)
        {
            starts.push_back(lines3.m_lines[k][j].a.x);
            ends.push_back(lines3.m_lines[k][j].b.x);            
        }           

        std::sort(starts.begin(), starts.end());
        std::sort(ends.begin(), ends.end());
        auto it1 = std::unique(starts.begin(), starts.end());
        starts.resize(std::distance(starts.begin(), it1));
        auto it2 = std::unique(ends.begin(), ends.end());
        ends.resize(std::distance(ends.begin(), it2));
        
        vector<TDLine> newEnters;
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
                    newEnters.push_back(TDLine(start, end));
                    break;
                }
            }
            
            // we get possible newEnters, so just create new objects.
            for (int m = 0; m < (int)newEnters.size(); m++)
            {
                int h = 0;
                for (h = 0; h < (int)lines1.m_lines[k].size(); h++)
                {   // newEnters[m] is part of lines1[h]
                    if (isXContainedBy(newEnters[m], lines1.m_lines[k][h]) == true)
                        break;
                }
                if (h == (int)lines1.m_lines[k].size()) // not part of lines1
                    continue;
                    
                // we calculate the lux/luy, possible width/height
                int lux = 0, luy = 0, possibleWidth = 0;
                // fixed value, 8 pixels when first craeted, a little bigger than actual 6.
                const int possibleHeight = 8; 
                switch(k)
                {
                case 0: // top
                    lux = newEnters[m].a.x; // start point
                    luy = 0;
                    possibleWidth = newEnters[m].b.x - lux;
                    break;                    
                case 1: // bottom
                    lux = newEnters[m].a.x;
                    luy = m_imgHeight - possibleHeight;
                    possibleWidth = newEnters[m].b.x - lux;
                    break;                    
                case 2: // left
                    lux = 0;
                    luy = newEnters[m].a.x;
                    possibleWidth = newEnters[m].b.x - luy;
                    break;                    
                case 3: // right
                    lux = m_imgWidth - possibleHeight;
                    luy = newEnters[m].a.x;;
                    possibleWidth = newEnters[m].b.x - luy;                    
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

int ThreeDiff :: updateAfterOneFrameProcess(const cv::Mat in, const cv::Mat & bgResult,
                                            const FourBorders & lines3)
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
    vector<TDLine> & oneBorder = curFourLines.m_lines[direction];
    for (auto it = oneBorder.begin(); it != oneBorder.end(); /*it++ no increment*/)
    {
        if ((*it).b.x < start.x || (*it).a.x >= end.x)
            it++;
        else
            oneBorder.erase(it);
        //else if ((*it).a.x < start.x && (*it).b.x > end.x)
        //{
        //    TDPoint end0 = start;
        //    TDPoint start1 = end;
        //    oneBorder.insert(it, TDLine((*it).a, end0));
        //    oneBorder.insert(it, TDLine(start1, (*it).b));
        //    oneBorder.erase(it); // insert two, remove one.
        //    // no need it++, for we already do erase.
        //}
        //else if ((*it).a.x < start.x) // change it.
        //{
        //    (*it).b = start; // end point as the start.
        //    it++;
        //}
        //else if ((*it).b.x > end.x) // change it.
        //{
        //    (*it).a = end; // end point as the start.
        //    it++;
        //}
        //else // inside [start, end]
        //    oneBorder.erase(it);
    }

    return 0;
}
    
} // namespace Seg_Three    
////////////////////////////// End of File //////////////////////////////////////////
