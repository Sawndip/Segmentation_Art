#include "boundaryScan.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
BoundaryScan :: BoundaryScan()
{
    return;
}

BoundaryScan :: ~BoundaryScan()
{
    return;        
}

int BoundaryScan :: init(const int width, const int height,
                         const int skipTB, const int skipLR,
                         const int scanSizeTB, const int scanSizeLR)
{
    m_imgWidth = width;
    m_imgHeight = height;
    m_inputFrames = 0;
    m_skipTB = skipTB;
    m_skipLR = skipLR;
    m_scanSizeTB = scanSizeTB;
    m_scanSizeLR = scanSizeLR;
    //normaly heightTB=heightLR=2, widthTB=imgWidth, widthLR=imgHeight
    m_bordersMem.init(m_imgWidth - 2 * skipTB, scanSizeTB,
                      m_imgHeight - 2 * skipLR, scanSizeLR);
    
    return 0;    
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs

/***************************************************************************
processFrame:
   do simplified Erode/dilate of two lines: 2x2 window slide the lines; 
   get the lines that are foreground across the boundary.
args:
   in: the binary(0 or 255) of the background / foreground values of the image
   lines: output result, the foreground lines that just in borders
return:   
****************************************************************************/
int BoundaryScan :: processFrame(const BgResult & bgResult)
{
    m_inputFrames++;
    // 0. prepare
    assert(bgResult.binaryData.channels() == 1);
    assert((int)bgResult.binaryData.step[0] == m_imgWidth &&
           (int)bgResult.binaryData.step[1] == (int)sizeof(unsigned char));
    assert(m_scanSizeTB == m_bordersMem.heightTB &&
           m_scanSizeLR == m_bordersMem.heightLR );

    // 1. first extract border data from bgResult
    for (int k = 0; k < m_scanSizeTB; k++)
    {
        memcpy(m_bordersMem.directions[0] + k*m_bordersMem.widthTB,
               bgResult.binaryData.data + m_imgWidth * (k+m_skipTB) + m_skipLR, 
               m_imgWidth - 2*m_skipLR);
    
        memcpy(m_bordersMem.directions[1] + k*m_bordersMem.widthTB,
               bgResult.binaryData.data + m_imgWidth*(m_imgHeight-m_skipTB-k-1) + m_skipLR,
               m_imgWidth - 2*m_skipLR);
    }
    // for left & right data
    for (int k = 0; k < m_scanSizeLR; k++)
    {
        for (int j = 0; j < m_bordersMem.widthLR; j++)
        {   
            m_bordersMem.directions[2][k*m_bordersMem.widthLR+j] = // left
                bgResult.binaryData.at<uchar>(j+m_skipTB, k+m_skipLR);
            m_bordersMem.directions[3][k*m_bordersMem.widthLR+j] = // right
                bgResult.binaryData.at<uchar>(j+m_skipTB, m_imgWidth-m_skipLR-k-1);
        }
    }

    // 2. we do open / close: seems for simplified erode/dilate, just open is ok.    
    // for (int k = 0; k < 2; k++)
    {   // oepn: erode then dilate
        doErode();
        doDilate();            
        // close: dilate then erode
        doDilate(2);
        doErode(2);
    }
    
    // 3. scan the border, get the TDPoint of the lines & merge short-lines
    scanBorders(bgResult);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers

//simplified Erode/dilate
int BoundaryScan :: doErode(const int times)
{
    // NOTE: following code just deal with 2x2 window! Be aware of it.
    for (int t = 0; t < times; t++)
    {
        int width = m_bordersMem.widthTB, height = m_bordersMem.heightTB;
        for (int n = 0; n < 4; n++)
        {   // note k = k + M_ELEMENT_HEIGHT
            if (n >= 2)
            {
                width = m_bordersMem.widthLR;            
                height = m_bordersMem.heightLR;
            }
         
            for (int k = 0; k < height; k+=M_ELEMENT_HEIGHT)
            {
                for (int j = 0; j < width - 1; j++)
                {
                    m_bordersMem.directions[n][k*width + j] =
                      m_bordersMem.directions[n][(k+1)*width + j] =
                        m_bordersMem.directions[n][k*width + j]     &
                        m_bordersMem.directions[n][(k+1)*width + j] &
                        m_bordersMem.directions[n][k*width + j+1]   & 
                        m_bordersMem.directions[n][(k+1)*width + j+1];
                    if (j == width - M_ELEMENT_WIDTH)
                        m_bordersMem.directions[n][k*width + j+1] =
                            m_bordersMem.directions[n][(k+1)*width + j+1] =
                                m_bordersMem.directions[n][k*width + j+1]   &
                                m_bordersMem.directions[n][(k+1)*width + j+1];
                    //if (m_bordersMem.directions[n][k*width + j] == 0xFF)
                    //    LogI("One point 255, x=%d, y=%d\n", j, k);
                }
            }
        }
    }
    return 0;
}

int BoundaryScan :: doDilate(const int times)
{
    for (int t = 0; t < times; t++)
    {
        int width = m_bordersMem.widthTB, height = m_bordersMem.heightTB;        
        for (int n = 0; n < 4; n++)
        {   // note k = k + M_ELEMENT_HEIGHT
            if (n >= 2)
            {
                height = m_bordersMem.heightLR;
                width = m_bordersMem.widthLR;
            }
            for (int k = 0; k < height; k+=M_ELEMENT_HEIGHT)
            {
                for (int j = 0; j < width - 1; j++)
                {
                    m_bordersMem.directions[n][k*width + j] =
                      m_bordersMem.directions[n][(k+1)*width + j] =
                        m_bordersMem.directions[n][k*width + j]     |
                        m_bordersMem.directions[n][(k+1)*width + j] |
                        m_bordersMem.directions[n][k*width + j+1]   | 
                        m_bordersMem.directions[n][(k+1)*width + j+1];
                    if (j == width - M_ELEMENT_WIDTH)
                        m_bordersMem.directions[n][k*width + j+1] =
                            m_bordersMem.directions[n][(k+1)*width + j+1] =
                                m_bordersMem.directions[n][k*width + j+1]   |
                                m_bordersMem.directions[n][(k+1)*width + j+1];
                }
            }
        }
    }
    return 0;
}

int BoundaryScan :: scanBorders(BgResult & bgResult)
{
    // we get borders with erode/dilate, then we get the foreground bgResult.lines.
    LogI("Scan the border.\n");
    int width = m_bordersMem.widthTB;
    for (int n = 0; n < 4; n++)
    {
        if (n >= 2) // for left/right borders
            width = m_bordersMem.widthLR;

        TDLine line;
        bool bStart = false;
        for (int k = 0; k < width; k++)
        {
            if (bStart == false && m_bordersMem.directions[n][k] == 0xFF)
            {
                bStart = true;
                line.a.x = k;
                line.a.y = 0;
            }
            if (bStart == true && m_bordersMem.directions[n][k] != 0xFF)
            {
                bStart = false;
                line.b.x = k;
                line.b.y = 0;
                bgResult.lines[n].push_back(line);
                LogI("Get One Line: n-%d, %d to %d\n", n, line.a.x, line.b.x);
                bStart = false;
            }
        }
        // for these are close enough bgResult.lines, we merge them.
        // Big chance they are the same object.
        premergeLines(bgResult, n);
    }
    return 0;
}

// 1. merge short-lines that we can be sure they are parts of the same objects.
// 2. For lines we process here are lines with no overlap, namely, l1.b.x < l2.a.x.
//    So we can make use of this property. 
int BoundaryScan :: premergeLines(BgResult & bgResult, const int index)
{
    vector<TDLine> & lines = bgResult.lines[index];
    if (lines.size() < 2) // no need merging
        return 0;

    vector<double> & xMvs = bgResult.xMvs[index];
    vector<double> & yMvs = bgResult.yMvs[index];    
    
    for (auto it = lines.begin(); it != lines.end(); /*No increment*/)
    {
        auto nextIt = it + 1;
        if (nextIt == line.end())
            break;
        else
        {
            if (canLinesBeMerged(*it, *nextIt, xMvs, yMvs) == true)
            {
                nextIt->a = it->a;
                lines.erase(it);
            }
            else
                it++;
        }
    }
    return 0;
}

// using mvs & also the line's x direction distance    
// if 1. xDistance is less than 2% of the imgWidth + imgHeight
//    2. xDistance is less than 20% of the 'line1's length + line2's length'
//    3. mvs are quit close (angle less than 10 degree): weight 60;
// then we can merge this two lines.    
bool BoundaryScan :: canLinesBeMerged(const TDLine & l1, const TDLine & l2,
                                      const vector<double> & xMvs,
                                      const vector<double> & yMvs)
{
    assert(l1.b.x < l2.a.x); // end's point < start's point.
    const int xDistance = l2.a.x - l1.b.x;
    if (1.0 * xDistance / (m_imgWidth + m_imgHeight) > 0.02)
        return false;
    const int line1len = l1.b.x - l1.a.x;
    const int line2len = l2.b.x - l2.a.x;    
    if (1.0 * xDistance / (line1len + line2len) > 0.2)
        return false;
    
    if ()
    
    return true;
}
    
} // namespace Seg_Three
