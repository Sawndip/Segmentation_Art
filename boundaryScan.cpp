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

int BoundaryScan :: init(const int width, const int height)
{
    m_imgWidth = width;
    m_imgHeight = height;
    m_inputFrames = 0;
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
int BoundaryScan :: processFrame(const cv::Mat & bgResult, FourBorders & lines)
{
    m_inputFrames++;
    // 0. prepare
    assert(bgResult.channels() == 1);
    assert((int)bgResult.step[0] == m_imgWidth &&
           (int)bgResult.step[1] == (int)sizeof(unsigned char));
    if (m_bordersMem.bInit == false)
    {
        //normaly heightTB=heightLR=2, widthTB=imgWidth, widthLR=imgHeight
        m_bordersMem.init(lines.m_widthTB, lines.m_heightTB,
                          lines.m_widthLR, lines.m_heightLR);
        m_directions[0] = m_bordersMem.top;
        m_directions[1] = m_bordersMem.bottom;
        m_directions[2] = m_bordersMem.left;
        m_directions[3] = m_bordersMem.right;    
    }
    // 1. first extract border data from bgResult    
    memcpy(m_bordersMem.top,
           bgResult.data + lines.m_skipT*lines.m_widthTB,
           lines.m_widthTB * lines.m_heightTB);
    memcpy(m_bordersMem.bottom,
           bgResult.data + bgResult.step[0]*(m_imgHeight-lines.m_skipB-lines.m_widthTB),
           lines.m_widthTB * lines.m_heightTB);
    
    // for left & right data
    for (int k = 0; k < lines.m_heightLR; k+=2) //note +=2
    {
        for (int j = 0; j < lines.m_widthLR; j++)
        {
            m_bordersMem.left[k*lines.m_widthLR+j] = bgResult.at<uchar>(j, k);
            m_bordersMem.left[(k+1)*lines.m_widthLR+j] = bgResult.at<uchar>(j, k+1);
            m_bordersMem.right[k*lines.m_widthLR+j] =
                bgResult.at<uchar>(j, m_imgWidth-k-1);
            m_bordersMem.right[(k+1)*lines.m_widthLR+j] =
                bgResult.at<uchar>(j, m_imgWidth-(k+1)-1);
        }
    }

    // 2. we do open / close: seems for simplified erode/dilate, just open is ok.    
    for (int k = 0; k < 2; k++)
    {   // oepn: erode then dilate
        doErode();
        doDilate();            
        // close: dilate then erode
        doDilate();
        doErode();
    }
    // make the close enough part connected
    for (int k = 0; k < 2; k++)
    {   
        doDilate();
        doDilate();
        doErode();
        doErode();
    }
    
    // 3. scan the border, get the TDPoint of the lines
    scanBorders(lines);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers

//simplified Erode/dilate
int BoundaryScan :: doErode()
{
    // NOTE: following code just deal with 2x2 window! Be aware of it.
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
                m_directions[n][k*width + j] =
                  m_directions[n][(k+1)*width + j] =
                    m_directions[n][k*width + j]     &
                    m_directions[n][(k+1)*width + j] &
                    m_directions[n][k*width + j+1]   & 
                    m_directions[n][(k+1)*width + j+1];
                if (j == width - M_ELEMENT_WIDTH)
                    m_directions[n][k*width + j+1] =
                        m_directions[n][(k+1)*width + j+1] =
                            m_directions[n][k*width + j+1]   &
                            m_directions[n][(k+1)*width + j+1];
                //if (m_directions[n][k*width + j] == 0xFF)
                //    LogI("One point 255, x=%d, y=%d\n", j, k);
            }
        }
    }    
    return 0;
}

int BoundaryScan :: doDilate()
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
                m_directions[n][k*width + j] =
                  m_directions[n][(k+1)*width + j] =
                    m_directions[n][k*width + j]     |
                    m_directions[n][(k+1)*width + j] |
                    m_directions[n][k*width + j+1]   | 
                    m_directions[n][(k+1)*width + j+1];
                if (j == width - M_ELEMENT_WIDTH)
                    m_directions[n][k*width + j+1] =
                        m_directions[n][(k+1)*width + j+1] =
                            m_directions[n][k*width + j+1]   |
                            m_directions[n][(k+1)*width + j+1];
            }
        }
    }
    return 0;
}

int BoundaryScan :: scanBorders(FourBorders & lines)
{
    // we get borders with erode/dilate, then we get the foreground lines.
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
            if (bStart == false && m_directions[n][k] == 0xFF)
            {
                bStart = true;
                line.a.x = k;
                line.a.y = 0;
            }
            if (bStart == true && m_directions[n][k] != 0xFF)
            {
                bStart = false;
                line.b.x = k;
                line.b.y = 0;
                if (line.b.x - line.a.x >= 4)
                    lines.m_lines[n].push_back(line);
                LogI("Get One Line: n-%d, %d to %d\n", n, line.a.x, line.b.x);
                bStart = false;
            }
        }
        // for these are close enough lines, we merge them. Big chance they are the same object.
        // mergeLines(lines.m_lines[n]);
    }
    return 0;
}

//int BoundaryScan :: mergeLines(vector<TDLine> & lines)
//{
//    for (auto it = lines.begin(); it != lines.end(); /*No Increse Here*/)
//    {
//        
// 
//    }
//    return 0;
//}
    
} // namespace Seg_Three
