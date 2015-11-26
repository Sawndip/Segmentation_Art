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
    m_scanBordSizeTB = scanSizeTB;
    m_scanBordSizeLR = scanSizeLR;
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
int BoundaryScan :: processFrame(const cv::Mat & bgResult, FourBorders & lines)
{
    m_inputFrames++;
    // 0. prepare
    assert(bgResult.channels() == 1);
    assert((int)bgResult.step[0] == m_imgWidth &&
           (int)bgResult.step[1] == (int)sizeof(unsigned char));
    assert(m_scanSizeTB == m_bordersMem.heightTB &&
           m_scanSizeLR == m_bordersMem.heightLR )

    // 1. first extract border data from bgResult
    for (int k = 0; k < m_scanSizeTB; k++)
    {
        memcpy(m_bordersMem.directions[0] + k*m_bordersMem.widthTB,
               bgResult.data + m_imgWidth * (k+m_skipTB) + m_skipLR, // note: using skipLR
               m_imgWidth - 2*m_skipLR);
    
        memcpy(m_bordersMem.directions[1] + k*m_bordersMem.widthTB,
               bgResult.data + m_imgWidth * (m_imgHeight-m_skipBT-k) + m_skipLR,
               m_imgWidth - 2*m_skipLR);
    }
    // for left & right data
    for (int k = 0; k < m_scanSizeLR; k+=2) //note +=2, scanSize should be multiple of 2.
    {
        for (int j = 0; j < m_bordersMem.widthLR; j++)
        {   // left
            m_bordersMem.directions[2][k*m_bordersMem.widthLR+j] =
                bgResult.at<uchar>(j+m_scanSizeTB, k+m_scanSizeLR);
            m_bordersMem.directions[2][(k+1)*bordersMem.widthLR+j]
                = bgResult.at<uchar>(j+m_scanSizeTB, k+m_scanSizeLR+1);
            // right
            m_bordersMem.directions[3][k*m_bordersMem.widthLR+j] =
                bgResult.at<uchar>(j+m_scanSizeTB, m_imgWidth-m_scanSizeLR-k-1);
            m_bordersMem.directions[3][(k+1)*bordersMem.widthLR+j] =
                bgResult.at<uchar>(j+m_scanSizeTB, m_imgWidth-m_scanSizeLR-(k+1)-1);
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
                m_borderMem.directions[n][k*width + j] =
                  m_borderMem.directions[n][(k+1)*width + j] =
                    m_borderMem.directions[n][k*width + j]     &
                    m_borderMem.directions[n][(k+1)*width + j] &
                    m_borderMem.directions[n][k*width + j+1]   & 
                    m_borderMem.directions[n][(k+1)*width + j+1];
                if (j == width - M_ELEMENT_WIDTH)
                    m_borderMem.directions[n][k*width + j+1] =
                        m_borderMem.directions[n][(k+1)*width + j+1] =
                            m_borderMem.directions[n][k*width + j+1]   &
                            m_borderMem.directions[n][(k+1)*width + j+1];
                //if (m_borderMem.directions[n][k*width + j] == 0xFF)
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
                m_borderMem.directions[n][k*width + j] =
                  m_borderMem.directions[n][(k+1)*width + j] =
                    m_borderMem.directions[n][k*width + j]     |
                    m_borderMem.directions[n][(k+1)*width + j] |
                    m_borderMem.directions[n][k*width + j+1]   | 
                    m_borderMem.directions[n][(k+1)*width + j+1];
                if (j == width - M_ELEMENT_WIDTH)
                    m_borderMem.directions[n][k*width + j+1] =
                        m_borderMem.directions[n][(k+1)*width + j+1] =
                            m_borderMem.directions[n][k*width + j+1]   |
                            m_borderMem.directions[n][(k+1)*width + j+1];
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
            if (bStart == false && m_borderMem.directions[n][k] == 0xFF)
            {
                bStart = true;
                line.a.x = k;
                line.a.y = 0;
            }
            if (bStart == true && m_borderMem.directions[n][k] != 0xFF)
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
