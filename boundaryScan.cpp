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
    m_borders.init(M_BORDER_ROWS, m_imgWidth);
    // actualy not used, for we use a simplified erode/dilate
    for (int k = 0; k < M_ELEMENT_HEIGHT; k++)
    {
        for (int j = 0; j < M_ELEMENT_HEIGHT; j++)
        {
            //erodeMatrix[k][j] = 255;
            //dilateMatrix[k][j] = 255;
        }
    }
    // 
    m_directions[0] = m_borders.top;
    m_directions[1] = m_borders.bottom;
    m_directions[2] = m_borders.left;
    m_directions[3] = m_borders.right;    
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
int BoundaryScan :: processFrame(const cv::Mat & in,
                                 vector<vector<tuple<TDPoint, TDPoint> > > & lines)
{
    m_inputFrames++;
    // 1. first extract border data (two lines), `in` data is in Gray(CV_8UC1);
    assert(in.channels() == 1);
    assert((int)in.step[0] == m_imgWidth && (int)in.step[1] == (int)sizeof(unsigned char));
    memcpy(m_borders.top, in.data, M_BORDER_ROWS * m_imgWidth);
    memcpy(m_borders.bottom, in.data + in.step[0]*(m_imgHeight-M_BORDER_ROWS),
           M_BORDER_ROWS*m_imgWidth);
    
    // for left & right data
    for (int j = 0; j < M_BORDER_ROWS; j++)   
    {
        for (int k = 0; k < m_imgHeight; k++)
        {
            m_borders.left[j*m_imgHeight+k] = in.at<uchar>(k, j);
            m_borders.right[j*m_imgHeight+k] = in.at<uchar>(k, m_imgWidth - j - 1);
        }
    }

    // 2. we do open / close: seems for simplified erode/dilate, just open is ok.
    // oepn: erode then dilate
    doErode();
    doDilate();
    // close: dilate then erode
    doDilate();
    doErode();
    
    // 3. scan the border, get the TDPoint of the lines
    scanBorders(lines);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers

//simplified Erode/dilate
int BoundaryScan :: doErode()
{
    // TODO: following code just deal with 2x2 window! Be aware of it.
    for (int n = 0; n < 4; n++)
    {   // note k = k + M_ELEMENT_HEIGHT
        for (int k = 0; k < m_borders.rows; k+=M_ELEMENT_HEIGHT)
        {
            for (int j = 0; j < m_borders.columns - 1; j++)
            {
                m_directions[n][k*m_borders.columns + j] =
                  m_directions[n][(k+1)*m_borders.columns + j] =
                    m_directions[n][k*m_borders.columns + j]     &
                    m_directions[n][(k+1)*m_borders.columns + j] &
                    m_directions[n][k*m_borders.columns + j+1]   & 
                    m_directions[n][(k+1)*m_borders.columns + j+1];
                if (j == m_borders.columns - M_ELEMENT_WIDTH)
                    m_directions[n][k*m_borders.columns + j+1] =
                        m_directions[n][(k+1)*m_borders.columns + j+1] =
                            m_directions[n][k*m_borders.columns + j+1]   &
                            m_directions[n][(k+1)*m_borders.columns + j+1];
            }
        }
    }
    return 0;
}

int BoundaryScan :: doDilate()
{
    for (int n = 0; n < 4; n++)
    {   // note k = k + M_ELEMENT_HEIGHT
        for (int k = 0; k < m_borders.rows; k+=M_ELEMENT_HEIGHT)   
        {
            for (int j = 0; j < m_borders.columns - 1; j++)
            {
                m_directions[n][k*m_borders.columns + j] =
                  m_directions[n][(k+1)*m_borders.columns + j] =
                    m_directions[n][k*m_borders.columns + j]     |
                    m_directions[n][(k+1)*m_borders.columns + j] |
                    m_directions[n][k*m_borders.columns + j+1]   | 
                    m_directions[n][(k+1)*m_borders.columns + j+1];
                if (j == m_borders.columns - 2)
                    m_directions[n][k*m_borders.columns + j+1] =
                        m_directions[n][(k+1)*m_borders.columns + j+1] =
                            m_directions[n][k*m_borders.columns + j+1]   |
                            m_directions[n][(k+1)*m_borders.columns + j+1];
            }
        }
    }
    return 0;
}

int BoundaryScan :: scanBorders(vector<vector<tuple<TDPoint, TDPoint> > > & lines)
{
    // we get borders with erode/dilate, then we get the foreground lines.
    for (int n = 0; n < 4; n++)
    {
        TDPoint start, end;
        bool bStart = false, bEnd = false;
        vector<tuple<TDPoint, TDPoint> > oneDirection;
        for (int k = 0; k < m_borders.columns; k++)
        {
            if (bStart == false && m_directions[n][k] == 0xFF)
            {
                bEnd = false;
                bStart = true;
                start.x = 0;
                start.y = k;
            }
            if (bStart == true && m_directions[n][k] != 0xFF)
            {
                bEnd = true;
                bStart = false;
                end.x = 0;
                end.y = k;
            }            
            if (bStart == true && bEnd == true)
            {
                oneDirection.push_back(std::make_tuple(start, end));
                bStart = false;
                bEnd =  false;
            }
        }
        if (end.y - start.y > 4)
            lines.push_back(oneDirection);
    }
    return 0;
}

} // namespace Seg_Three
