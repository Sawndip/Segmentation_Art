#include <algorithm>
#include "psoBook.h"

namespace Seg_Three
{
//// helpers
double distanceToProbability(const double distance)
{
    if (distance <= 20)
        return 1.0 - distance * 0.005;
    else if (distance <= 200)
        return 1.0 - (distance * 0.001 + 0.4);
    else if (distance < 400)
        return distance * 0.0005 * (-1.0) + 0.3;
    else
        return 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// PsoNN class method
double PsoNN :: processOneInput(const double input)
{
    return 0.0;
    //m_inputFrames++;
    //m_lastInput = input;
    //if (m_inputFrames == 1)
    //    m_bgNeuron.setWeightVector(input);
    // 
    //const double bgDistance = grayEulerDistance(m_bgNeuron.getWeightVector(), input);
    //const double movingDistance = grayEulerDistance(m_movingNeuron.getWeightVector(), input);
    //const double bgProbability = distanceToProbability(bgDistance);
    //const double movingProbability = distanceToProbability(movingDistance);
    //return movingProbability > bgProbability ? (1 - movingProbability) : bgProbability;
}
    
double PsoNN :: processOneInput(const VectorSpace<double> & input)
{
    m_inputFrames++;
    m_lastInputVector = input;
    if (m_inputFrames == 1)
        m_bgNeuron.setWeightVector(input);
    
    const double bgDistance = VectorSpace<double>::
                              rgbEulerDistance(m_bgNeuron.getWeightVector(), input);
    const double movingDistance = VectorSpace<double>::
                                  rgbEulerDistance(m_movingNeuron.getWeightVector(), input);    
    const double bgProbability = distanceToProbability(bgDistance);
    const double movingProbability = distanceToProbability(movingDistance);
    return movingProbability > bgProbability ? (1 - movingProbability) : bgProbability;
}

int PsoNN :: updateNeuron(const bool bBg)
{
    if (bBg)
    {
        m_bgNeuron.updateAsWinner(m_lastInputVector);
        m_movingNeuron.updateAsLoser();
    }
    else
    {
        m_movingNeuron.updateAsWinner(m_lastInputVector);
        m_bgNeuron.updateAsLoser();
    }

    // change
    //if (m_bgNeuron.getScores() < m_movingNeuron.getScores())
    //{
    //    m_bgNeuron = m_movingNeuron;
    //    m_movingNeuron.reset();
    //}
    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// PsoBook class method
int PsoBook :: processFrameRgb(const cv::Mat & in, cv::Mat & out)
{
    assert(in.cols == m_imgWidth && in.rows == m_imgHeight);
    assert(out.cols == m_imgWidth && out.rows == m_imgHeight);
    assert(in.channels() == 3 && out.channels() ==1);
    m_inputFrames++;
    vector<double> selfProbabilty(m_imgWidth * m_imgHeight, 0.0);

    for (int k = 0; k < m_imgHeight; k++)
    {
        for (int j = 0; j < m_imgWidth; j++)
        {
            vector<double> input;
            const cv::Vec3b & intensity = in.at<cv::Vec3b>(k, j);
            for(int n=0; n < in.channels(); n++)
                input.push_back(intensity[n]);
            selfProbabilty[k*m_imgWidth+j] =
                m_pPsos[k][j]->processOneInput(VectorSpace<double>(input));
            out.at<uchar>(k, j) = 0;
        }
    }
    
    return refineNetsByCollectiveWisdom(selfProbabilty, out);
}

int PsoBook :: processFrameGray(const cv::Mat & in, cv::Mat & out)
{
    assert(in.cols == m_imgWidth && in.rows == m_imgHeight);
    assert(out.cols == m_imgWidth && out.rows == m_imgHeight);
    assert(in.channels() == 1 && out.channels() ==1);
    m_inputFrames++;
    vector<double> selfProbabilty(m_imgWidth * m_imgHeight, 0.0);

    for (int k = 0; k < m_imgHeight; k++)
    {
        for (int j = 0; j < m_imgWidth; j++)
        {
            selfProbabilty[k*m_imgWidth+j] =
                m_pPsos[k][j]->processOneInput(out.at<uchar>(k, j));
            out.at<uchar>(k, j) = 0;
        }
    }
    
    return refineNetsByCollectiveWisdom(selfProbabilty, out);
}
    
// it seems this can be done by Erode/Dilate    
int PsoBook :: refineNetsByCollectiveWisdom(const vector<double> & p, cv::Mat & out)
{
    vector<double> finalP(m_imgWidth * m_imgHeight, 0.0);
    // for top/bottom border
    const int width = m_imgWidth;
    const int height = m_imgHeight;
    const int leftBottom = width * (height -1);
    const int rightBottom = width * height - 1;
    finalP[0] = p[0] + 0.2 * (p[1] + p[width] + p[width + 1]);
    finalP[width-1] = p[width-1] + 0.2*
                      (p[width-1-1] + p[2*width-1] + p[2*(width-1)]);
    finalP[leftBottom] = p[leftBottom] + 0.2*
                         (p[leftBottom+1] + p[leftBottom-width] + p[leftBottom-width+1]);
    finalP[rightBottom] = p[rightBottom] + 0.2*
                          (p[rightBottom-1] + p[rightBottom-width] + p[rightBottom-width-1]);

    for (int k = 1; k < width - 1; k++)
        finalP[k] = p[k] + 0.15 * (p[k-1] + p[k+1] + 
                             p[width + k-1] + p[width + k] + p[width + k+1]);    
    for (int k = leftBottom+1; k < rightBottom-1; k++)
        finalP[k] = p[k] + 0.15 * (p[k-1] + p[k+1] + 
                             p[k-1-width] + p[k-width] + p[k+1-width]);

    // for  left/right border
    for (int k = 1; k < height - 1; k++)
        finalP[k*width] = p[k*width] + 0.15 * (p[k*width+1] + p[(k-1)*width] +
                               p[(k-1)*width+1] + p[(k+1)*width] + p[(k+1)*width+1]);

    for (int k = 1; k < height - 1; k++)
        finalP[(k+1)*width-1] = p[(k+1)*width-1] + 
            0.15 * (p[(k+1)*width-1-1] + p[k*width-1] + p[k*width-1-1] +
                    p[(k+2)*width-1] + p[(k+2)*width-1-1]);

    // for inside area
    for (int k = 1; k < height - 1; k++)
        for (int j = 1; j < width - 1; j++)
            finalP[k*width + j] = p[k*width + j] + 
                0.1 * (p[k*width + j-1] + p[k*width + j-1] +
                p[(k-1) * width + j-1] + p[(k-1) * width + j] + p[(k-1) * width + j+1] +
                p[(k+1) * width + j-1] + p[(k+1) * width + j] + p[(k+1) * width + j+1]); 

    // mark bg & foreground
    for (int k = 0; k < height; k++)
    {
        for (int j = 0; j < width; j++)   
        {
            const bool bBg = finalP[k*width+j] > M_COLLECTIVE_WISDOM_THREATHOLD;
            out.at<uchar>(k, j) = bBg ? 0 : 255;
            m_pPsos[k][j]->updateNeuron(bBg);
        }
    }
    
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
PsoBook :: ~PsoBook()
{
    if (m_bInit == true)
        for (int k = 0; k < m_imgHeight; k++)
            for (int j = 0; j < m_imgWidth; j++)
                if (m_pPsos[k][j] != NULL)
                    delete m_pPsos[k][j];
    return;        
}

int PsoBook :: init(const int width, const int height)
{
    if (m_bInit == false)
    {
        m_imgWidth = width;
        m_imgHeight = height;
        m_inputFrames = 0;
         
        for (int k = 0; k < m_imgHeight; k++)
        {
            vector<PsoNN *> row;
            for (int j = 0; j < m_imgWidth; j++)
            {
                PsoNN * pPsoNN = new PsoNN(k * m_imgWidth + j);
                row.push_back(pPsoNN);
            }
            m_pPsos.push_back(row);
        }
        m_bInit = true;    
    }

    return 0;
}
    
} // namespace Seg_Three
