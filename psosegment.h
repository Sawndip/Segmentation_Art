#ifndef _PSO_SEGMENT_H_
#define _PSO_SEGMENT_H_

// sys
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
// tools - just using Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segmisc.h"
#include "vectorspace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace Vector_Space;

namespace Pso_Segment
{

enum {MAX_MEMORY_AGES = (25 * 1)}; // 25fps * 1s
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
class Neuron
{
public:
    explicit Neuron (const VectorSpace<double> & vs)
        : m_weightVector(vs)
        , m_learningRate(1.0)
        , m_vigilance(20.0)
        , m_liveTimes(0)
        , m_curScore(0)
        , m_scores(MAX_MEMORY_AGES, 0)
    {
    
    }    
    ~Neuron(){}

    //void copyTo(Neuron & newNeuron)
    //{
    //    newNeuron.setWeightVector(m_weightVector);
    //    newNeuron.setNewVigilance(m_vigilance);
    //    newNeuron.setNewAges(m_liveTimes);
    //}
   
    // apis
    bool doVigilanceTest(const double distance) {return distance < m_vigilance;}        
    void setWeightVector(const VectorSpace<double> & vs) {m_weightVector = vs;}
    VectorSpace<double> & getWeightVector() {return m_weightVector;}
    
    void updateScoreAsLoser()
    {
        const int thisRoundScoreIdx = m_liveTimes % MAX_MEMORY_AGES;
        m_scores[thisRoundScoreIdx] = 0;
        m_curScore -= m_scores[(thisRoundScoreIdx + 1) % MAX_MEMORY_AGES];
        m_liveTimes++;
    }
    // this method must be called afeter 'updateScoreAsLoser()'
    void reupdateThisRoundAsWinner(const VectorSpace<double> & input) 
    {   // 1. score update
        const int lastUpdateRoundScoreIdx = (m_liveTimes - 1) % MAX_MEMORY_AGES;
        m_scores[lastUpdateRoundScoreIdx] = 1;
        m_curScore++;
        // 2. learning rate update
        m_learningRate = 0.1; //TODO: 1.0 / (1.0 + m_curScore);
        // 3. weight vector update
        m_weightVector = m_weightVector + ((input - m_weightVector) * m_learningRate);
    }


    unsigned int getAges() {return m_liveTimes;}
    void setAges(const int newAges) {m_liveTimes = newAges;}

private:
    VectorSpace<double> m_weightVector;
    unsigned int m_liveTimes;
    unsigned int m_scores;
};

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// class PSO Neural Network
class PsoNN
{
public:
    PsoNN(const int idx) : m_idx(idx), m_bgPercent(0.9), m_overlapRate (0.1), m_inputFrames(0)
    { 
        return;
    }
    // calculate PsoNN's output, update internal neurons' states.
    double processOneInput(const VectorSpace<double> & input);

private:
    const int m_idx;
    // determine the percent of backgroud neuron's avtivate times in all.
    // for movingNeuron transform to bgNeuron.
    double m_bgPercent; // 0.9
    // if two neurons are close enough (overlap), they are merged.
    double m_overlapRate; // 0.9
    // neuron models for this pixel
    vector<Neuron *> m_bgNeurons;
    vector<Neuron *> m_movingNeurons;
    unsigned int m_inputFrames;

private: // internal helper members
    bool m_bBGWin;
    int m_winnerIdx;
    double fireANewNeuron(const VectorSpace<double> & input);
    double updateNeuronsWithNewInput(const VectorSpace<double> & input);
    void mergeCloseNeurons(vector<Neuron *> & neurons, const string & mergeType);
    int rearrangeNeurous();
    bool tryRemoveBgNeurons(const int lastNFrames);
};


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
struct SegmentFeatures
{
    int m_xCentroid;
    int m_yCentroid;
    int m_l;
    int m_a;
    int m_b;
    int m_width;
    int m_height;
};

class PsoSegment
{
public:
    PsoSegment(const int width, const int height);
    ~PsoSegment();
    
    // API
    //vector<SegmentFeatures> & processFrame(const unsigned char * pR, 
    //                                       const unsigned char * pG, 
    //                                       const unsigned char * pB);
    int processFrame(const cv::Mat & in, cv::Mat & out);

private:
    const int m_imgWidth;
    const int m_imgHeight;
    vector<vector<PsoNN *> > m_pPsos; // in width x height
    vector<SegmentFeatures> m_features;
    int refineProbabilitiesByCollectiveWisdom(vector<double> & p, cv::Mat & out);
};

} // 

#endif // _PSO_SEGMENT_H_
