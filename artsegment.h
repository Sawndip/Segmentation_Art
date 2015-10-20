#ifndef _ART_SEGMENT_H_
#define _ART_SEGMENT_H_

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

namespace Art_Segment
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
        , m_vigilance(15.0)
        , m_liveTimes(0)
        , m_curScore(0)
        , m_scores(MAX_MEMORY_AGES, 0);
    {
    
    }

    ~Neuron(){}
   
    // apis
    bool doVigilanceTest(const double distance) {return distance < m_vigilance;}        
    void setNewWeightVector(const VectorSpace<double> & vs) {m_weightVector = vs;}
    VectorSpace<int> & getWeightVector() {return m_weightVector;}
    void updateScoreAsLoser()
    {
        const int thisRoundScoreIdx = m_liveTimes % MAX_MEMORY_AGES;
        m_scores[thisRoundScoreIdx] = 0;
        m_curScore -= m_scores[(thisRoundScoreIdx + 1) % MAX_MEMORY_AGES];
        m_liveTimes++;
    }
    // this method must be called afeter 'updateScoreAsLoser()'
    void reupdateThisRoundAsWinner(const VectorSpace<double> & input, const double distance) 
    {   // 1. score update
        const int lastUpdateRoundScoreIdx = (m_liveTimes - 1) % MAX_MEMORY_AGES;
        m_scores[lastUpdateRoundScoreIdx] = 1;
        m_curScore++;
        // 2. learning rate update
        m_learningRate = 1.0 / (1.0 + m_curScore);
        // 3. weight vector update
        m_weightVector = m_weightVector + ((input - m_weightVector) * m_learningRate);
    }

    unsigned int getCurScore() {return m_curScore;}
    unsigned int getAges() {return m_liveTimes;}
    unsigned int getMaxMemoryAges() {return MAX_MEMORY_AGES;}
    double getCurVigilance() {return m_vigilance;}
    void setNewVigilance(const double newVigilance) {m_vigilance = newVigilance;}
    vector<unsigned int> & getScores() {return m_scores;}
    void setScores(vector<unsigned int> & newScores) {return m_scores = newScores;}

private:
    VectorSpace<double> m_weightVector;
    double       m_learningRate; // decrease through time with initial value 1.0        
    // winner neuron takes the vigilance test, then update its weightVector
    // or create a new neuron.
    dboule       m_vigilance; // dynamic change through merging or ??
    // to memory previous socres
    unsigned int m_liveTimes;
    unsigned int m_curScore;
    vector<unsigned int> m_scores;
};

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// class ART Neural Network
class ArtNN
{
public:
    ArtNN() : m_bgPercent(0.8), m_overlapRate (0.9), m_inputFrames(0)
    { 
        return;
    }
    // calculate artNN's output, update internal neurons' states.
    int processOneInput(const VectorSpace<int> & input);

private:
    // determine the percent of backgroud neuron's avtivate times in all.
    // for movingNeuron transform to bgNeuron.
    double m_bgPercent; // 0.8
    // if two neurons are close enough (overlap), they are merged.
    double m_overlapRate; // 0.9
    // neuron models for this pixel
    vector<Neuron *> m_bgNeurons;
    vector<Neuron *> m_movingNeurons;
    unsigned int m_inputFrames;

private: // internal helper members
    bool m_bBGWin;
    int m_winnerIdx;
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

class ArtSegment
{
public:
    ArtSegment(const int width, const int height);
    ~ArtSegment();
    
    // API
    //vector<SegmentFeatures> & processFrame(const unsigned char * pR, 
    //                                       const unsigned char * pG, 
    //                                       const unsigned char * pB);
    int processFrame(const cv::Mat & in, cv::Mat & out);
    
private:
    const int m_imgWidth;
    const int m_imgHeight;
    vector<vector<ArtNN *> > m_pArts; // in width x height
    vector<SegmentFeatures> m_features;
}

} // 

#endif // _ART_SEGMENT_H_
