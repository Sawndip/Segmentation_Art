#ifndef _PSO_BOOK_H_
#define _PSO_BOOK_H_

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
#include "bookmisc.h"
#include "vectorspace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace Vector_Space;

namespace Seg_Three
{

enum {MAX_MEMORY_AGES = (25 * 1)}; // 25fps * 1s
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
class Neuron
{
public:    
    explicit Neuron (const VectorSpace<double> & vs = 
                           VectorSpace<double>(vector<double>(3, 0.0)))
        : m_weightVector(vs)
        , m_liveTimes(0)
        , m_scores(0)        
    {
    
    }    
    ~Neuron(){}
   
    // apis
    void updateAsLoser()
    {        
        m_scores--;
        if (m_scores < 0) m_scores = 0;
        m_liveTimes++;
    }
    void updateAsWinner(const VectorSpace<double> & input) 
    {   
        //m_weightVector = input;
        m_weightVector = m_weightVector + ((input - m_weightVector) * 0.5);
        m_scores++;
        m_liveTimes++;
    }

    void reset() 
    {
        m_weightVector = VectorSpace<double>(vector<double>(3, 0.0));
        m_liveTimes = 0;
        m_scores = 0;
    }

    void setWeightVector(const VectorSpace<double> & vs) {m_weightVector = vs;}
    VectorSpace<double> & getWeightVector() {return m_weightVector;}    
    void setAges(const int newAges) {m_liveTimes = newAges;}
    unsigned int getAges() {return m_liveTimes;}
    void setScores(const int newScores) {m_scores = newScores;}
    unsigned int getScores() {return m_scores;}

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
    explicit PsoNN(const int idx) 
        : m_idx(idx)
        , m_inputFrames(0)
        , m_lastInputVector(VectorSpace<double> (vector<double>(3, 0.0)))
    { 
        return;
    }
    // calculate PsoNN's output, update internal neurons' states.
    double processOneInput(const VectorSpace<double> & input);
    int updateNeuron(const bool bBg);

private:
    const int m_idx;
    Neuron m_bgNeuron;
    Neuron m_movingNeuron;
    unsigned int m_inputFrames;
    VectorSpace<double> m_lastInputVector;
};

class PsoBook
{
public:
    PsoBook() : m_bInit(false) {};
    ~PsoBook();    
    // API
    int init(const int width, const int height);
    int processFrame(const cv::Mat & in, cv::Mat & out);

private:
    bool m_bInit;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    vector<vector<PsoNN *> > m_pPsos; // in width x height
    int refineNetsByCollectiveWisdom(const vector<double> & p, cv::Mat & out);
    const double M_COLLECTIVE_WISDOM_THREATHOLD = 0.6;
};

} // namespace Seg_Three 

#endif // _PSO_BOOK_H_
