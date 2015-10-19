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

// class ART Neural Networ    
class ArtNN
{
public:
    ArtNN() : m_theta (0.8), m_overlapRate (0.9)
    {
        return;
    }
    // calculate artNN's output, update internal neurons' states.
    int processOneInput(const VectorSpace<int> & input);

    void addNewBgNeuron(const VectorSpace<int> & vs)
    {
        m_bgNeurons.push_back(Neuron(vs));
    }    
    void addNewMovingNeuron(const VectorSpace<int> & vs)
    {
        m_movingNeurons.push_back(Neuron(vs));
    }

    // internal class for classify background/foreground pixel    
public:
    class Neuron
    {
    public:
        Neuron() : m_learningRate(1.0), m_vigilance(15.0) 
        {
            m_weightVector.resize(3);
            return;
        }
        explicit Neuron (const VectorSpace<int> & vs)
            : m_weightVector(vs)
            , m_learningRate(1.0) 
            , m_vigilance(15.0) 
        {

        }
        ~Neuron(){}

    public:
        void setNewWeightVector(const VectorSpace<int> & vs) {m_weightVector = vs;}
        VectorSpace<int> & getNeuronWeightVector() {return m_weightVector;}
        //
        bool doVigilanceTest(const double distance) {return distance < m_vigilance;}        

    private:
        VectorSpace<int> m_weightVector;
        double m_learningRate; // decrease through time with initial value 1.0        
        // winner neuron take the vigilance test to determine update its weightVector
        // or create a new neuron.
        dboule m_vigilance; // dynamic change through merging or ??
    };

private:
    // determine the percent of backgroud neuron's avtivate times in all.
    // for movingNeuron transform to bgNeuron.
    double m_theta; // 0.8
    // if two neurons are close enough (overlap), they are merged.
    double m_overlapRate; // 0.9
    // neuron models for this pixel
    vector<Neuron> m_bgNeurons;
    vector<Neuron> m_movingNeurons;

private: // internal helper members
    bool m_bBGWin;
    int m_winnerIdx;
};



//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

struct SegmentFeatures
{
    int m_x;
    int m_y;
    int m_y;


};

class ArtSegment
{
public:
    ArtSegment(const int width, const int height)
        : m_imgWidth(width)
        , m_imgHeight(height)
    {

    }
    ~ArtSegment()
    {
        
    }
    
    // API
    vector<VectorSpace<double> > & processFrame(const unsigned char * pR, 
                                                const unsigned char * pG, 
                                                const unsigned char * pB);
    
private:
    const int m_imgWidth;
    const int m_imgHeight;
    vector<vector<ArtNN *> > m_pArts; // in width x height
    vector<VectorSpace<double>> m_features;
}

} // 

#endif // _ART_SEGMENT_H_
