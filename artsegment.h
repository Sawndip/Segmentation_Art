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
    ArtNN();
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
    
public:
    // internal class for classify background/foreground pixel
    class Neuron
    {
    public:
        explicit Neuron (const VectorSpace<int> & vs = VectorSpace<int> (0,3))
            : m_weightVector(vs)
        {

        }
        ~Neuron(){}
        
    private:
        VectorSpace<int> m_weightVector;
        double m_learningRate; // change through time
        double m_theta; // determine the precent of background: 1 / T
    };

private:
    // winner neuron take the vigilance test to determine update its weightVector
    // or create a new neuron.
    dboule m_vigilance; // 15 - 25
    double m_overlapRate; // for merging neurons
    vector<Neuron> m_bgNeurons;
    vector<Neuron> m_movingNeurons;
};

class ArtSegment
{
public:
    ArtSegment(const int width, const int height)
    {
       for 
    }
    ~ArtSegment()
    {
        
    }
    
    vector<Rect> & rects processFrame(Mat & frame);
    
private:
    vector<vector<ArtNN *> > m_pArts; // in width x height
    vector<Rect> m_motionRects;
}

} // 

#endif // _ART_SEGMENT_H_
