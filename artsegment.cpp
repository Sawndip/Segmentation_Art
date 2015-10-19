#include "artsegment.h"

namespace Art_Segment
{

//////////////////////////////////////////////////////////////////////////////////////////
//// ArtNN class method

/**** processOneInput:
 1. classify the input pixel as background or foreground
 2. update its internal neurons' states.
 3. return value: 0 - background pixel; > 0 - moving pixel; < 0: process error;
****/
int ArtNN :: processOneInput(const VectorSpace<int> & input)
{
    int retVal = 0;
    double distance = calculateWinningNeuron(input);
    if (distance < 0)
    {
        // fire a new neuron and put it in the proper group.
        retVal = fireANewNeuron(input);
        // set up retVal
    }
    else
    {
        // update the neuron's status
        updateNetStatus(input, distance);
        // set up retVal
        retVal = m_bBGWin ? 0 : 1;
    }

    return retVal;
}

/**** fireANewNeuron:
 1. update the winner neuron
 2. recalculate the neuron group 
 3. remove death neurons
 4. return value: >= 0 update ok
                  < 0 proccess err
****/
int ArtNN :: updateNetStatus(const VectorSpace<int> & input, const double distance)
{
    

    return 0;
}

/**** fireANewNeuron:
 1. initial a new neuron withe the input Vector as the weightVector
 2. put it in the proper group: bg or moving 
 3. return value: = 0 means its a bg neuron
                  > 0 means its a moving neuron
                  < 0 proccess err
****/
int ArtNN :: fireANewNeuron(const VectorSpace<int> & input)
{
    Neuron newNeuron(input);
    // determin which group to put this neuron in, bg or moving?
        
    

    return 0;
}

/**** calculateWinningNeuron:
 1. calculate the similarity(euler distance) of the input with the existing neurons
 2. set the winner: m_bBGWin & m_winnerIdx
 3. return value: >= 0 the winner neuron's score for updateing neuron's weightVector
                  < 0  need fire a new neuron
****/
double ArtNN :: calculateWinningNeuron(const VectorSpace<int> & input)
{
    // 1. if no neurons in the net, we return nagive 
    if (m_bgNeurons.size() == 0 && m_movingNeurons.size() == 0)
    {
        LogW("Currently no neurons in the ArtNeuralNetwrok.\n");
        return -1.0;
    }

    // 2. calculate all neurons' eulerDistance will the input
    double distance = std::numeric_limits::max;
    // background neurons
    for (int k = 0; k < (int)m_bgNeurons.size(); k++)
    {
        double tmp = eulerDistance(m_bgNeurons[k].getNeuronWeightVector(), input);
        if (tmp < distance)
        {
            distance = tmp;
            m_bBGWin = true;
            m_winnerIdx = k;
        }                                                                                       
    }
    // moving neurons
    for (int k = 0; k < (int)m_movingNeurons.size(); k++)
    {
        double tmp = eulerDistance(m_movingNeurons[k].getNeuronWeightVector(), input);
        if (tmp < distance)
        {
            distance = tmp;
            m_bBGWin = false;
            m_winnerIdx = k;
        }                                                                                       
    }
 
    const bool bVigilanceTestPass = m_bBGWin ?
                   m_bgNeurons[m_winnerIdx].doVigilanceTest(distance) :
                   m_movingNeurons[m_winnerIdx].doVigilanceTest(distance);
    return bVigilanceTestPass ? distance : -1.0;
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// ArtSegment class method


} // namespace
