#include <algorithm>
#include "artsegment.h"

namespace Art_Segment
{
//// helpers
bool neuronScoreComp(Neuron * p1, Neuron * p2)
{
    return p1->getCurScore() < p2->getCurScore();
}

//////////////////////////////////////////////////////////////////////////////////////////
//// ArtNN class method
/**** processOneInput:
 1. classify the input pixel as background or foreground
 2. update its internal neurons' states.
 3. re-arrange neurons as different groups.
 3. return value: 0 - background pixel; > 0 - moving pixel; < 0: process error;
****/
int ArtNN :: processOneInput(const VectorSpace<double> & input)
{
    m_inputFrames++;
    int pixelClassify = 0;
    // 1. update neurons internal scores by new coming input 
    //    in this step all neurons are treated as losers.
    const double distance = updateNeuronsWithNewInput(input);
    if (distance >= 0) // re-update the winner neurons
    {   // resetThisRoundScoreAsWinner must be called after 'updateScoreAsLoser()'
        if (m_bBGWin == true) 
        {
            m_bgNeurons[m_winnerIdx]->reupdateThisRoundAsWinner(input, distance);
            pixelClassify = 0; // it is a bg pixel
        }
        else
        {
            m_movingNeurons[m_winnerIdx]->reupdateThisRoundAsWinner(input, distance);
            pixelClassify = 255; // it is a foreground pixel
        }
    }
    else// 2) fire a new neuron and put it in the proper group.
        pixelClassify = fireANewNeuron(input);

    // 2. move neuron's belongings, bg/moving, merge, delete, etc.
    rearrangeNeurous();
    if (m_inputFrames % 100 == 0 && m_idx % 200 == 0)
        LogI("%d : bg size %d, fg size %d.\n", m_idx, 
             (int)m_bgNeurons.size(), (int)m_movingNeurons.size());
    return pixelClassify;
}

/**** rearrangeNeurous:
 1. update the winner neuron
 2. recalculate the neuron group 
 3. remove death neurons
 4. return value: >= 0 update ok
                  < 0 proccess err
****/
int ArtNN :: rearrangeNeurous()
{
    // moving to bg is important
    if (m_bBGWin == false && m_movingNeurons[m_winnerIdx]->getCurScore() >= 4)
    {
        Neuron *pNeuron = m_movingNeurons[m_winnerIdx];
        m_bgNeurons.push_back(pNeuron);
        m_movingNeurons.erase(std::remove(m_movingNeurons.begin(), 
                              m_movingNeurons.end(), pNeuron), m_movingNeurons.end());
    }

    // 1. moving dead neurons first
    for (auto it = m_bgNeurons.begin(); it != m_bgNeurons.end(); /* No Increment */)
    {
        if (((*it)->getAges() > ((*it)->getMaxMemoryAges() * 2)) && 
            ((*it)->getCurScore() == 0))
        {
            //LogI("bg %d removing dead\n", m_idx);
            m_bgNeurons.erase(it);
        }
        else
            it++;
    }

    for (auto it = m_movingNeurons.begin(); it != m_movingNeurons.end(); /* No Increment */)
    {
        if (((*it)->getAges() > ((*it)->getMaxMemoryAges() * 2)) && 
            ((*it)->getCurScore() == 0))
        {
            //LogI("moving %d removing dead\n", m_idx);
            m_movingNeurons.erase(it);
        }
        else
            it++;
    }

    // 2. do regrouping TODO: seems here is the most important
    const int lastNFrames = m_inputFrames < MAX_MEMORY_AGES ? m_inputFrames : MAX_MEMORY_AGES;

    // 1) if possible mv bg's neuron to moving group, 'possible' here is to check the 'bgPercent'
    std::sort(m_bgNeurons.begin(), m_bgNeurons.end(), neuronScoreComp);
    //if (m_inputFrames == 50)
    //    if (m_idx % 200 == 0)        
    //        for (int k = 0; k < (int)m_bgNeurons.size(); k++)
    //            LogI("%d - %d\n", k, m_bgNeurons[k]->getCurScore());
    unsigned int totalScoresWithoutMin = 0;
    for (int k = 1; k < (int)m_bgNeurons.size(); k++) // NOTE: from index 1
        totalScoresWithoutMin += m_bgNeurons[k]->getCurScore();

    if (totalScoresWithoutMin * 1.0 / lastNFrames > m_bgPercent && 
        m_bgNeurons[0]->getAges() >= MAX_MEMORY_AGES &&
        m_bgNeurons[0]->getCurScore() <= 1)
    {// ok, we can move this neuron to foreground group
        auto it = m_bgNeurons.begin();
        //LogI("%d total score without min: %d, kick's score: %d.\n", 
        //     m_idx, totalScoresWithoutMin, m_bgNeurons[0]->getCurScore());
        m_movingNeurons.push_back(*it);
        m_bgNeurons.erase(it);
    }
    
    // 2) move movingGroup's to bgGroup in case of:
    //    a. movingGroup size >> bgGroup b. movingGourp bgPercent is hight.
    std::sort(m_movingNeurons.begin(), m_movingNeurons.end(), neuronScoreComp);
    unsigned int totalScores = 0;
    for (int k = 0; k < (int)m_movingNeurons.size(); k++) // NOTE: from index 1
        totalScores += m_movingNeurons[k]->getCurScore();

    if (totalScores * 1.0 / lastNFrames > (m_bgPercent / 2)) //TODO: how to effective move to bg?
    {
        auto pNeuron = m_movingNeurons.back();
        m_bgNeurons.push_back(pNeuron);
        m_movingNeurons.pop_back();
    }
   
    // 3. do merging
    mergeCloseNeurons(m_bgNeurons, "bg");
    mergeCloseNeurons(m_movingNeurons, "moving");
    return 0;
}

// each time we at most merge one pair
void ArtNN :: mergeCloseNeurons(vector<Neuron *> & neurons, const string & mergeType)
{
    // TODO: debug
    return;

    bool canMerge = false;
    auto it1 = neurons.begin();
    auto it2 = neurons.begin();
    for (/**/; it1 != neurons.end(); it1++)
    {
        for (/**/; it2 != neurons.end(); it2++)
        {
            if (it1 != it2)
            {
                canMerge = VectorSpace<double>::eulerDistance((*it1)->getWeightVector(),
                                                              (*it2)->getWeightVector())  /
                          ((*it1)->getCurVigilance() + (*it2)->getCurVigilance()) < m_overlapRate;
                if (canMerge == true)
                    break;
            }
        }
        if (it2 != neurons.end())
            break;
    }
    
    if (canMerge == true)
    {
        const int a1 = (*it1)->getCurScore();
        const int a2 = (*it2)->getCurScore();
        const double v1 = (*it1)->getCurVigilance();
        const double v2 = (*it2)->getCurVigilance();
        VectorSpace<double> newWeight = ((*it1)->getWeightVector() * a1 + 
                                         (*it2)->getWeightVector() * a2) * (1.0 / (a1 + a2));
        
        const double vigilanceDiff = std::min(VectorSpace<double>::eulerDistance((*it1)->getWeightVector(), newWeight),
                                              VectorSpace<double>::eulerDistance((*it2)->getWeightVector(), newWeight));
        const double newVigilance = (a1 * v1 + a2 * v2) / (a1 + a2) + vigilanceDiff;
        newWeight.dumpComponents();

        Neuron * pMergeNeuron = new Neuron(newWeight); // using it2 as the default
        pMergeNeuron->setScores((*it2)->getScores());
        pMergeNeuron->setCurScore((*it2)->getCurScore());
        pMergeNeuron->setAges((*it2)->getAges());
        pMergeNeuron->setVigilance(newVigilance);

        // learning rate no need to copy
        Neuron *p1 = *it1;
        Neuron *p2 = *it2;
        p1->getWeightVector().dumpComponents();
        p2->getWeightVector().dumpComponents();
        LogI ("%s MergeNeuron: a1 %d, a2 %d, v1 %.2f, v2 %.2f, diff %.2f, p1 %p, p2 %p.\n", 
              mergeType.c_str(), a1, a2, v1, v2, vigilanceDiff, p1, p2);
        delete p1;
        delete p2;
        neurons.erase(std::remove(neurons.begin(), neurons.end(), p1), neurons.end());
        neurons.erase(std::remove(neurons.begin(), neurons.end(), p2), neurons.end());
        neurons.push_back(pMergeNeuron);
    }

    return;
}

/**** fireANewNeuron:
 1. initial a new neuron withe the input Vector as the weightVector
 2. put it in the proper group: bg or moving 
 3. return value: = 0 means its a bg neuron
                  > 0 means its a moving neuron
                  < 0 proccess err
****/
int ArtNN :: fireANewNeuron(const VectorSpace<double> & input)
{   // for the new neuron, it is a = 1, T = 1, keep that.
    // there are several condition, when we create a new neuron.
    // 1. the first several frames, with BG / Moving Group are not stable.
    // 2. the stream has taken for some time, we have a relative stable BG.    
    // TODO: 
    Neuron *pNew = new Neuron(input);
    if (m_bgNeurons.size() == 0)   
    {
        m_bgNeurons.push_back(pNew);
        m_bBGWin = true;
        m_winnerIdx = m_bgNeurons.size() - 1;
        return 0;
    }
    else if (m_bgNeurons.size() > 4 && 
             m_bgNeurons.size() >= m_movingNeurons.size())
    {
        m_movingNeurons.push_back(pNew);
        m_bBGWin = false;
        m_winnerIdx = m_movingNeurons.size() - 1;
        return 255;
    }

    // we check the last win
    if (m_bBGWin == true)
    {
        m_bgNeurons.push_back(pNew);
        m_bBGWin = true;
        m_winnerIdx = m_bgNeurons.size() - 1;
        return 0;
    }

    m_movingNeurons.push_back(pNew);
    m_bBGWin = false;
    m_winnerIdx = m_movingNeurons.size() - 1;
    return 255; // 1 means foreground pixel
}

/**** calculateNeuronScoreWithNewInput:
 1. calculate the similarity(euler distance) of the input with the existing neurons
 2. set the winner: m_bBGWin & m_winnerIdx
 3. winner neuron do the vigilance test
 4. return value: >= 0 the winner neuron's score for updateing neuron's weightVector
                  < 0  need fire a new neuron
****/
double ArtNN :: updateNeuronsWithNewInput(const VectorSpace<double> & input)
{
    // 1. if no neurons in the net, we return nagive 
    if (m_bgNeurons.size() == 0 && m_movingNeurons.size() == 0)
        return -1.0;

    // 2. calculate all neurons' eulerDistance with the input
    //    set all neurons' as the loser neuron first, then reset the winner neuron.
    double distance = std::numeric_limits<double>::max();
    // background neurons
    for (int k = 0; k < (int)m_bgNeurons.size(); k++)
    {   
        m_bgNeurons[k]->updateScoreAsLoser();
        const double tmp = VectorSpace<double>::eulerDistance(m_bgNeurons[k]->getWeightVector(), input);
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
        m_movingNeurons[k]->updateScoreAsLoser();
        const double tmp = VectorSpace<double>::eulerDistance(m_movingNeurons[k]->getWeightVector(), input);
        if (tmp < distance)
        {
            distance = tmp;
            m_bBGWin = false;
            m_winnerIdx = k;
        }                                                       
    }
 
    const bool bVigilanceTestPass = m_bBGWin ?
                   m_bgNeurons[m_winnerIdx]->doVigilanceTest(distance) :
                   m_movingNeurons[m_winnerIdx]->doVigilanceTest(distance);

    return bVigilanceTestPass ? distance : (-1.0);
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// ArtSegment class method

// test api
int ArtSegment :: processFrame(const cv::Mat & in, cv::Mat & out)
{
    assert(in.cols == m_imgWidth && in.rows == m_imgHeight);
    assert(out.cols == m_imgWidth && out.rows == m_imgHeight);
    assert(in.channels() == 3 && out.channels() ==1);
    for (int k = 0; k < m_imgHeight; k++)
    {
        unsigned char *data = in.data + k*m_imgWidth * in.channels();
        for (int j = 0; j < m_imgWidth; j++)
        {
            vector<double> input1;
            vector<int> input2;
            const cv::Vec3b & intensity = in.at<cv::Vec3b>(k, j);
            for(int n=0; n < in.channels(); n++)
                input1.push_back(intensity[n]);
            input2.push_back(*(data + j * in.channels()));
            input2.push_back(*(data + j * in.channels() + 1));
            input2.push_back(*(data + j * in.channels() + 2));
            //assert(input1 == input2);
            //for (int m = 0; m < (int)input1.size(); m++)
            //    printf("%.2f, ", input1[m]);
            //printf("\n");
            //for (int m = 0; m < (int)input2.size(); m++)
            //    printf("%d, ", input2[m]);
            //printf("\n %d %d---\n", k, j);
            out.at<uchar>(k, j) = m_pArts[k][j]->processOneInput(VectorSpace<double>(input1));
        }
    }
    return 0;
}

ArtSegment :: ArtSegment(const int width, const int height)
    : m_imgWidth(width)
    , m_imgHeight(height)
{
    for (int k = 0; k < m_imgHeight; k++)
    {
        vector<ArtNN *> row;
        for (int j = 0; j < m_imgWidth; j++)
        {
            ArtNN * pArtNN = new ArtNN(k * m_imgWidth + j);
            row.push_back(pArtNN);
        }
        m_pArts.push_back(row);
    }

    return;    
}

ArtSegment :: ~ArtSegment()
{
    for (int k = 0; k < m_imgHeight; k++)
        for (int j = 0; j < m_imgWidth; j++)
            delete m_pArts[k][j];
    return;        
}


} // namespace

/*
for(int i=0;i<img2.rows;i++)
    for(int j=0;j<img2.cols;j++)
         img2.at<uchar>(i,j)=255；   //取得像素或者赋值
 
for(int i=0;i<img2.rows;i++)
    for(int j=0;j<img2.cols;j++)
        for（int n=0;n<img2.channels();n++）
              img2.at<uchar>(i,j*img2.channels()+n)=255；   //取得像素或者赋值
*/
