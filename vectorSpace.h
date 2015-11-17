#ifndef _VECTOR_SPACE_H_
#define _VECTOR_SPACE_H_
//sys
#include <assert.h>
#include <math.h>
#include <iostream>
#include <vector>

using :: std :: vector;

namespace Vector_Space
{

template <typename T>
class VectorSpace
{
public:
    explicit VectorSpace(const vector<T> & initVector)
        : m_components(initVector)
    {
        return;
    }
    //// copy constructor
    //VectorSpace(const VectorSpace<T> & vs)
    //{
    //    m_components = vs.components();
    //    return;
    //}
    //// assign operator
    //VectorSpace<T> & operator=(const VectorSpace<T> & vs)
    //{
    //    m_components = vs.components();
    //    return *this;
    //}
    
    VectorSpace<T> innerProduct(const VectorSpace<T> & vs2) const
    {
        return VectorSpace<T>(vectorDotProduct(vs2.components()));
    }

    VectorSpace<T> operator*(const T scaler) const
    {
        return VectorSpace<T>(vectorScale(scaler));
    }

    VectorSpace<T> operator+(const VectorSpace<T> & vs2) const
    {
        return VectorSpace<T>(vectorAdd(vs2.components()));
    }

    VectorSpace<T> operator-(const VectorSpace<T> & vs2) const 
    {
        return VectorSpace<T>(vectorMinus(vs2.components()));
    }

    // helpers    
    static double generalEulerDistance(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return vectorEuler(vs1.components(), vs2.components());
    }
    // rgb distance
    static double rgbEulerDistance1(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        assert(vs1.components().size() == 3 && vs2.components().size() == 3);
        return vectorEuler(vs1.components(), vs2.components());
    }

    static double rgbEulerDistance(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        /* using the following formular
        typedef struct {
           unsigned char r, g, b;
        } RGB;
         
        double ColourDistance(RGB e1, RGB e2)
        {
          long rmean = ( (long)e1.r + (long)e2.r ) / 2;
          long r = (long)e1.r - (long)e2.r;
          long g = (long)e1.g - (long)e2.g;
          long b = (long)e1.b - (long)e2.b;
          return sqrt((((512+rmean)*r*r)>>8) + 4*g*g + (((767-rmean)*b*b)>>8));
        }
        */
        assert(vs1.components().size() == 3 && vs2.components().size() == 3);
        int meanRed = (vs1.components()[0] + vs2.components()[0]) / 2;
        int r =  vs1.components()[0] - vs2.components()[0];
        int g =  vs1.components()[1] - vs2.components()[1];
        int b =  vs1.components()[2] - vs2.components()[2];
        return sqrt((((512 + meanRed)*r*r)>>8) + 4*g*g + (((767-meanRed)*b*b)>>8));
    }

    const vector<T> & components() const {return m_components;}
    int dimention() {return (int)m_components.size();}
    void dumpComponents()
    {
        for (int k = 0; k < (int)m_components.size(); k++) 
            std::cout << m_components[k] << " ";
        std::cout << std::endl;
    }


private:
    vector<T> m_components;
    
private:    
    // helpers
    static double vectorEuler(const vector<T> & v1, const vector<T> & v2)
    {
        assert(v1.size() == v2.size());
        double result = 0.0;
        for (int k = 0; k < (int) v1.size(); k++)
            result += pow((double)(v1[k] - v2[k]), 2.0);
        return sqrt(result);
    }

    vector<T> vectorDotProduct(const vector<T> & v2) const
    {
        vector<T> result;
        assert(m_components.size() == v2.size());
        for (int k = 0; k < (int)m_components.size(); k++) 
            result += m_components[k] * v2[k];
        return result;
    }

    vector<T> vectorScale(const T scaler) const
    {
        vector<T> result;
        for (int k = 0; k < (int)m_components.size(); k++) 
            result.push_back(m_components[k] * scaler);
        return result;
    }

    vector<T> vectorAdd(const vector<T> & v2) const
    {
        vector<T> result;
        assert(m_components.size() == v2.size());
        for (int k = 0; k < (int)m_components.size(); k++)
            result.push_back(m_components[k] + v2[k]);
        return result;
    }

    vector<T> vectorMinus(const vector<T> & v2) const
    {
        vector<T> result;
        assert(m_components.size() == v2.size());
        for (int k = 0; k < (int)m_components.size(); k++)
            result.push_back(m_components[k] - v2[k]);
        return result;
    }
};

} // namespace VectorSpace

#endif //  _VECTOR_SPACE_H_
