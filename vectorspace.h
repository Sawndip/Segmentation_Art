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
    static double eulerDistance(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return vectorEuler(vs1.components(), vs2.components());
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
        //return result;
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
