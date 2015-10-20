#ifndef _VECTOR_SPACE_H_
#define _VECTOR_SPACE_H_

//sys
#include <math.h>
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

    // copy constructor
    explicit VectorSpace<T> & operator=(const VectorSpace<T> & vs)
    {
        m_components = vs.components();
        return *this;
    }
    
    static double eulerDistance(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return vectorEuler(vs1.components(), vs2.components());
    }

    static VectorSpace<T> innerProduct(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return VectorSpace<T>(vectorDotProduct(vs1.components(), vs2.components()));
    }

    static VectorSpace<T> operator*(const VectorSpace<T> & vs, const double scaler) // double or T?
    {
        return VectorSpace<T>(vectorScale(vs.components(), scaler);
    }

    static VectorSpace<T> operator+(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return VectorSpace<T>(vectorAdd(vs1.components(), vs2.components()));
    }

    static VectorSpace<T> operator-(const VectorSpace<T> & vs1, const VectorSpace<T> & vs2)
    {
        return vs1 + (vs2 * (-1.0))
    }


    // assign operator
    VectorSpace<T> & operator=(const VectorSpace<T> & vs)
    {
        m_components = vs.components();
        return *this;
    }

    // helpers
    Vector<T> & components() {return m_components;}
    int dimention() {return (int)m_components.size();}

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
        //return sqrt(result);
        return result;
    }

    static vector<T> vectorDotProduct(const vector<T> & v1, const vector<T> & v2)
    {
        vector<T> result;
        assert(v1.size() == v2.size());
        for (int k = 0; k < (int)v1.size(); k++)
            result += v1[k] * v2[k];
        return result;
    }

    static vector<T> vectorScaler(const vector<T> & v1, const T scaler)
    {
        vector<T> result;
        for (int k = 0; k < (int)v1.size(); k++)
            result.push_back(v1[k] * scaler);
        return result;
    }

    static vector<T> vectorAdd(const vector<T> & v1, const vector<T> & v2)
    {
        vector<T> result;
        assert(v1.size() == v2.size());
        for (int k = 0; k < (int)v1.size(); k++)
            result.push_back(v1[k] + v2[k]);
        return result;
    }
};

} // namespace VectorSpace

#endif //  _VECTOR_SPACE_H_
