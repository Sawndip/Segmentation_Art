#include <stdio.h>
#include <vector>
#include "vectorspace.h"

using std :: vector;
using namespace Vector_Space;

int main (int argc, char ** argv)
{
    
    vector<double> v1(3, 0);
    v1[0] = 1; v1[1] = 2; v1[2] = 3; 

    vector<double> v2(3, 0);
    v2[0] = 4; v2[1] = 5; v2[2] = 6; 

    VectorSpace<double> vs1(v1);
    VectorSpace<double> vs2(v2);
    printf("vs1: \n");
    vs1.dumpComponents();

    printf("vs2: \n");
    vs2.dumpComponents();

    printf("Addition: \n");
    (vs1 + vs2).dumpComponents();

    printf("Minus: \n");
    (vs1 - vs2).dumpComponents();

    printf("scale: \n");
    (vs1 * 2).dumpComponents();

    printf("Euler: %.2f\n", VectorSpace<double>::eulerDistance(vs1, vs2));
    
    return 0;
}
