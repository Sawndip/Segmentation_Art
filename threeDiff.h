#ifndef _THREE_DIFF_H_
#define _THREE_DIFF_H_

// sys
#include <string>
#include <vector>
// project
#include "segMisc.h"
#include "vectorSpace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace Vector_Space;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
class ThreeDiff // San Fen
{
public:
    ThreeDiff();

private:
    // We try to know the whole lifespan of the moving object, even it is stopped for a
    // while, or it is disguised by something;
    //
    // psosegment is just for codebook & collectiveWiddom
    // three diff will do a lot more:
    // 1. accurately locate the object
    // 2. boundary detect;
    // 3. simple optical flow; direction detect;
    // 4. three frames do diff, do '&';
    
};

} // namespace Seg_Three

#endif // _THREE_DIFF_H_
