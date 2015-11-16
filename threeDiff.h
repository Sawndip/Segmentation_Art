#ifndef _THREE_DIFF_H_
#define _THREE_DIFF_H_

// sys
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>

namespace Seg_Three
{
    
class SegControl
{
// GET All instance we need:
// 1.read frames and dispatch frames to proper member;
// 2.simple optical flow should hold one class with direction/boundary part;
// 3.three diff do locate;
// 4.border do object size detect;
// 5.psoseg do background and foreground preprocess;

};

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segmisc.h"
#include "vectorspace.h"

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
