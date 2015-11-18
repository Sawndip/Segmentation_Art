#ifndef _THREE_SEGMENT_MISC_H_
#define _THREE_SEGMENT_MISC_H_

#include <tuple>

namespace Seg_Three
{
//// Log Macros
#define LogD(format, ...)  printf("[%-8s:%4d] [DEBUG] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogI(format, ...)  printf("[%-8s:%4d] [INFO] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogW(format, ...)  printf("[%-8s:%4d] [WARN] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogE(format, ...)  printf("[%-8s:%4d] [ERROR] " format, \
                                   __FILE__, __LINE__,  ##__VA_ARGS__)
//// 
//// 
struct TDPoint
{
    TDPoint(const int a, const int b) : x = a, y = b {}
    int x;
    int y;
};

struct TDRect
{
    int x;
    int y;
    int width;
    int height;
};

enum DIRECTION {TOP = 0, BOTTOME, RIGHT, LEFT};
enum MOVING_STATUS {MOVING_IN = 0, MOVING_INSIDE, MOVING_STOP, MOVING_OUT};

extern bool isYContainedBy(const std:tuple<TDPoint, TDPoint> & small,
                           const std:tuple<TDPoint, TDPoint> & large);
extern bool isXContainedBy(const std:tuple<TDPoint, TDPoint> & small,
                           const std:tuple<TDPoint, TDPoint> & large);

}

#endif ////
