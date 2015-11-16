#ifndef _THREE_SEGMENT_MISC_H_
#define _THREE_SEGMENT_MISC_H_

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

}

#endif ////
