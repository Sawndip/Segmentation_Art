#include "segMisc.h"

namespace Seg_Three
{
    bool isYContainedBy(const std:tuple<TDPoint, TDPoint> & small,
                        const std:tuple<TDPoint, TDPoint> & large)
    {
        return std::get<0>(small).y >= std::get<0>(large).y &&
               std::get<1>(small).y <= std::get<1>(large).y &&
    }
    bool isXContainedBy(const std:tuple<TDPoint, TDPoint> & small,
                        const std:tuple<TDPoint, TDPoint> & large)
    {
        return std::get<0>(small).x >= std::get<0>(large).x &&
               std::get<1>(small).x <= std::get<1>(large).x &&
    }

}
