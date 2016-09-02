#ifndef orb_slam2_TYPES_HPP
#define orb_slam2_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Time.hpp>

namespace orb_slam2
{
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

    /** Output port type **/
    struct Information
    {
        base::Time time; //time-stamp
        int number_relocations;
        int number_loops;
    };
}

#endif

