#ifndef orb_slam2_TYPES_HPP
#define orb_slam2_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Time.hpp>
#include <base/Eigen.hpp>

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
        int number_relocalizations;
        int number_loops;
        int images_computing_counts;
        float desired_fps;
        float actual_fps;
        float inliers_matches_ratio;
        float map_matches_ratio;
        double frame_gp_residual;
        double kf_gp_residual;
        double kf_gp_threshold;
    };
}

namespace pituki
{
    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
    };

    struct OutlierRemovalFilterConfiguration
    {
        OutlierFilterType type;

        //STATISTICAL: the number of nearest neighbors to use for mean distance estimation (nr_k)
        //RADIUS: Get the radius of the sphere that will determine which points are neighbors (radiu).
        float parameter_one;

        //STATISTICAL: the standard deviation multiplier for the distance threshold calculation.(stddev_null)
        //RADIUS: number of neighbors that need to be present in order to be classified as an inlier(min_pts)
        float parameter_two;
    };

    struct ConditionalRemovalConfiguration
    {
        bool filter_on;
        bool keep_organized;
        base::Vector3d gt_boundary;
        base::Vector3d lt_boundary;
    };


}

#endif

