/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ORB_SLAM2_TASK_TASK_HPP
#define ORB_SLAM2_TASK_TASK_HPP

#include "orb_slam2/TaskBase.hpp"

/** ORB-SALM2  **/
#include <orb_slam2/System.h>
#include <orb_slam2/Converter.h>
#include <thread>
#include <iomanip>

/** FrameHelper libraries **/
#include <frame_helper/FrameHelper.h> /** Rock lib for manipulate frames **/
#include <frame_helper/FrameHelperTypes.h> /** Types for FrameHelper **/

/** PCL **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** Envire **/
#include <envire_core/all>

/** Boost **/
#include <boost/shared_ptr.hpp> /** shared pointers **/
#include <boost/math/special_functions/round.hpp> // to round a number in standard C++ < 11

namespace orb_slam2
{

    /** PCL TYPES **/
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> PCLPointCloud;
    typedef PCLPointCloud::Ptr PCLPointCloudPtr;

    typedef envire::core::Item<PCLPointCloud> PointCloudItem;

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the orb_slam2 namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','orb_slam2::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
    friend class TaskBase;

    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Intrinsic and extrinsic parameters for the pinhole camera model **/
        frame_helper::StereoCalibration cameracalib;

        /***************************/
        /** Input Port Variables **/
        /***************************/

        PCLPointCloudPtr keyframe_point_cloud;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** Flag to know when the image frame has been processed **/
        bool flag_process_frame;

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        boost::shared_ptr< ::ORB_SLAM2::System> slam;

        //integer to control the period
        unsigned short computing_counts, left_computing_idx, right_computing_idx;

        // incremental stereo pair index
        int frame_idx; 

        frame_helper::FrameHelper frameHelperLeft, frameHelperRight; /** Frame helper **/

        base::samples::frame::FramePair frame_pair; /** Left and right images **/

        Eigen::Affine3d tf_odo_sensor_sensor_1; // Relative camera transformations from delta_poses Tsensor(k)_sensor(k-1)

        Eigen::Affine3d tf_orb_sensor_1_sensor; //Relative camera transformation from ORB_SLAM2 Tsensor(k-1)_sensor(k)

        Eigen::Affine3d tf_keyframe_sensor; // Transformation from last keyframe to sensor frame Tkeyframe_sensor(k)

        std::string origin_frame_id;

        envire::core::EnvireGraph envire_graph; // The map in a graph structure

        PCLPointCloudPtr merge_point_cloud; // merge point cloud from sensor

        /***************************/
        /** Output Port Variables **/
        /***************************/
        base::samples::RigidBodyState slam_pose_out, keyframe_pose_out;
        std::vector< ::base::Waypoint > keyframes_trajectory;
        std::vector< ::base::Waypoint > allframes_trajectory;

        /** Debug intra frame image **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_out;

        /** Features map points **/
        base::samples::Pointcloud features_map;

        /** Task information **/
        orb_slam2::Information info;

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample);

        virtual void left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);

        virtual void right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample);

        virtual void point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "orb_slam2::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Process the images
         * */
        void process(const base::samples::frame::Frame &frame_left,
                const base::samples::frame::Frame &frame_right,
                const base::Time &timestamp);

        /** @brief Calculates whether the computation of a frame is required
         */
        void needFrame (const ::base::samples::RigidBodyState &delta_pose_samples);

        /** @brief Get the Frames Pose wrt to the origin
         */
        void getFramesPose( std::vector< ::base::Waypoint > &kf_trajectory, std::vector< ::base::Waypoint > &frames_trajectory, const Eigen::Affine3d &tf);

        /** @brief Get the Sparse map containing the feature points
         */
        void getMapPointsPose( ::base::samples::Pointcloud &points_map,  const Eigen::Affine3d &tf);

        /** @brief Write the KF trajectory in a text file
         */
        void saveKFTrajectoryText(const string &filename, const Eigen::Affine3d &tf = Eigen::Affine3d::Identity());

        /** @brief Write frames trajectory in a text file
         */
        void saveAllTrajectoryText(const string &filename, const Eigen::Affine3d &tf = Eigen::Affine3d::Identity());

        /** @brief Convert to point cloud
         */
        void toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< PointType >& pcl_pc, double density = 1.0);

        /** @brief Transform a pcl point cloud
         * */
        void transformPointCloud(PCLPointCloud &pcl_pc, const Eigen::Affine3d& transformation);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
}

#endif

