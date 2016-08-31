/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

//#define DEBUG_PRINTS 1

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace orb_slam2;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{

    Eigen::Affine3d tf_body_sensor; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        tf_body_sensor.setIdentity();
    }
    else if (!_sensor2body.get(ts, tf_body_sensor, false))
    {
        RTT::log(RTT::Fatal)<<"[ORB_SLAM2 FATAL ERROR] No transformation provided."<<RTT::endlog();
       return;
    }

    /** Set to identity if it is not initialized **/
    if (!base::isnotnan(this->tf_odo_sensor_sensor_1.matrix()))
    {
        this->tf_odo_sensor_sensor_1.setIdentity();
    }

    /** Accumulate the relative sensor to sensor transformation Tsensor_sensor(k-1) **/
    Eigen::Affine3d tf_body_body = delta_pose_samples_sample.getTransform();
    this->tf_odo_sensor_sensor_1 = this->tf_odo_sensor_sensor_1 * (tf_body_sensor.inverse() * tf_body_body.inverse() * tf_body_sensor);

}

void Task::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout << "[ORB_SLAM2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
    #endif

    /** The image need to be in gray scale and undistorted **/
    frame_pair.first.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperLeft.convert (*left_frame_sample, frame_pair.first, 0, 0, _resize_algorithm.value(), true);

    /** Increase the computing index **/
    this->left_computing_idx++;

    /** If the difference in time is less than half of a period run the odometry **/
    base::Time diffTime = frame_pair.first.time - frame_pair.second.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_left_frame_period/2.0) && (this->left_computing_idx >= this->computing_counts))
    {
        frame_pair.time = frame_pair.first.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[ORB_SLAM2 LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        /** Process the images with ORB_SLAM2 **/
        this->process(frame_pair.first, frame_pair.second, frame_pair.time);



        /** Reset computing indices **/
        this->left_computing_idx = this->right_computing_idx = 0;
    }
}

void Task::right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<< "[ORB_SLAM2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toString()<<std::endl;
    #endif

    /** Correct distortion in image right **/
    frame_pair.second.init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperRight.convert (*right_frame_sample, frame_pair.second, 0, 0, _resize_algorithm.value(), true);

    /** Increase th computing index **/
    this->right_computing_idx++;

    /** Check the time difference **/
    base::Time diffTime = frame_pair.second.time - frame_pair.first.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_right_frame_period/2.0) && (this->right_computing_idx >= this->computing_counts))
    {
        frame_pair.time = frame_pair.second.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[ORB_SLAM2 RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        /** Process the images with ORB_SLAM2 **/
        this->process(frame_pair.first, frame_pair.second, frame_pair.time);

        /** Reset computing indices **/
        this->left_computing_idx = this->right_computing_idx = 0;
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Frame index **/
    this->frame_idx = 0;

    /** Read the camera calibration parameters **/
    this->cameracalib = _calib_parameters.value();

    /** Frame Helper **/
    this->frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    this->frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Initialize output frame **/
    ::base::samples::frame::Frame *outframe = new ::base::samples::frame::Frame();
    this->frame_out.reset(outframe);
    outframe = NULL;

    /** Set Tsensor_sensor(k-1) from odometry to Nan **/
    this->tf_odo_sensor_sensor_1.matrix()= Eigen::Matrix<double, 4, 4>::Zero() * base::NaN<double>();

    /** Set Tsensor(k-1)_sensor from ORB_SLAM2  to Identity*/
    this->tf_orb_sensor_1_sensor.setIdentity();

    /** Optimized Output port **/
    this->slam_pose_out.invalidate();
    this->slam_pose_out.sourceFrame = _slam_localization_source_frame.value();

    /** Relative Frame to port out the SAM pose samples **/
    this->slam_pose_out.targetFrame = _world_frame.value();

    RTT::log(RTT::Warning)<<"[ORB_SLAM2 TASK] DESIRED TARGET FRAME IS: "<<this->slam_pose_out.targetFrame<<RTT::endlog();

    /** Initialize the slam object **/
    this->slam.reset(new ORB_SLAM2::System(_orb_vocabulary.get(), _orb_calibration.get(), ORB_SLAM2::System::STEREO, true));

    /** Check task property parameters **/
    if (_left_frame_period.value() != _right_frame_period.value())
    {
        throw std::runtime_error("[ORB_SLAM2] Input port period in Left and Right images must be equal!");
    }

    if (_desired_period.value() < _left_frame_period.value())
    {
        throw std::runtime_error("[ORB_SLAM2] Desired period cannot be smaller than input ports period!");
    }
    else if (_desired_period.value() == 0.00)
    {
        _desired_period.value() = _left_frame_period.value();
        this->computing_counts = 1;
    }
    else
    {
        this->computing_counts = boost::math::iround(_desired_period.value()/_left_frame_period.value());
        _desired_period.value() = this->computing_counts * _left_frame_period.value();
    }

    RTT::log(RTT::Warning)<<"[ORB_SLAM2] Actual Computing Period: "<<_desired_period.value()<<" [seconds]"<<RTT::endlog();

    this->left_computing_idx = this->right_computing_idx = 0;

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Stop ORB_SLAM2 all threads **/
    this->slam->Shutdown();

    /** Reset ORB_SLAM2 **/
    this->slam.reset();
}

void Task::process(const base::samples::frame::Frame &frame_left,
                const base::samples::frame::Frame &frame_right,
                const base::Time &timestamp)
{
    /** Convert Images to opencv **/
    cv::Mat img_l = frameHelperLeft.convertToCvMat(frame_left);
    cv::Mat img_r = frameHelperRight.convertToCvMat(frame_right);

    /** Check whether there is delta pose in camera frame from motion model **/
    cv::Mat tf_motion_model;
    if (base::isnotnan(this->tf_odo_sensor_sensor_1.matrix()))
    {
        //std::cout<<"[ORB_SLAM2 PROCESS] TF_ODO_SENSOR_SENSOR_1:\n"<< this->tf_odo_sensor_sensor_1.matrix() <<"\n";

        /** ORB_SLAM2 with motion model information **/
        tf_motion_model = ORB_SLAM2::Converter::toCvMat(this->tf_odo_sensor_sensor_1.matrix());

        /** Reset the Tsensor(k)_sensor(k-1) **/
        this->tf_odo_sensor_sensor_1.setIdentity();
    }

    /** ORB_SLAM2 **/
    this->slam->TrackStereo(img_l, img_r, timestamp.toSeconds(), tf_motion_model);

    /** Left color image **/
    if (_output_debug.get())
    {
        /** Get frame with information from ORB_SLAM2 **/
        cv::Mat img_out = this->slam->mpFrameDrawer->DrawFrame();

        /** Convert to Frame **/
        ::base::samples::frame::Frame *frame_ptr = this->frame_out.write_access();
        this->frameHelperLeft.copyMatToFrame(img_out, *frame_ptr);

        /** Out port the image **/
        frame_ptr->time = this->frame_pair.time;
        this->frame_out.reset(frame_ptr);
        _frame_samples_out.write(this->frame_out);
    }

    /** Get the transformation Tworld_navigation **/
    Eigen::Affine3d tf_world_nav; /** Transformer transformation **/
    /** Get the transformation Tworld_navigation (navigation is body_0) **/
    if (_navigation_frame.value().compare(_world_frame.value()) == 0)
    {
        tf_world_nav.setIdentity();
    }
    else if (!_navigation2world.get(timestamp, tf_world_nav, false))
    {
        RTT::log(RTT::Fatal)<<"[ORB_SLAM2 FATAL ERROR]  No transformation provided."<<RTT::endlog();
       return;
    }

    Eigen::Affine3d tf_body_sensor; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        tf_body_sensor.setIdentity();
    }
    else if (!_sensor2body.get(timestamp, tf_body_sensor, false))
    {
        RTT::log(RTT::Fatal)<<"[ORB_SLAM2 FATAL ERROR] No transformation provided."<<RTT::endlog();
       return;
    }

    /** Get the camera pose from ORB_SLAM2 **/
    if (!this->slam->mpTracker->mCurrentFrame.mTcw.empty())
    {
        g2o::SE3Quat se3_nav_sensor = ORB_SLAM2::Converter::toSE3Quat(this->slam->mpTracker->mCurrentFrame.mTcw).inverse();

        /** SE3 to Affine3d **/
       this->tf_orb_sensor_1_sensor = se3_nav_sensor.rotation();
       this->tf_orb_sensor_1_sensor.translation() = se3_nav_sensor.translation();
    }

    /** Tworld_body = Tworld_body * Tbody_sensor * Tsensor(k-1)_sensor * Tsensor_body **/
    Eigen::Affine3d tf_world_body = tf_world_nav * tf_body_sensor * this->tf_orb_sensor_1_sensor * tf_body_sensor.inverse();

    /** Out port the last slam pose **/
    this->slam_pose_out.time = timestamp;
    this->slam_pose_out.setTransform(tf_world_body);
    _pose_samples_out.write(this->slam_pose_out);
}
