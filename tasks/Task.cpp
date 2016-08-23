/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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
    throw std::runtime_error("Transformer callback for delta_pose_samples not implemented");
}

void Task::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout << "[VISUAL_STEREO LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
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
        std::cout<< "[VISUAL_STEREO LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        /** Reset computing indices **/
        this->left_computing_idx = this->right_computing_idx = 0;

    }
}

void Task::right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<< "[VISUAL_STEREO RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toString()<<std::endl;
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
        std::cout<< "[VISUAL_STEREO RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

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

    /** Frame Helper **/
    this->frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    this->frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Initialize the slam object **/
    this->slam.reset(new ORB_SLAM2::System(_orb_vocabulary.get(), _orb_calibration.get(), ORB_SLAM2::System::STEREO,true));

    /** Check task property parameters **/
    if (_left_frame_period.value() != _right_frame_period.value())
    {
        throw std::runtime_error("[VISUAL_STEREO] Input port period in Left and Right images must be equal!");
    }

    if (_desired_period.value() < _left_frame_period.value())
    {
        throw std::runtime_error("[VISUAL_STEREO] Desired period cannot be smaller than input ports period!");
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

    RTT::log(RTT::Warning)<<"[VISUAL_STEREO] Actual Computing Period: "<<_desired_period.value()<<" [seconds]"<<RTT::endlog();

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
