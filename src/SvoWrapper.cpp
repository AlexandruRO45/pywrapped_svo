#include "SvoWrapper.h"

// Now we include the actual, full SVO and OpenCV headers.
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>

// We also include pybind11 to use its data types.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

SvoWrapper::SvoWrapper() {}
SvoWrapper::~SvoWrapper() {}

bool SvoWrapper::initialize(const py::dict &config)
{
    // This is where you would load camera intrinsics and other parameters
    // from the 'config' dictionary passed from Python and create the SVO objects.

    // TODO: Replace this with your actual SVO initialization logic.
    // Example placeholder logic:
    // double fx = config["fx"].cast<double>();
    // svo::CameraPtr cam = std::make_shared<svo::PinholeCamera>(width, height, fx, ...);
    // frame_handler_ = std::make_shared<svo::FrameHandlerMono>(cam);
    // frame_handler_->start(); // Or similar function to begin the processing thread.

    this->get_logger().info("SVO C++ Wrapper Initialized (Placeholder).");
    initialized_ = true;
    return true;
}

py::dict SvoWrapper::processFrame(const cv::Mat &image, double timestamp)
{
    if (!initialized_)
        return py::dict();

    // TODO: This is the core call to the SVO library. Replace 'addImage'
    // with the actual function name from 'svo::FrameHandlerMono'.
    // frame_handler_->addImage(image, timestamp);

    // After processing, get the latest pose.
    // TODO: Replace with the actual function to get the pose.
    // Sophus::SE3f pose_se3 = frame_handler_->lastPose();
    // Eigen::Matrix4d pose_matrix = pose_se3.matrix();

    // For now, we create and return a dummy result.
    py::dict result;
    // result["position"] = pose_matrix.topRightCorner<3,1>();
    // result["orientation_quat"] = Eigen::Quaterniond(pose_matrix.topLeftCorner<3,3>());

    return result;
}

py::list SvoWrapper::getMapPoints()
{
    py::list points;
    if (!initialized_)
        return points;

    // TODO: Access the map and get the 3D points. The syntax will depend on the SVO API.
    // for(const auto& point_pair : frame_handler_->getMap()->points_) {
    //    if(point_pair.second->quality > 3) { // Example of filtering
    //        points.append(point_pair.second->pos()); // pos() is an Eigen::Vector3d
    //    }
    // }

    return points;
}

void SvoWrapper::reset()
{
    // TODO: Call the underlying SVO reset function.
    // if(frame_handler_) {
    //     frame_handler_->reset();
    // }
    initialized_ = false;
    this->get_logger().info("SVO C++ Wrapper Reset.");
}