#ifndef SVO_WRAPPER_H
#define SVO_WRAPPER_H

#include <iostream>
#include <memory>
#include <vector>
#include <string>

// SVO and vikit headers
#include <svo/frame_handler_mono.h>
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/map.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>

// OpenCV, Eigen Sophus for data types
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/StdVector>

// Forward-declare SVO classes to keep header clean
namespace svo
{
    class FrameHandlerMono;
}
namespace vk
{
    class AbstractCamera;
}

struct SVOConfig
{
    // Camera Parameters
    std::string camera_model; // "pinhole" or "kannala-brandt"
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;             // Pinhole distortion
    double k_kb1, k_kb2, k_kb3, k_kb4; // Kannala-Brandt distortion
    int img_width, img_height;

    // SVO Algorithm Parameters
    int n_pyr_levels;
    int klt_max_level;
    int klt_min_level;
    double kfselect_mindist;
};

// --- Data Transfer Objects (DTOs) for Map Publishing ---
struct NodeData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    double timestamp;
    Sophus::SE3d pose_fw; // Pose T_f_w
};

// DTO for KeyFrame connection data, mirroring the Python `MapEdge`
struct EdgeData
{
    int from_id;
    int to_id;
};

// DTO for 3D MapPoint data
struct PointData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    Eigen::Vector3d position;
};

// DTO for the result of a single frame processing step
struct ProcessResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sophus::SE3d pose_fw;
    svo::FrameHandlerBase::Stage stage;
    svo::FrameHandlerBase::TrackingQuality quality;
    bool is_keyframe; // True if the frame became a keyframe
};

// The main wrapper class for the SVO engine
class SVOPython
{
public:
    SVOPython();
    ~SVOPython();

    // Initializes the SVO engine with parameters from Python
    bool initialize(const SVOConfig &config);

    // Main processing function, takes an image and returns the latest pose and state
    // std::pair<Sophus::SE3d, int> ProcessImage(const cv::Mat &image, double timestamp);
    ProcessResult ProcessImage(const cv::Mat &image, double timestamp);

    // Resets the SVO map and state
    void reset();

    // --- Data Getter Methods ---
    svo::FrameHandlerBase::Stage get_tracking_state() const;
    svo::FrameHandlerBase::TrackingQuality get_tracking_quality() const;
    std::vector<NodeData> get_all_nodes() const;
    std::vector<EdgeData> get_all_edges() const;
    std::vector<PointData> get_all_map_points() const; 
    
    // -- Quantify info ---
    size_t getLastNumObservations() const;
    double getLastProcessingTime() const;
    int getLastFrameId() const;

private:
    std::unique_ptr<svo::FrameHandlerMono> m_frame_handler;
    vk::AbstractCamera *m_cam; // SVO manages camera memory, so raw pointer is okay here
    bool m_is_initialized;
};

#endif // SVO_WRAPPER_H