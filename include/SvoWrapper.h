#ifndef SVOWRAPPER_H
#define SVOWRAPPER_H

#pragma once

#include <memory>
#include <string>
#include <vector>

// Forward declarations to avoid including heavy SVO and OpenCV headers here.
// This tells the compiler that these types exist without needing their full definition.
namespace svo
{
    class FrameHandlerMono;
}
namespace cv
{
    class Mat;
}
namespace pybind11
{
    class dict;
    class list;
}

/**
 * @class SvoWrapper
 * @brief Acts as a simplified C++ interface (façade) to the SVO library.
 * This class holds the main SVO object and exposes a minimal set of methods
 * for initialization, processing, and data retrieval.
 */
class SvoWrapper
{
public:
    SvoWrapper();
    ~SvoWrapper();

    /**
     * @brief Initializes the SVO system with configuration parameters.
     * @param config A pybind11 dictionary passed from Python containing settings.
     * @return True if initialization is successful, false otherwise.
     */
    bool initialize(const pybind11::dict &config);

    /**
     * @brief Processes a single image frame through the SVO pipeline.
     * @param image The input image as a cv::Mat.
     * @param timestamp The timestamp of the image.
     * @return A pybind11 dictionary containing the resulting pose.
     */
    pybind11::dict processFrame(const cv::Mat &image, double timestamp);

    /**
     * @brief Retrieves the current 3D map points from the SVO map.
     * @return A pybind11 list of 3D point coordinates.
     */
    pybind11::list getMapPoints();

    /**
     * @brief Resets the SVO system to its initial state.
     */
    void reset();

private:
    // A smart pointer to the main SVO processing object.
    std::shared_ptr<svo::FrameHandlerMono> frame_handler_;
    bool initialized_ = false;
};

#endif // ORBSLAM3_WRAPPER_H