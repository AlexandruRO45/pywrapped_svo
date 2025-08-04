#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "SVOWrapper.h"
#include "NDArrayConverter.h"

namespace py = pybind11;

// Constructor: Initializes the smart pointer.
SVOPython::SVOPython() : m_frame_handler(nullptr), m_cam(nullptr), m_is_initialized(false) {}

// Destructor: Cleans up the camera object.
SVOPython::~SVOPython()
{
    if (m_cam)
        delete m_cam;
    // if (m_frame_handler)
    //     delete m_frame_handler; //TODO: find out if is managed better by lib <memory>
}

// Initializes the SVO engine.
bool SVOPython::initialize(const SVOConfig &config)
{
    if (m_is_initialized)
    {
        std::cout << "SVO Wrapper is already initialized." << std::endl;
        return true;
    }

    try
    {
        // 1. Configure the SVO singleton using the provided config struct.
        svo::Config::nPyrLevels() = config.n_pyr_levels;
        svo::Config::kltMaxLevel() = config.klt_max_level;
        svo::Config::kltMinLevel() = config.klt_min_level;
        svo::Config::kfSelectMinDist() = config.kfselect_mindist;
        // ... set other config parameters as needed ... TODO

        // 2. Create the camera model based on the configuration.
        if (config.camera_model == "Pinhole")
        {
            m_cam = new vk::PinholeCamera(
                config.img_width, config.img_height,
                config.fx, config.fy, config.cx, config.cy,
                config.k1, config.k2, config.p1, config.p2);
        }
        else
        {
            // Add other camera models like KannalaBrandt here if needed
            std::cerr << "Unsupported camera model specified in config: " << config.camera_model << std::endl;
            return false;
        }

        // 3. Instantiate the main SVO engine object.
        // m_frame_handler = std::make_unique<svo::FrameHandlerMono>(m_cam);
        // Instead of std::make_unique, we use a raw `new` which correctly calls the
        // overloaded `new` operator defined by EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // in the SVO classes. We then move the raw pointer into the unique_ptr.
        svo::FrameHandlerMono *raw_handler = new svo::FrameHandlerMono(m_cam);
        m_frame_handler.reset(raw_handler);

        // 4. Start the engine. In SVO, this is done by setting the stage.
        m_frame_handler->start(); // This sets the stage to STAGE_FIRST_FRAME

        m_is_initialized = true;
        std::cout << "SVO Wrapper Initialized Successfully." << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during SVO initialization: " << e.what() << std::endl;
        return false;
    }
}

// Main processing function.
ProcessResult SVOPython::ProcessImage(const cv::Mat &image, double timestamp)
{
    if (!m_is_initialized || !m_frame_handler)
    {
        throw std::runtime_error("SVO engine not initialized. Call initialize() first.");
    }

    // Call the main SVO processing function.
    m_frame_handler->addImage(image, timestamp);

    // After processing, extract the results from the handler's public members.
    svo::FramePtr last_frame = m_frame_handler->lastFrame();
    svo::FrameHandlerBase::Stage current_stage = m_frame_handler->stage();
    svo::FrameHandlerBase::TrackingQuality quality = m_frame_handler->trackingQuality();

    ProcessResult result;
    if (last_frame)
    {
        result.pose_fw = last_frame->T_f_w_;
        result.is_keyframe = last_frame->isKeyframe();
    }
    result.stage = current_stage;
    result.quality = quality;

    return result;
}

// Resets the system.
void SVOPython::reset()
{
    if (m_frame_handler)
    {
        m_frame_handler->reset();
    }
}

// --- Getter Implementations ---
svo::FrameHandlerBase::Stage SVOPython::get_tracking_state() const
{
    if (!m_frame_handler)
        return svo::FrameHandlerBase::STAGE_PAUSED;
    return m_frame_handler->stage();
}

svo::FrameHandlerBase::TrackingQuality SVOPython::get_tracking_quality() const
{
    if (!m_frame_handler)
        return svo::FrameHandlerBase::TRACKING_INSUFFICIENT;
    return m_frame_handler->trackingQuality();
}

// Gets all keyframes from the map to construct the nodes of the MapGraph.
std::vector<NodeData> SVOPython::get_all_nodes() const
{
    std::vector<NodeData> nodes;
    if (!m_is_initialized)
        return nodes;

    // Access the public `keyframes_` member of the map
    const std::list<svo::FramePtr> &keyframes = m_frame_handler->map().keyframes_;
    nodes.reserve(keyframes.size());

    for (const svo::FramePtr &pKF : keyframes)
    {
        nodes.push_back({pKF->id_,
                         pKF->timestamp_,
                         pKF->T_f_w_});
    }
    return nodes;
}

// Creates edges based on the temporal connections of keyframes.
std::vector<EdgeData> SVOPython::get_all_edges() const
{
    std::vector<EdgeData> edges;
    if (!m_is_initialized)
        return edges;

    const std::list<svo::FramePtr> &keyframes = m_frame_handler->map().keyframes_;

    // For SVO, a simple temporal link between consecutive keyframes is a good representation of the graph.
    svo::Frame *prev_kf = nullptr;
    for (const svo::FramePtr &pKF : keyframes)
    {
        if (prev_kf != nullptr)
        {
            edges.push_back({prev_kf->id_,
                             pKF->id_});
        }
        prev_kf = pKF.get();
    }
    return edges;
}

// Gets all 3D map points.
std::vector<PointData> SVOPython::get_all_map_points() const
{
    std::vector<PointData> points;
    if (!m_is_initialized)
        return points;

    const std::list<svo::FramePtr> &keyframes = m_frame_handler->map().keyframes_;
    std::set<svo::Point *> unique_points; // Use a set to avoid duplicating points

    // Iterate through all keyframes and collect their observed points.
    for (const svo::FramePtr &pKF : keyframes)
    {
        for (const svo::Feature *ftr : pKF->fts_)
        {
            if (ftr->point)
            {
                unique_points.insert(ftr->point);
            }
        }
    }

    points.reserve(unique_points.size());
    for (const svo::Point *pMP : unique_points)
    {
        points.push_back({pMP->id_,
                          pMP->pos_});
    }
    return points;
}

size_t SVOPython::getLastNumObservations() const
{
    if (!m_frame_handler)
        return 0;
    return m_frame_handler->lastNumObservations();
}

double SVOPython::getLastProcessingTime() const
{
    if (!m_frame_handler)
        return 0.0;
    return m_frame_handler->lastProcessingTime();
}

int SVOPython::getLastFrameId() const
{
    if (!m_frame_handler || !m_frame_handler->lastFrame())
        return -1; 
    return m_frame_handler->lastFrame()->id_;
}


// --- Pybinds Implementations ---
PYBIND11_MODULE(_core, m)
{
    m.doc() = "Python bindings for the SVO Engine";

    // Initialize the custom converter for cv::Mat <-> numpy.ndarray
    NDArrayConverter::init_numpy();

    // --- Bind the SVO Stage Enum ---
    py::enum_<svo::FrameHandlerBase::Stage>(m, "TrackingState")
        .value("PAUSED", svo::FrameHandlerBase::STAGE_PAUSED)
        .value("FIRST_FRAME", svo::FrameHandlerBase::STAGE_FIRST_FRAME)
        .value("SECOND_FRAME", svo::FrameHandlerBase::STAGE_SECOND_FRAME)
        .value("DEFAULT_FRAME", svo::FrameHandlerBase::STAGE_DEFAULT_FRAME)
        .value("RELOCALIZING", svo::FrameHandlerBase::STAGE_RELOCALIZING);

    // --- Bind the TrackingQuality enum ---
    py::enum_<svo::FrameHandlerBase::TrackingQuality>(m, "TrackingQuality")
        .value("INSUFFICIENT", svo::FrameHandlerBase::TRACKING_INSUFFICIENT)
        .value("GOOD", svo::FrameHandlerBase::TRACKING_GOOD)
        .value("BAD", svo::FrameHandlerBase::TRACKING_BAD);

    // --- Bind the Configuration Struct ---
    // This lets Python create and populate the config object.
    py::class_<SVOConfig>(m, "Config")
        .def(py::init<>())
        .def_readwrite("camera_model", &SVOConfig::camera_model)
        .def_readwrite("fx", &SVOConfig::fx)
        .def_readwrite("fy", &SVOConfig::fy)
        .def_readwrite("cx", &SVOConfig::cx)
        .def_readwrite("cy", &SVOConfig::cy)
        .def_readwrite("k1", &SVOConfig::k1)
        .def_readwrite("k2", &SVOConfig::k2)
        .def_readwrite("p1", &SVOConfig::p1)
        .def_readwrite("p2", &SVOConfig::p2)
        .def_readwrite("img_width", &SVOConfig::img_width)
        .def_readwrite("img_height", &SVOConfig::img_height)
        .def_readwrite("n_pyr_levels", &SVOConfig::n_pyr_levels)
        .def_readwrite("klt_max_level", &SVOConfig::klt_max_level)
        .def_readwrite("klt_min_level", &SVOConfig::klt_min_level)
        .def_readwrite("kfselect_mindist", &SVOConfig::kfselect_mindist);

    // --- Bind the Data Transfer Objects ---
    py::class_<NodeData>(m, "NodeData")
        .def_readonly("id", &NodeData::id)
        .def_readonly("timestamp", &NodeData::timestamp)
        .def_readonly("pose_fw", &NodeData::pose_fw);

    py::class_<EdgeData>(m, "EdgeData")
        .def_readonly("from_id", &EdgeData::from_id)
        .def_readonly("to_id", &EdgeData::to_id);

    py::class_<PointData>(m, "PointData")
        .def_readonly("id", &PointData::id)
        .def_readonly("position", &PointData::position);

    // --- Bind the ProcessResult struct ---
    py::class_<ProcessResult>(m, "ProcessResult")
        .def(py::init<>())
        .def_readonly("pose_fw", &ProcessResult::pose_fw)
        .def_readonly("stage", &ProcessResult::stage)
        .def_readonly("quality", &ProcessResult::quality)
        .def_readonly("is_keyframe", &ProcessResult::is_keyframe);

    // --- Bind the Main SVOPython Wrapper Class ---
    py::class_<SVOPython>(m, "System")
        .def(py::init<>())
        .def("initialize", &SVOPython::initialize, py::arg("config"))
        .def("process_image", &SVOPython::ProcessImage, py::arg("image"), py::arg("timestamp"))
        .def("reset", &SVOPython::reset)
        .def("get_tracking_state", &SVOPython::get_tracking_state)
        .def("get_tracking_quality", &SVOPython::get_tracking_quality)
        .def("get_all_nodes", &SVOPython::get_all_nodes)
        .def("get_all_edges", &SVOPython::get_all_edges)
        .def("get_all_map_points", &SVOPython::get_all_map_points)
        .def("get_last_num_observations", &SVOPython::getLastNumObservations)
        .def("get_last_processing_time", &SVOPython::getLastProcessingTime)
        .def("get_last_frame_id", &SVOPython::getLastFrameId);
}