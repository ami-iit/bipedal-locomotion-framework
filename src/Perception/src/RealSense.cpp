/**
 * @file RealSense.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Perception/Capture/RealSense.h>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <tuple>

using namespace BipedalLocomotion::Perception::Capture;

struct RealSense::Impl
{
    bool isValidCamera(const std::string& camName);
    bool startStream();
    void stopStream();
    
    /**
     * Copied from Realsense examples 
     * see https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp
     */
    std::tuple<int, int, int> rgbTexture(rs2::video_frame textureImg, rs2::texture_coordinate textureXY);
    void toPCL(const rs2::points& points, 
               const rs2::video_frame& color, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        
    bool isColorEnabled{true};
    bool isIREnabled{false};
    bool isDepthEnabled{false};
    bool isPCLEnabled{false};

    int genericWidth{640};
    int genericHeight{480};
    int genericFPS{30};

    rs2_format colorFormat{RS2_FORMAT_BGR8};
    rs2_format irFormat{RS2_FORMAT_Y8};
    rs2_format depthFormat{RS2_FORMAT_Z16};
        
    bool isStreaming{false};
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    
    rs2::pointcloud pointcloud;
    rs2::points points;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    rs2::frame irFrame;
    
    rs2::colorizer color_map;
    bool doAlignToColor{false};
    std::unique_ptr<rs2::align> alignToColor;

    std::string camName{"RealSense"};
};

RealSense::RealSense() : m_pimpl(std::make_unique<Impl>())
{
}

RealSense::~RealSense() 
{
    m_pimpl->stopStream();
}

bool RealSense::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view logPrefix = "[RealSense::initialize] ";
    auto ptr = handler.lock();
    if (ptr == nullptr) { return false; }

    if (!ptr->getParameter("camera_name", m_pimpl->camName))
    {
        std::cout << logPrefix << " Parameter \"camera_name\" not available in the configuration." 
                  << "Using default name \"RealSense\"." << std::endl;
    }
    
    if (!ptr->getParameter("frame_width", m_pimpl->genericWidth))
    {
        std::cout << logPrefix << " Parameter \"frame_width\" not available in the configuration." 
                  << "Using default value \"640\"." << std::endl;
    }
    
    if (!ptr->getParameter("frame_height", m_pimpl->genericHeight))
    {
        std::cout << logPrefix << " Parameter \"frame_width\" not available in the configuration." 
                  << "Using default value \"480\"." << std::endl;
    }
    
    if (!ptr->getParameter("fps", m_pimpl->genericFPS))
    {
        std::cout << logPrefix << " Parameter \"frame_width\" not available in the configuration." 
                  << "Using default value \"30\"." << std::endl;
    }
    
    if (!ptr->getParameter("stream_color", m_pimpl->isColorEnabled))
    {
        std::cout << logPrefix << " Parameter \"stream_color\" not available in the configuration." 
                  << "Color stream will not be available." << std::endl;
    }
    
    if (!ptr->getParameter("stream_depth", m_pimpl->isDepthEnabled))
    {
        std::cout << logPrefix << " Parameter \"stream_depth\" not available in the configuration." 
                  << "Depth stream will not be available." << std::endl;
    }
    
    if (!ptr->getParameter("stream_ir", m_pimpl->isIREnabled))
    {
        std::cout << logPrefix << " Parameter \"stream_ir\" not available in the configuration." 
                  << "IR stream will not be available." << std::endl;
    }
    
    if (!ptr->getParameter("stream_pcl", m_pimpl->isPCLEnabled))
    {
        std::cout << logPrefix << " Parameter \"stream_pcl\" not available in the configuration." 
                  << "PCL stream will not be available." << std::endl;
    }
        
    if (!ptr->getParameter("align_frames_to_color", m_pimpl->doAlignToColor))
    {
        std::cout << logPrefix << " Parameter \"align_frames_to_color\" not available in the configuration." 
                  << "Frames will not be aligned to color frame." << std::endl;
    }
    
    if (!m_pimpl->startStream())
    {
        return false;
    }
    
    if (m_pimpl->doAlignToColor)
    {
        m_pimpl->alignToColor = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    }
    return true;
}

bool RealSense::isValid()
{
    if (!m_pimpl->isStreaming)
    {
        std::cerr << "[RealSenseCapture::isValid] Stream not active." << std::endl;
        return false;
    }
    
    return true;
}

bool RealSense::Impl::isValidCamera(const std::string& _camName)
{
    if (_camName != camName)
    {
        std::cerr << "[RealSenseCapture::Impl::isValidCamera] Requested camera not available" << std::endl;
        return false;
    }
    
    return true;
}

bool RealSense::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    rgbCamerasList.clear();
    rgbCamerasList.push_back(m_pimpl->camName);
    return true;
}

bool RealSense::getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList)
{
    rgbdCamerasList.clear();
    rgbdCamerasList.push_back(m_pimpl->camName);
    return true;
}

bool RealSense::getPCLDevicesList(std::vector<std::string>& pclDevList)
{
    pclDevList.clear();
    pclDevList.push_back(m_pimpl->camName);
    return true;
}

bool RealSense::getColorImage(const std::string& camName,
                              cv::Mat& colorImg,
                              double* receiveTimeInSeconds)
{    
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }
    
    if (!m_pimpl->isColorEnabled)
    {
        std::cerr << "[RealSenseCapture::getColorImage] Color stream was not enabled." << std::endl;
        return false;
    }
    
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->colorFrame = m_pimpl->frames.get_color_frame();
    colorImg = cv::Mat(cv::Size(m_pimpl->genericWidth, m_pimpl->genericHeight),
                       CV_8UC3, (void*)m_pimpl->colorFrame.get_data(), cv::Mat::AUTO_STEP);
    
    return true;
}


bool RealSense::getDepthImage(const std::string& camName,
                              cv::Mat& depthImg,
                              double* receiveTimeInSeconds)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }
    
    if (!m_pimpl->isDepthEnabled)
    {
        std::cerr << "[RealSenseCapture::getDepthImage] Depth stream was not enabled." << std::endl;
        return false;
    }
    
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame();
    
    depthImg = cv::Mat(cv::Size(m_pimpl->genericWidth, m_pimpl->genericHeight),
                                CV_8UC1, (void*)m_pimpl->depthFrame.get_data(), cv::Mat::AUTO_STEP);
        
    return true;
}

bool RealSense::getColorizedDepthImage(const std::string& camName,
                                       cv::Mat& depthImg,
                                       double* receiveTimeInSeconds)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }
    
    if (!m_pimpl->isDepthEnabled)
    {
        std::cerr << "[RealSenseCapture::getDepthImage] Depth stream was not enabled." << std::endl;
        return false;
    }
    
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame().apply_filter(m_pimpl->color_map);
    
    depthImg = cv::Mat(cv::Size(m_pimpl->genericWidth, m_pimpl->genericHeight),
                                CV_8UC3, (void*)m_pimpl->depthFrame.get_data(), cv::Mat::AUTO_STEP);
        
    return true;
}

bool RealSense::getInfraredImage(const std::string& camName,
                                 cv::Mat& irImage,
                                 double* receiveTimeInSeconds)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }
    
    if (!m_pimpl->isIREnabled)
    {
        std::cerr << "[RealSenseCapture::getInfraredImage] IR stream was not enabled." << std::endl;
        return false;
    }
    
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->irFrame = m_pimpl->frames.get_infrared_frame();
    irImage = cv::Mat(cv::Size(m_pimpl->genericWidth, m_pimpl->genericHeight),
                               CV_8UC1, (void*)m_pimpl->irFrame.get_data(), cv::Mat::AUTO_STEP);
        
    return true;
}

bool RealSense::getPointCloud(const std::string& pclDevName,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud,
                              double* receiveTimeInSeconds)
{        
    if (!isValid() || !m_pimpl->isValidCamera(pclDevName))
    {
        return false;
    }
    
    if (!m_pimpl->isPCLEnabled)
    {
        std::cerr << "[RealSenseCapture::getPointCloud] PCL stream was not enabled." << std::endl;
        return false;
    }
    
    if (coloredPointCloud == nullptr)
    {
        std::cerr << "[RealSenseCapture::getPointCloud] Did not receive a valid pointer." << std::endl;
        return false;
    }
    
    // Wait for the next set of frames from the camera
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    m_pimpl->points = m_pimpl->pointcloud.calculate(m_pimpl->depthFrame);

    m_pimpl->colorFrame = m_pimpl->frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    m_pimpl->pointcloud.map_to(m_pimpl->colorFrame);
    
    m_pimpl->toPCL(m_pimpl->points, m_pimpl->colorFrame, coloredPointCloud);
    return true;
}

bool RealSense::Impl::startStream()
{
    if (isPCLEnabled)
    {
        if (!isColorEnabled)
        {
            isColorEnabled = true;
        }
        
        if (!isDepthEnabled)
        {
            isDepthEnabled = true;
        }
        
        std::cout << "[RealSenseCapture::Impl::startStream] PCL stream enabled. Automatically enabling RGB and Depth streaming." << std::endl;
    }
    
    if (!isColorEnabled &&
        !isIREnabled &&
        !isDepthEnabled)
    {
        std::cerr << "[RealSenseCapture::Impl::startStream] None of the stream options enabled. Cannot start streaming." << std::endl;
        return false;
    }
        
    if (isColorEnabled)
    {
        cfg.enable_stream(RS2_STREAM_COLOR, genericWidth, genericHeight, colorFormat, genericFPS);
    }

    if (isIREnabled)
    {
        cfg.enable_stream(RS2_STREAM_INFRARED, genericWidth, genericHeight, irFormat, genericFPS);
    }

    if (isDepthEnabled)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, genericWidth, genericHeight, depthFormat, genericFPS);
    }

    std::cout << "[RealSenseCapture::Impl::startStream] Trying to connect to device." << std::endl;
    try
    {
        pipe.start(cfg);
    }
    catch (const rs2::error& e)
    {
        std::cerr << "[RealSenseCapture::Impl::startStream] Failed to start the pipeline:"<< "(" << e.what() << ")" << std::endl;
        return false;
    }
        
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    isStreaming = true;
    return true;
}

void RealSense::Impl::stopStream()
{
    pipe.stop();
    cfg.disable_all_streams();
    isStreaming = false;
}


//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RealSense::Impl::rgbTexture(rs2::video_frame textureImg, rs2::texture_coordinate textureXY)
{
    // Get Width and Height coordinates of texture
    int width  = textureImg.get_width();  // Frame width in pixels
    int height = textureImg.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int xValue = std::min(std::max(int(textureXY.u * width  + .5f), 0), width - 1);
    int yValue = std::min(std::max(int(textureXY.v * height + .5f), 0), height - 1);

    int bytes = xValue * textureImg.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = yValue * textureImg.get_stride_in_bytes(); // Get line width in bytes
    int textureIndex =  (bytes + strides);

    const auto newTexture = reinterpret_cast<const uint8_t*>(textureImg.get_data());
    
    // RGB components to save in tuple
    int NT1 = newTexture[textureIndex];
    int NT2 = newTexture[textureIndex + 1];
    int NT3 = newTexture[textureIndex + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}


//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
void RealSense::Impl::toPCL(const rs2::points& points, 
                            const rs2::video_frame& color, 
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> rgbColor;
    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    if (cloud == nullptr)
    {
        return;
    }
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto textureCoord = points.get_texture_coordinates();
    auto vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = vertex[i].x;
        cloud->points[i].y = vertex[i].y;
        cloud->points[i].z = vertex[i].z;

        // Obtain color texture for specific point
        rgbColor = rgbTexture(color, textureCoord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = std::get<2>(rgbColor); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(rgbColor); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(rgbColor); // Reference tuple<0>
    }           
}

