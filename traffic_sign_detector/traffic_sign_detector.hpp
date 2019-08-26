#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <chrono>
#include <ros/package.h>


#include </opt/mrtsoftware/release/include/opencv-3.3.1/opencv2/opencv.hpp>
//#include <opencv2/opencv.hpp>

#include "traffic_sign_detector_ros_tool/TrafficSignDetectorInterface.h"
#include <sensor_msgs/Image.h>
#include "traffic_sign_detector_ros_tool/Sign.h"

namespace traffic_sign_detector_ros_tool {

class TrafficSignDetector {

    using Interface = TrafficSignDetectorInterface;
    using ReconfigureConfig = TrafficSignDetectorConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using Msg = sensor_msgs::Image;

public:
    TrafficSignDetector(ros::NodeHandle, ros::NodeHandle);

private:
    void setupDiagnostics();
    void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper&);

    void callbackSubscriber(const Msg::ConstPtr& msg);
    void depthCallback(const Msg::ConstPtr& msg);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);


    // DATASET LOADING
    /**
     *@brief Loads all the images from the dataset by type of sign and labels them
     *@param trainImgs is an output vector that contains all the training images
     *@param trainLabels is an output vector that contains all the labels of the training images
     *@return none
     */
    void loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);

    // DATASET LOADING
    /**
     *@brief Loads all the images from the dataset by type of sign and labels them
     *@param trainImgs is an output vector that contains all the training images
     *@param trainLabels is an output vector that contains all the labels of the training images
     *@return none
     */
    void loadTestingImgs(std::vector<cv::Mat> &testImgs);

    /**
     *@brief Apply gaussian filter to the input image to denoise it
     *@param input is the current image to be filtered
     *@return Blurred and denoised image
     */
    cv::Mat deNoise(cv::Mat input);

    /**
     *@brief The image is normalized w.r.t. red and blue.
     *@param input is the current image to be normalized
     *@return normalized image for MSER detection
     */
    cv::Mat preproc(cv::Mat input);

    // MSER FEATURES
    /**
     *@brief Find MSER features in the image. First normalize image and binarize, then look for regions and finally, resize each regions image.
     *@param img is the current image where MSER features are detected
     *@param area is an output variable that stores the area of each region's bounding box
     *@return Vector of all the images of detections
     */
    std::vector<cv::Mat> roiDetection(cv::Mat input,cv::Mat raw, double &area);

    // HOG FEATURES
    /**
     *@brief Compute the HOG features in every image of a vector
     *@param hog is the HOG Descriptor object with all the parameters set up
     *@param imgs is the vector of images to get theHOG from
     *@return Matrix with the HOG Descriptor for all the images.
     */
    cv::Mat hogFeatures(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);

    // TRAINING
    /**
     *@brief Function that runs loadTrainImgs(), HOG_Features() and SVM Training, sequentially
     *@param hog is the HOG Descriptor object with all the parameters set up
     *@param svm is the Support vector Machine object
     *@param trainImgs is an output vector that contains all the training images
     *@param trainLabels is an output vector that contains all the labels of the training images
     */
    void trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
        std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);
    // TESTING
    /**
     *@brief Function that runs loadTrainImgs(), HOG_Features() and SVM Testing, sequentially
     *@param hog is the HOG Descriptor object with all the parameters set up
     *@param svm is the Support vector Machine object
     *@param testImgs is an output vector that contains all the training images
     */
    void testStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
        std::vector<cv::Mat> &testImgs);

    // TRAIN SVM
    /**
     *@brief Sets all the parameters of the SVM object and feeds it with the HOG features of the labeled training images.
     *@param svm is the Support vector Machine object
     *@param trainHOG is the cv::Mat with the HOG features from the training images
     *@param trainLabels is the vector with all the labels for the training images
     *@return none
     */
    void svmTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHog, std::vector<int> trainLabels);

    // CLASSIFICATION
    /**
     *@brief Feed SVM with the HOG Matrix of the current image and outputs its label.
     *@param testHOG is the HOG Descriptor object with all the parameters set up
     *@param svm is the Support vector Machine object
     *@return Labels of the tested set of features. Label of the sign being recognized.
     */
    int svmTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHog);

    // VISUALIZATION
    /**
     *@brief Opens a vindow with the robot's view of the workspace. It outputs bouding box around detected signs and the name of the sign.
     *@param boxes and input image
     *@return none
     */
    void visualization(cv::Mat &img, cv::Rect rect);

    /**
     * @brief Save an image from every detection in a folder according to its label
     * @param
     * @return none
     */
    void saveDetection(cv::Mat detection);

    /**
     * @brief calculate the distance of a traffic sign
     * @param
     * @return none
     */
    int calcDistance(cv::Mat &imgDepth, cv::Rect box);


    Interface interface_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    diagnostic_updater::Updater updater_; ///< Diagnostic updater
    // std::unique_ptr<diagnostic_updater::DiagnosedPublisher<std_msgs::Header>> publisherDiagnosed_;
    diagnostic_msgs::DiagnosticStatus diagnosticStatus_; ///< Current diagnostic status

    cv::Mat imgColor;  /// OpenCV Image from the camera given by the subscriber
    cv::Mat imgDepth;
    cv::Mat imgDenoise;
    cv::Mat imgRBNorm;
    cv::Size size;

    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    std::vector<cv::Mat> testImgs;




    cv::Mat trainHog;
    cv::Mat testHog;

    cv::Ptr<cv::ml::SVM> svm;
    std::vector<cv::Rect> boxes;  /// Bounding boxes in the current frame
    int winSize;
    int counter;
    std::string pkgPath;


    int traffic_sign;  /// The label of the detections, outputed by the SVM
    int distance;
    double area;
};
} // namespace traffic_sign_detector_ros_tool
