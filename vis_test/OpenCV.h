//
// Created by fred on 4/19/18.
//

#ifndef VIS_TEST_OPENCV_H
#define VIS_TEST_OPENCV_H

#include "Algorithms.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvwimage.h>
#include <opencv2/core/core.hpp>


class OpenCV {
public:

    OpenCV();
    void create2dDepthImage(std::vector<Algorithms::pts> inputCloud);
    void loadPlane (std::vector<Algorithms::pts>);

    void create2dDepthImageFromPlane(std::vector<Algorithms::pts> inputCloud);
    void create2dDepthImageFloat(std::vector<Algorithms::pts> inputCloud);

    void threshold(char ThreshType, float pointThresh);

    void interPolate();
    float getMedian(Algorithms::pts pt);

    void cv1(std::vector<Algorithms::pts> inputCloud);
    void cv2(std::vector<Algorithms::pts> inputCloud);

    void findBoundingBox(float lowerDiagonolThreshold, float upperDiagonolThreshold);
    void findRoatedBoundingBox(float lowerDiagonolThreshold, float upperDiagonolThreshold);

    double findVolumeWithinBoxes();

    std::vector<float> getBoundingBoxCorners();


    float xMin = 0.0f;
    float xMax = 0.0f;
    float yMin = 0.0f;
    float yMax = 0.0f;



private:
    Algorithms alg;
    cv::Mat floatImage;
    cv::Mat orgImage;
	cv::Mat draw;
    cv::Mat thresholdImage;
    std::vector<cv::Rect> boundingBoxes;
    int counter = 0;

};


#endif //VIS_TEST_OPENCV_H
