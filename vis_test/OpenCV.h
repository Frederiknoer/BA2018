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
    void loadPlane (std::vector<Algorithms::pts>);
    void create2dDepthImage(std::vector<Algorithms::pts> inputCloud);
    void findBoundingBox(float pointThresh);
    double findVolumeWithinBoxes();
    void drawBoundingBoxes(cv::Mat inputImg);



private:
    Algorithms alg;
    cv::Mat floatImage;
    cv::Mat orgImage;
    cv::Mat thresholdImage;
    std::vector<cv::Rect> boundingBoxes;

};


#endif //VIS_TEST_OPENCV_H
