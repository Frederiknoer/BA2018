//
// Created by fred on 4/19/18.
//

#include "OpenCV.h"

OpenCV::OpenCV() {}

void OpenCV::loadPlane(std::vector<Algorithms::pts> planeCloud)
{
    alg.leastSquarSVD(planeCloud);
}

void OpenCV::create2dDepthImage(std::vector<Algorithms::pts> inputCloud)
{
    std::vector<Algorithms::pts> SortedX = alg.mergeSortX(inputCloud);
    std::vector<Algorithms::pts> SortedY = alg.mergeSortY(inputCloud);
    float xMin = SortedX.front().x;
    float xMax = SortedX.back().x;
    float yMin = SortedY.front().y;
    float yMax = SortedY.back().y;


    int imgRow = ((abs(xMin))+xMax)+1;
    int imgCol = ((abs(yMin))+yMax)+1;

    cv::Mat floatImg(imgRow, imgCol, CV_32FC1, cv::Scalar(0));
    cv::Mat cvCloud(imgRow, imgCol, CV_8UC1, cv::Scalar(0));
    cv::Mat threshCloud(imgRow, imgCol, CV_8UC1, cv::Scalar(0));

    uchar* cvCloudPt = cvCloud.data;

    for(int i = 0; i < inputCloud.size(); i++)
    {
        static float x = 0.0f,y = 0.0f,z = 0.0f;
        static int row = 0, col = 0;
        static float dist = 0.0f;
        x = (inputCloud[i].x);
        y = (inputCloud[i].y);
        z = (inputCloud[i].z);

        dist = alg.getDistToPlane(x, y, z); //Distance to the plane in mm

        row = (int)(x+(abs(xMin)));
        col = (int)(y+(abs(yMin)));
        if (dist > 2.5)
        {
            cvCloudPt[row*imgCol+col] = dist;
            floatImg.at<float>(row,col) = dist;
        }
        else
        {
            cvCloudPt[row*imgCol+col] = 0;
            floatImg.at<float>(row,col) = 0.0f;
        }
    }
    orgImage = cvCloud;
    thresholdImage = threshCloud;
    floatImage = floatImg;
    cv::imshow("cvCloud", cvCloud);
    cv::waitKey(0);

}

void OpenCV::findBoundingBox(float pointThresh)
{
    cv::threshold(orgImage, thresholdImage, pointThresh, 255, cv::THRESH_BINARY );
    cv::imshow("cvCloud", thresholdImage);
    cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Rect> rectThresh;

    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::approxPolyDP((contours[i]), contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect( (contours_poly[i]) );

        static double brDist = 0.0;
        brDist = sqrt(pow((boundRect[i].tl().x - boundRect[i].br().x),2) + pow((boundRect[i].tl().x - boundRect[i].br().x),2));
        if(brDist > 100.0)
            rectThresh.push_back(boundRect[i]);
    }
    boundingBoxes = rectThresh;

    //Draws the bounding boxes
    cv::Mat drawing (thresholdImage.size(), CV_8UC1, cv::Scalar(0));
    drawing = orgImage;

    for( size_t i = 0; i < boundingBoxes.size(); i++ )
    {
        drawContours( drawing, contours_poly, (int)i, 125, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point(0,0) );
        rectangle( drawing, rectThresh[i].tl(), rectThresh[i].br(), 125, 2, 8, 0 );
    }
    cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE);
    imshow( "Contours", drawing );
    cv::waitKey(0);

}

double OpenCV::findVolumeWithinBoxes()
{
    std::cout << "Finding Volume for " << boundingBoxes.size() << " boxes .." << std::endl;
    double volume = 0.0;
    for(int i = 0; i < boundingBoxes.size(); i++)
        volume += cv::sum(floatImage(boundingBoxes[i]))[0];
    return volume;
}

void OpenCV::drawBoundingBoxes(cv::Mat inputImg)
{
    cv::Mat drawing (thresholdImage.size(), CV_8UC1, cv::Scalar(0));

    for( size_t i = 0; i < boundingBoxes.size(); i++ )
    {
        //drawContours( drawing, contours_poly, (int)i, 125, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point(0,0) );
        //rectangle( drawing, rectThresh[i].tl(), rectThresh[i].br(), 125, 2, 8, 0 );
        //cout << boundRect[i].tl() << " - " << boundRect[i].br() << endl;
    }
    //cv::imwrite("pgmtest.pgm" , drawing);
    cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE);
    imshow( "Contours", drawing );
    cv::waitKey(0);
}
