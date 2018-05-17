//
// Created by fred on 4/19/18.
//

#include "OpenCV.h"

OpenCV::OpenCV() {}


void OpenCV::create2dDepthImage(std::vector<Algorithms::pts> inputCloud)
{
    int imgRow = 1280;
    int imgCol = 720;

    cv::Mat cvCloud(imgRow, imgCol, CV_8UC1, cv::Scalar(0));
    cv::Mat threshCloud(imgRow, imgCol, CV_8UC1, cv::Scalar(0));

    uchar* cvCloudPt = cvCloud.data;

    for(int i = 0; i < inputCloud.size(); i++)
    {
        static float x = 0.0f,y = 0.0f,z = 0.0f;
        static int row = 0, col = 0;
        x = (inputCloud[i].x);
        y = (inputCloud[i].y);
        z = (inputCloud[i].z);
        if (x < 1280 && y < 720)
        {
            row = (int)(x+(1280/2));
            col = (int)(y+(720/2));
            if(abs(y) < 720/2 && abs(x) < 1280/2)
            {
                if (z > 1 && z < 75)
                    cvCloudPt[row*imgCol+col] = z;
                else if(z < 750)
                    cvCloudPt[row*imgCol+col] = 0;
            }

        }
    }
    orgImage = cvCloud;
    thresholdImage = threshCloud;
    //cv::imshow("cvCloud", cvCloud);
    //cv::waitKey(0);
    //cv::waitKey(0);
}

void OpenCV::loadPlane(std::vector<Algorithms::pts> planeCloud)
{
    alg.leastSquarSVD(planeCloud);
}

void OpenCV::create2dDepthImageFromPlane(std::vector<Algorithms::pts> inputCloud)
{

    float minX = 0.0f, maxX = 0.0f,minY = 0.0f, maxY = 0.0f;
    for (int i = 0 ; i < inputCloud.size();i++)					// Find corners
    {
        if (inputCloud[i].x < minX) minX = inputCloud[i].x;
        else if  (inputCloud[i].x > maxX) maxX = inputCloud[i].x;

        if (inputCloud[i].y < minY) minY = inputCloud[i].y;
        else if  (inputCloud[i].y > maxY) maxY = inputCloud[i].y;
    }

    xMin = minX;
    yMin = minY;
    xMax = maxX;
    yMax = maxY;

    int imgRow = ((abs(xMin))+xMax)+1;
    int imgCol = ((abs(yMin))+yMax)+1;

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
        if (dist > 5)
            cvCloudPt[row*imgCol+col] = dist;
        else
            cvCloudPt[row*imgCol+col] = 0;
    }
    //std::cout << "projection done!" << std::endl;
    //cv::imshow("cvCloud", cvCloud);
    //cv::waitKey(0);
    orgImage = cvCloud;
    thresholdImage = threshCloud;

}

void OpenCV::create2dDepthImageFloat(std::vector<Algorithms::pts> inputCloud)
{

    float minX = 0.0f, maxX = 0.0f,minY = 0.0f, maxY = 0.0f;
    for (int i = 0 ; i < inputCloud.size();i++)					// Find corners
    {
        if (inputCloud[i].x < minX) minX = inputCloud[i].x;
        else if  (inputCloud[i].x > maxX) maxX = inputCloud[i].x;

        if (inputCloud[i].y < minY) minY = inputCloud[i].y;
        else if  (inputCloud[i].y > maxY) maxY = inputCloud[i].y;
    }

    xMin = minX;
    yMin = minY;
    xMax = maxX;
    yMax = maxY;

    int imgRow = ((abs(xMin))+xMax)+1;
    int imgCol = ((abs(yMin))+yMax)+1;

    cv::Mat floatImg(imgRow, imgCol, CV_32FC1, cv::Scalar(0));

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
        if (dist > 0.5)
            floatImg.at<float>(row,col) = dist;
        else
            floatImg.at<float>(row,col) = 0.0f;
    }
    floatImage = floatImg;
}

void OpenCV::threshold(char ThreshType, float pointThresh)
{
    if (ThreshType == 'N')
    {
        cv::threshold(orgImage, thresholdImage, pointThresh, 255, cv::THRESH_BINARY);
        //cv::imshow("cvCloud", thresholdImage);
        //cv::waitKey(0);
    }
    else if(ThreshType == 'I')
    {
        cv::threshold(orgImage, thresholdImage, pointThresh, 255, cv::THRESH_BINARY_INV);
        //cv::imshow("cvCloud", thresholdImage);
        //cv::waitKey(0);
    }
}

void OpenCV::findBoundingBox(float lowerDiagonolThreshold, float upperDiagonolThreshold)
{
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
        if(brDist > lowerDiagonolThreshold && brDist < upperDiagonolThreshold)
            rectThresh.push_back(boundRect[i]);
    }
    boundingBoxes = rectThresh;
    //std::cout << "Bounding boxes: " << boundingBoxes.size() << std::endl;

    //Draws the bounding boxes

    cv::Mat drawing (thresholdImage.size(), CV_8UC1, cv::Scalar(0));
    //drawing = orgImage;

    for( size_t i = 0; i < boundingBoxes.size(); i++ )
    {
        drawContours( drawing, contours_poly, (int)i, 125, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point(0,0) );
        rectangle( drawing, rectThresh[i].tl(), rectThresh[i].br(), 125, 2, 8, 0 );
    }
    draw = drawing;
    //cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE);
    //imshow( "Contours", drawing );
    //cv::imwrite("emptyTrayCon1.jpg", drawing);
    //cv::waitKey(0);


}

void OpenCV::findRoatedBoundingBox(float lowerDiagonolThreshold, float upperDiagonolThreshold)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //std::cout << contours.size() << std::endl;

    std::vector<cv::RotatedRect> minRect( contours.size() );
    std::vector<cv::Rect> rectThresh;

    //std::cout << minRect.size() << std::endl;

    for( int i = 0; i < contours.size(); i++ )
    {
        minRect[i] = cv::minAreaRect((contours[i]));

        static double brDist = 0.0;
        brDist = sqrt(pow((minRect[i].boundingRect().tl().x - minRect[i].boundingRect().br().x),2) +
                      pow((minRect[i].boundingRect().tl().x - minRect[i].boundingRect().br().x),2));
        if(brDist > lowerDiagonolThreshold && brDist < upperDiagonolThreshold)
            rectThresh.push_back(minRect[i].boundingRect());
    }
    boundingBoxes = rectThresh;
    //std::cout << boundingBoxes.size() << std::endl;

    cv::Mat drawing (thresholdImage.size(), CV_8UC1, cv::Scalar(0));
    drawing = orgImage;

    for( int i = 0; i< contours.size(); i++ )
    {
        drawContours( drawing, contours, i, 125, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );

        cv::Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( drawing, rect_points[j], rect_points[(j+1)%4], 125, 1, 8 );
    }
    //cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE);
    //imshow( "Contours", drawing );
    //cv::imwrite("emptyTrayCon1.jpg", drawing);
    //cv::waitKey(0);


}

double OpenCV::findVolumeWithinBoxes()
{
    std::cout << "Finding Volume for " << boundingBoxes.size() << " box(es) .." << std::endl;
    double volume = 0.0;
    double volumeSum = 0.0;

    for(int i = 0; i < boundingBoxes.size(); i++)
    {
        volume = cv::sum(floatImage(boundingBoxes[i]))[0];
        //std::cout << "Volume for box " << i+1 << ": " << volume << std::endl;
        volumeSum += volume;
    }
    return volumeSum;
}

std::vector<float> OpenCV::getBoundingBoxCorners()
{
    std::vector<float> boxVec;
    for(int i = 0; i < boundingBoxes.size(); i++)
    {
        boxVec.push_back(boundingBoxes[i].tl().x); //min y
        boxVec.push_back(boundingBoxes[i].tl().y); //min x
        boxVec.push_back(boundingBoxes[i].br().x); //max y
        boxVec.push_back(boundingBoxes[i].br().y); //max x
    }
    //std::cout << "Bounding boxes returned(" << boxVec.size() <<")" << std::endl;
    return boxVec;
}

void OpenCV::interPolate()
{
    //std::cout << "starting interpolation" << std::endl;
    std::vector<float> corners = getBoundingBoxCorners();
    Algorithms::pts pt;
    for(int y = corners[0] + 1; y < corners[2] - 1; y++)
        for(int x = corners[1] + 1; x < corners[3] - 1; x++)
        {
            if(floatImage.at<float>(x,y) == 0)
            {
                pt.x = x; pt.y = y; pt.z = floatImage.at<float>(x,y);
                floatImage.at<float>(x,y) = getMedian(pt);
            }
        }
}

float OpenCV::getMedian(Algorithms::pts pt)
{
    //std::cout << "get median" << std::endl;
    float perimeter[8] = {};

    perimeter[0] = floatImage.at<float>(pt.x - 1, pt.y + 1);
    perimeter[1] = floatImage.at<float>(pt.x, pt.y + 1);
    perimeter[2] = floatImage.at<float>(pt.x + 1, pt.y + 1);
    perimeter[3] = floatImage.at<float>(pt.x + 1, pt.y);
    perimeter[4] = floatImage.at<float>(pt.x + 1, pt.y - 1);
    perimeter[5] = floatImage.at<float>(pt.x, pt.y - 1);
    perimeter[6] = floatImage.at<float>(pt.x - 1, pt.y - 1);
    perimeter[7] = floatImage.at<float>(pt.x - 1, pt.y);

    float sum = 0;
    int counter = 0;
    for(int i = 0; i < 8; i++)
    {
        if(perimeter[i] != 0)
        {
            counter++;
            sum += perimeter[i];
        }
    }
    if (counter > 3)
        return (sum/counter);
    else
        return 0;
}

void OpenCV::cv1(std::vector<Algorithms::pts> inputCloud)
{
    //uchar* cvCloudPt = orgImage.data;
    create2dDepthImageFromPlane(inputCloud);
    threshold('N', 7.5);
    findBoundingBox(100, 1200);
    if(getBoundingBoxCorners().size() > 4)
    {
        orgImage = draw;

        for (int i = 0; i < boundingBoxes.size(); i++)
        {
            for (int y = boundingBoxes[i].tl().x; y < boundingBoxes[i].br().x; y++)
                for (int x = boundingBoxes[i].tl().y; x < boundingBoxes[i].br().y; x++)
                {
                    orgImage.at<uchar>(x,y) = 55;
                }
        }
        for (int i = 0; i < boundingBoxes.size(); i++)
            boundingBoxes.pop_back();

        threshold('N', 50);
        findBoundingBox(100, 1200);
    }
}

void OpenCV::cv2(std::vector<Algorithms::pts> inputCloud)
{
    create2dDepthImageFromPlane(inputCloud);
    threshold('N', 10);
    findRoatedBoundingBox(100, 950);
}
