#include "PclPlane.h"
#include "rsCam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>


using namespace cv;
using namespace std;

int main (int argc, char * argv[]) //try
{
    int RSx = 1280, RSy = 720, fps = 30;

    //PointCloud<PointXYZ>::Ptr empty_tray_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr empty_tray_cloud_mm (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr empty_tray_cloud_f_mm (new PointCloud<PointXYZ>);

    //PointCloud<PointXYZ>::Ptr rs_box_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr rs_box_cloud_mm (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr rs_box_cloud_f_mm (new PointCloud<PointXYZ>);

    //PointCloud<PointXYZ>::Ptr odroid_box_cloud (new PointCloud<PointXYZ>);
    //PointCloud<PointXYZ>::Ptr odroid_box_cloud_f (new PointCloud<PointXYZ>);

    PointCloud<PointXYZ>::Ptr plane_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZRGB>::Ptr common_cloud (new PointCloud<PointXYZRGB>);




    PCDReader reader;
    reader.read ("empty_tray.pcd", *empty_tray_cloud_mm);
    reader.read ("rs_box.pcd", *rs_box_cloud_mm);
    //reader.read ("odriod_box.pcd", *odroid_box_cloud);
    cout << "PCD reader done!" << endl;

    PclPlane planetest;
    planetest.insertCloud(empty_tray_cloud_mm);
    planetest.findPlane();

    empty_tray_cloud_f_mm = planetest.removeOutliers(empty_tray_cloud_mm, 'm');
    rs_box_cloud_f_mm = planetest.removeOutliers(rs_box_cloud_mm, 'm');
    cout << "Ouliers Gone!" << endl;
/*
    double residuals = 0.0;
    for(int i = 0; i < rs_box_cloud_f_mm->points.size(); i++)
    {
        static float x,y,z;
        x = rs_box_cloud_f_mm->points[i].x;
        y = rs_box_cloud_f_mm->points[i].y;
        z = rs_box_cloud_f_mm->points[i].z;
        //residuals += pow(planetest.getDistToPlane(x,y,z),2);
            cout << planetest.getDistToPlane(x,y,z) << endl;
    }
    residuals = sqrt(residuals);
    residuals = residuals/empty_tray_cloud_f_mm->points.size();
    cout << residuals << endl;
*/

    planetest.mergeSort(rs_box_cloud_f_mm, rs_box_cloud_f_mm->size());
    float xMin = planetest.sorted_x->points.front().x;
    float xMax = planetest.sorted_x->points.back().x;
    float yMin = planetest.sorted_y->points.front().y;
    float yMax = planetest.sorted_y->points.back().y;
    float yDist = abs(yMin - yMax);
    float xDist = abs(xMin - xMax);
    cout << "Number of points in plane cloud:  " << planetest.plane_cloud->points.size() << endl;
    cout <<  "Cloud Height: " << xDist << endl;
    cout <<  "Cloud Width:  " << yDist << endl;



    int imgCol = ((abs(xMin))+xMax)+1;
    int imgRow = ((abs(yMin))+yMax)+1;

    //Create Empty Mat
    Mat cvCloud(Size(imgRow, imgCol), CV_32FC1);
    Mat segCloud(Size(imgRow, imgCol), CV_8UC1);
    Mat conCloud(Size(imgRow, imgCol), CV_32FC1);
    //Get pointer to data
    //float* cvCloudPt = cvCloud.data;

    cout << "Parameters initialized.." << endl;

    for(int i = 0; i < rs_box_cloud_f_mm->points.size(); i++)
    {
        static float x = 0.0f,y = 0.0f,z = 0.0f;
        static int row = 0, col = 0;
        static float dist = 0.0f;
        x = (rs_box_cloud_f_mm->points[i].x);
        y = (rs_box_cloud_f_mm->points[i].y);
        z = (rs_box_cloud_f_mm->points[i].z);

        dist = planetest.getDistToPlane(x, y, z); //Distance to the plane in mm
        row = (int)(x+(abs(xMin)));
        col = (int)(y+(abs(yMin)));

        if(dist > 7.5)
            cvCloud.at<float>(row,col) = dist;
        else
            cvCloud.at<float>(row,col) = 0.0f;

    }
    //imshow("pic1", cvCloud);
    //waitKey(0);

    threshold(cvCloud,segCloud,10.0, 255, THRESH_BINARY);
    cout << "Thresh done" << endl;
    vector<vector<Point>> contours;
    findContours(segCloud, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));


    //imshow("con",conCloud);
    //waitKey(0);

    double volume = 0.0;
    for(int x = 0; x < imgCol; x++)
        for(int y = 0; y < imgRow; y++)
        {
            if (segCloud.at<float>(x,y) == 65536.0)
                volume += cvCloud.at<float>(x,y);
        }
    cout << volume << endl;
    imshow("pic2", segCloud);
    waitKey(0);

/*
    cout << "Black Spots before resizing: " << blackSpots << endl;
    cout << "Min x: " << minBarrierX << "  Max x: " << maxBarrierX << endl;
    cout << "Min y: " << minBarrierX << "  Max y: " << maxBarrierX << endl;

    double volume = 0.0;
    blackSpots = 0;
    float perimeter[8];
    float interpolationSum = 0.0f;
    int interpolationCounter = 0;
    for(int x = minBarrierX; x < maxBarrierX; x++)
        for(int y = minBarrierY; y < maxBarrierY; y++)
        {
            if(imgCloud[x][y] == 0)
            {
                blackSpots++;
                perimeter[0] = imgCloud[x - 1][y + 1];
                perimeter[1] = imgCloud[x][y + 1];
                perimeter[2] = imgCloud[x + 1][y + 1];

                perimeter[3] = imgCloud[x + 1][y];
                perimeter[4] = imgCloud[x + 1][y - 1];
                perimeter[5] = imgCloud[x][y - 1];

                perimeter[6] = imgCloud[x - 1][y - 1];
                perimeter[7] = imgCloud[x - 1][y];

                for(int i = 0; i < 8; i++)
                    if (perimeter[i] != 0)
                    {
                        interpolationSum += perimeter[i];
                        interpolationCounter++;
                    }
                if (interpolationCounter != 0)
                    imgCloud[x][y] = (interpolationSum / interpolationCounter);
            }
            volume += imgCloud[x][y];
        }
    cout << "Black spots after resizing: " << blackSpots << endl;
    cout << "Volume: " << volume << endl;
*/
    //planetest.visualizeCloud(empty_tray_cloud_f_mm);
    //planetest.visualizeColorCloud(planetest.mergeCloudsColor(empty_tray_cloud_f_mm, 'r', rs_box_cloud_f_mm, 'g'));

    //common_cloud->clear();
    //PclPlane comPlane;
    //common_cloud = comPlane.mergeCloudsColor(planetest.plane_cloud,'g', org_empty_tray_cloud,'b');
    //comPlane.visualizeColorCloud(common_cloud);
    //comPlane.visualizeCloud(planetest.plane_cloud);



    //return EXIT_SUCCESS;
}
/*
catch (const error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    return EXIT_FAILURE;
}
catch (const exception & e)
{
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
*/

/*
    PclPlane rsc;
    PointXYZ camPoint;
    rsCam Stcam(RSx,RSy,fps);
    Stcam.startStream();

    auto rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        camPoint.x = rsFrame[i].x * 1000.0f;
        camPoint.y = rsFrame[i].y * 1000.0f;
        camPoint.z = rsFrame[i].z * 1000.0f;
        empty_tray_cloud_mm->push_back(camPoint);
    }

    io::savePCDFileASCII ("empty_tray.pcd", *empty_tray_cloud_mm);
    //rsc.visualizeCloud(empty_tray_cloud_mm);
    cout << "ENTET ANY KEY1 ... " << endl;
    cin.get();
    cout << "... " << endl;

    auto rsFrame2 = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        camPoint.x = rsFrame2[i].x * 1000.0f;
        camPoint.y = rsFrame2[i].y * 1000.0f;
        camPoint.z = rsFrame2[i].z * 1000.0f;
        rs_box_cloud_mm->push_back(camPoint);
    }

    io::savePCDFileASCII ("rs_box.pcd", *rs_box_cloud_mm);
    //rsc.visualizeCloud(rs_box_cloud_mm);
*/