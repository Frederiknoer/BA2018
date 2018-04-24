#include "Algorithms.h"
#include "PclPlane.h"
#include "rsCam.h"
#include "OpenCV.h"
#include <iostream>
//#include "nr3.h"
//#include "utilities.h"
//#include "mcintegrate.h"

using namespace std;


int main (int argc, char * argv[]) try
{
    int RSx = 1280, RSy = 720, fps = 30;

    rsCam Stcam(RSx,RSy,fps);

    int camPlace;
    int method = 0;
    cout << "Enter if camera is over inlet conveyor(1) or inside X-Ray(2) ..." << endl;
    cin >> camPlace;
    cout << endl << "Choose method of operation; PointCloud subtraction(0), OpenCV 2D image projection(1), "
            "PCL Nearest Neighbor trapzoidal(2), OpenCV 2D image projection with minRect function(3)" << endl;
    cin >> method;
    cout << endl;


    //Create objects
    Algorithms algo;
    OpenCV ocvWS;

    std::vector<Algorithms::pts> emptyTrayVec(RSx*RSy);
    std::vector<Algorithms::pts> objVec(RSx*RSy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Taking picture of empty plane..." << endl;
    Stcam.startStream();
    auto rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 100.0f; //This is only for finding workspace!!!!!
    }
    ocvWS.create2dDepthImage(emptyTrayVec);

    if(camPlace == 1)
        ocvWS.threshold('N', 47.5);
    else if(camPlace == 2)
        ocvWS.threshold('N', 35.0);

    ocvWS.findBoundingBox(500, 1500);
    std::vector<float> outlierVector = ocvWS.getBoundingBoxCorners();

    //delete ocvWS;
    //emptyTrayVec.clear();

    cout << "Workspace has been found!" << endl << "Initialising plane estimation..." << endl;

    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 1000.0f;
    }
    std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(emptyTrayVec, outlierVector, 0, 0);

    cout << "Plane estimation Done! " << endl << "Insert garments and press enter: .. " << endl;
    cin.get();
    for(int a = 0; a < 1000; a++)
    {
        cout << "Press Enter" << endl;
        cin.get();
        cout << "... " << endl;
        rsFrame = Stcam.RqSingleFrame();
        if(method < 3)
            for(int i = 0; i < (RSx*RSy); i ++)
            {
                objVec[i].x = rsFrame[i].x * 1000.0f;
                objVec[i].y = rsFrame[i].y * 1000.0f;
                objVec[i].z = rsFrame[i].z * 1000.0f;
            }
        else if (method == 3)
            for(int i = 0; i < (RSx*RSy); i ++)
            {
                objCloud->points[i].x = rsFrame[i].x * 1000.0f;
                objCloud->points[i].y = rsFrame[i].y * 1000.0f;
                objCloud->points[i].z = rsFrame[i].z * 1000.0f;
            }

        switch(method)
        {
            case 0 :
            {
                std::vector<Algorithms::pts> objVec_f = algo.removeOutliers(objVec, outlierVector, abs(ocvWS.xMin), abs(ocvWS.yMin));
                double traysum = 0.0; double objsum = 0.0;

                for(int i = 0; i < emptyTrayVec_f.size(); i++)
                    traysum += sqrt(pow(emptyTrayVec_f[i].x ,2) + pow(emptyTrayVec_f[i].y ,2) + pow(emptyTrayVec_f[i].z ,2));

                for(int i = 0; i < objVec_f.size(); i++)
                    objsum += sqrt(pow(objVec_f[i].x ,2) + pow(objVec_f[i].y ,2) + pow(objVec_f[i].z ,2));

                cout << "Volume: " << abs(traysum-objsum) << endl;

                break;
            }
            case 1 : //OpenCv 2Dimage projection
            {
                OpenCV ocvGarment;
                if(a == 0)
                    ocvGarment.loadPlane(algo.removeOutliers(emptyTrayVec, outlierVector, 0, 0));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVector, 0, 0));
                ocvGarment.threshold('N', 10);
                ocvGarment.findBoundingBox(100, 750);
                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
                break;
            }
            case 2 : //OpenCv 2Dimage projection with roatated rectangle
            {
                OpenCV ocvGarment;
                if(a == 0)
                    ocvGarment.loadPlane(algo.removeOutliers(emptyTrayVec, outlierVector, 0, 0));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVector, 0, 0));
                ocvGarment.threshold('N', 10);
                ocvGarment.findRoatedBoundingBox(100, 750);
                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
            }
            case 3 : //PCL nearest neighbor trapzoidal
            {
                PclPlane pclObj; //Static?

                static int K = 2;
                static int NofNeighbor = 0;
                PointXYZ A, B, C;
                PointXYZ searchPoint;

                std::vector<int> pointIdx(K);
                std::vector<float> pointNKNSquaredDist(K);

                KdTreeFLANN<PointXYZ> kdtree;
                static double area = 0.0;
                static double volume = 0.0;
                static double sumVolume;
                if(a == 0)
                {
                    pclObj.insertCloud(emptyTrayVec_f);
                    pclObj.findPlane();
                    sumVolume = 0.0;
                }

                PointCloud<PointXYZ>::Ptr obj_cloud_f (new PointCloud<PointXYZ>);
                obj_cloud_f = pclObj.removeOutliers(objCloud, outlierVector, abs(ocvWS.xMin), abs(ocvWS.yMin));

                kdtree.setInputCloud(obj_cloud_f);

                for(int i = 0; i < obj_cloud_f->points.size(); i++)
                {
                    searchPoint = obj_cloud_f->points[i];
                    NofNeighbor = kdtree.nearestKSearch (searchPoint, K, pointIdx, pointNKNSquaredDist);
                    if(NofNeighbor == 2)
                    {
                        A = searchPoint; B = obj_cloud_f->points[pointIdx[0]]; C = obj_cloud_f->points[pointIdx[1]];
                        area = 0.5*abs((A.x - C.x)*(B.y - A.y)-(A.x - B.x)*(C.y - A.y)); //Check this equation for triangle area

                        volume = area * ((pclObj.getDistToPlane(A.x, A.y, A.z) + pclObj.getDistToPlane(B.x, B.y, B.z) + pclObj.getDistToPlane(C.x, C.y, C.z))/3);
                        sumVolume += volume;
                        //ERASE POINT HERE
                    }
                }
                cout << "Volume: " << sumVolume << endl;
                break;
            }
            default :
            {
                cout << "Invalid option chosen!" << endl;
                break;
            }
        }

    }

/*
    //Create objects
    Algorithms algo;
    OpenCV ocvWS;
    OpenCV ocvGarment;

    std::vector<Algorithms::pts> emptyTrayVec(RSx*RSy);
    std::vector<Algorithms::pts> objVec(RSx*RSy);

    cout << "Taking picture of empty plane..." << endl;
    Stcam.startStream();
    auto rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 100.0f; //This is only for finding workspace!!!!!
    }
    ocvWS.create2dDepthImage(emptyTrayVec);

    if(camPlace == 1)
        ocvWS.threshold('N', 47.5);
    else if(camPlace == 2)
        ocvWS.threshold('N', 35.0);

    ocvWS.findBoundingBox(500, 1500);
    std::vector<float> outlierVector = ocvWS.getBoundingBoxCorners();

    //delete ocvWS;
    //emptyTrayVec.clear();

    cout << "Workspace has been found!" << endl << "Initialising plane estimation..." << endl;

    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 1000.0f;
    }
    ocvGarment.loadPlane(algo.removeOutliers(emptyTrayVec, outlierVector, 0, 0));
    cout << "Plane estimation Done! " << endl << "Insert garments and press enter: .. " << endl;
    cin.get();
    cin.get(); //FIIIX!
    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        objVec[i].x = rsFrame[i].x * 1000.0f;
        objVec[i].y = rsFrame[i].y * 1000.0f;
        objVec[i].z = rsFrame[i].z * 1000.0f;
    }
    ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVector, 0, 0));
    ocvGarment.threshold('N', 10);
    ocvGarment.findBoundingBox(100, 750);
    cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
*/

    return EXIT_SUCCESS;
}

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