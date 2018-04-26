#include "Algorithms.h"
#include "PclPlane.h"
#include "rsCam.h"
#include "OpenCV.h"
#include "SafeQueue.h"
#include <thread>
#include <iostream>
#include <cstdlib>
#include <pthread.h>


using namespace std;

#define RSx 1280
#define RSy 720
#define fps 6

static SafeQueue sq;

void volumeEstimate()
{
    //const rs2::vertex *rsFrame;
    int camPlace;
    int method = 0;
    int convSpeed = 0;
    cout << "Enter if camera is over inlet conveyor(1) or inside X-Ray(2) ..." << endl;
    cin >> camPlace;
    //cout << endl << "Enter conveyor speed ..." << endl;
    //cin >> convSpeed;
    cout << endl << "Choose method of operation; PointCloud subtraction(0), OpenCV 2D image projection(1), "
            "PCL Nearest Neighbor trapzoidal(2), OpenCV 2D image projection with minRect function(3), "
            "PCL triangulation(4)"<< endl;
    cin >> method;
    cout << endl;

    Algorithms algo;
    OpenCV ocvWS;

    std::vector<Algorithms::pts> emptyTrayVec(RSx*RSy);
    std::vector<Algorithms::pts> objVec(RSx*RSy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZ>);
    PointXYZ tempPoint;

    cout << "Taking picture of empty plane..." << endl;

    while(sq.queueEmpty);
    auto rsFrame = sq.pull();
    cout << "Pulled" << endl;
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        //cout << i << endl;
        //cout << rsFrame[i].x << endl;
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 100.0f; //This is only for finding workspace!!!!!
    }
    cout << "test" << endl;
    ocvWS.create2dDepthImage(emptyTrayVec);

    if(camPlace == 1)
        ocvWS.threshold('N', 47.5);
    else if(camPlace == 2)
        ocvWS.threshold('N', 35.0); //Still needs adjustment

    ocvWS.findBoundingBox(500, 1500);
    std::vector<float> outlierVector = ocvWS.getBoundingBoxCorners();
    if(outlierVector.size() > 4)
        cout << "bad WS" << endl;
    cout << "X dist(WS): " << outlierVector[0] << "-" << outlierVector[2] << "   Y dist(WS): " << outlierVector[1] << "-" << outlierVector[3] << endl;
    outlierVector[0] = outlierVector[0] + 15; //x
    outlierVector[1] = outlierVector[1] + 25; //y
    outlierVector[2] = outlierVector[2] - 15; //x
    outlierVector[3] = outlierVector[3] - 250; //y


    cout << "Workspace has been found!" << endl << "Initialising plane estimation..." << endl;

    while(sq.queueEmpty);
    rsFrame = sq.pull();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 1000.0f;
    }


    cout << "Plane estimation Done! " << endl << "Insert garments and press enter: .. " << endl;
    cin.get();
    while(true)
    {
        cout << "Press Enter" << endl;
        cin.get();
        cout << "... " << endl;

        while(sq.queueEmpty);
        rsFrame = sq.pull();
        for(int i = 0; i < (RSx*RSy); i ++)
        {
            objVec[i].x = rsFrame[i].x * 1000.0f;
            objVec[i].y = rsFrame[i].y * 1000.0f;
            objVec[i].z = rsFrame[i].z * 1000.0f;
        }
        /*
        objCloud->clear();
            for(int i = 0; i < (RSx*RSy); i ++)
            {
                tempPoint.x = newRsFrame[i].x * 1000.0f;
                tempPoint.y = newRsFrame[i].y * 1000.0f;
                tempPoint.z = newRsFrame[i].z * 1000.0f;
                objCloud->points.push_back(tempPoint);
            }
        */
        switch(method)
        {
            case 0 :
            {
                cout << emptyTrayVec.size() << " - " << objVec.size() << endl;
                static double traysum = 0.0;
                std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(emptyTrayVec, outlierVector, 1280/2, 1280/2);
                cout << emptyTrayVec_f.size() << endl;
                for(int i = 0; i < emptyTrayVec_f.size(); i++)
                    traysum += sqrt(pow(emptyTrayVec_f[i].x ,2) + pow(emptyTrayVec_f[i].y ,2) + pow(emptyTrayVec_f[i].z ,2));
                cout << traysum << endl;

                std::vector<Algorithms::pts> objVec_f = algo.removeOutliers(objVec, outlierVector, 1280/2, 1280/2);
                double objsum = 0.0;
                cout << objVec_f.size() << endl;
                for(int j = 0; j < objVec_f.size(); j++)
                    objsum += sqrt( (pow(objVec_f[j].x ,2)) + (pow(objVec_f[j].y ,2)) + (pow(objVec_f[j].z ,2)) );


                cout << objsum << endl;

                cout << "Volume: " << abs(traysum-objsum) << endl;

                break;
            }
            case 1 : //OpenCv 2Dimage projection
            {
                OpenCV ocvGarment;
                ocvGarment.loadPlane(algo.removeOutliers(emptyTrayVec, outlierVector, 1280/2, 1280/2));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVector, 1280/2, 1280/2));
                ocvGarment.threshold('N', 7.5);
                ocvGarment.findBoundingBox(100, 750);
                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
                break;
            }
            case 2 : //OpenCv 2Dimage projection with roatated rectangle
            {
                OpenCV ocvGarment;
                ocvGarment.loadPlane(algo.removeOutliers(emptyTrayVec, outlierVector, 1280/2, 1280/2));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVector, 1280/2, 1280/2));
                ocvGarment.threshold('N', 10);
                ocvGarment.findRoatedBoundingBox(150, 750);
                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
                break;
            }
            case 3 : //PCL nearest neighbor trapzoidal
            {
                std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(emptyTrayVec, outlierVector, 1280/2, 1280/2);
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

                pclObj.insertCloud(emptyTrayVec_f);
                pclObj.findPlane();
                sumVolume = 0.0;

                PointCloud<PointXYZ>::Ptr obj_cloud_f (new PointCloud<PointXYZ>);
                obj_cloud_f = pclObj.removeOutliers(objCloud, outlierVector, 1280/2, 1280/2);

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
                        obj_cloud_f->erase(obj_cloud_f->begin()+i);
                    }
                }
                cout << "Volume: " << sumVolume << endl;
                break;
            }
            case 4 :
            {
                std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(emptyTrayVec, outlierVector, abs(ocvWS.xMin), abs(ocvWS.yMin));
                PclPlane pclObj;
                static double area = 0.0;
                static double volume = 0.0;
                static double sumVolume;

                pclObj.insertCloud(emptyTrayVec_f);
                pclObj.findPlane();
                sumVolume = 0.0;
                PointCloud<PointXYZ>::Ptr obj_cloud_f (new PointCloud<PointXYZ>);
                obj_cloud_f = pclObj.removeOutliers(objCloud, outlierVector, abs(ocvWS.xMin), abs(ocvWS.yMin));

                // Normal estimation*
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

                tree->setInputCloud (obj_cloud_f);
                n.setInputCloud (obj_cloud_f);
                n.setSearchMethod (tree);
                n.setKSearch (20);
                n.compute (*normals);
                //* normals should not contain the point normals + surface curvatures

                // Concatenate the XYZ and normal fields*
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields (*obj_cloud_f, *normals, *cloud_with_normals);
                //* cloud_with_normals = cloud + normals

                // Create search tree*
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
                tree2->setInputCloud (cloud_with_normals);

                // Initialize objects
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                pcl::PolygonMesh triangles;

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius (0.025);

                // Set typical values for the parameters
                gp3.setMu (2.5);
                gp3.setMaximumNearestNeighbors (50);
                gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                gp3.setMinimumAngle(M_PI/18); // 10 degrees
                gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
                gp3.setNormalConsistency(false);

                // Get result
                gp3.setInputCloud (cloud_with_normals);
                gp3.setSearchMethod (tree2);
                gp3.reconstruct (triangles);

            }
            default :
            {
                cout << "Invalid option chosen!" << endl;
                break;
            }
        }
    }
}

void getFrames()
{
    rsCam Stcam(RSx,RSy,fps);
    Stcam.startStream();
    while (true)
    {
        auto rsFrame = Stcam.RqSingleFrame();
        sq.push(rsFrame);
    }


}

int main (int argc, char * argv[]) try
{
    /*
    std::thread cam(getFrames);
    std::thread vol(volumeEstimate);

    cam.join();
    vol.join();
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr trayCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ tempPoint;

    PclPlane test;
    rsCam Stcam(RSx,RSy,fps);
    Stcam.startStream();
    auto rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
        trayCloud->points.push_back(tempPoint);
    }
    trayCloud->resize(RSy*RSx);
    pcl::io::savePCDFileASCII ("disttest0.pcd", *trayCloud);
    PclPlane test2(trayCloud);
    test.visualizeCloud(trayCloud);
    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
        objCloud->points.push_back(tempPoint);
    }
    objCloud->resize(RSy*RSx);
    pcl::io::savePCDFileASCII ("disttest1.pcd", *objCloud);
    test.visualizeCloud(objCloud);
    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
        objCloud2->points.push_back(tempPoint);
    }
    objCloud2->resize(RSy*RSx);
    pcl::io::savePCDFileASCII ("disttest2.pcd", *objCloud2);
    test.visualizeCloud(objCloud2);

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

/*

*/