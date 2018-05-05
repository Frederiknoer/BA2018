#include "Algorithms.h"
#include "PclPlane.h"
#include "OpenCV.h"
#include "SafeQueue.h"
#include <thread>
#include <iostream>


using namespace std;

#define RSx 1280
#define RSy 720
#define fps 15

SafeQueue sq;

std::vector<float> initProgram()
{
    int camPlace = 1;
    cout << "Enter if camera is over inlet conveyor(1) or inside X-Ray(2) ..." << endl;
    cin >> camPlace;

    OpenCV ocvWS;

    //cout << "Taking picture of empty plane..." << endl;
    auto rsFrame = sq.dequeue();
    std::vector<Algorithms::pts> emptyTrayVec(rsFrame.size());
    for(int i = 0; i < rsFrame.size(); i++)
    {
        emptyTrayVec[i].x = rsFrame[i].x;
        emptyTrayVec[i].y = rsFrame[i].y;
        emptyTrayVec[i].z = rsFrame[i].z / 10.0f; //This is only for finding workspace!!!!!
    }
    ocvWS.create2dDepthImage(emptyTrayVec);

    if(camPlace == 1)
        ocvWS.threshold('N', 47.5);
    else if(camPlace == 2)
        ocvWS.threshold('N', 35.0); //Still needs adjustment

    ocvWS.findBoundingBox(500, 1500);
    std::vector<float> outlierVector = ocvWS.getBoundingBoxCorners();

    if(outlierVector.size() > 4)
        cout << "bad WS" << endl;

    outlierVector[0] = outlierVector[0] + 10; //min y
    outlierVector[1] = outlierVector[1] + 150; //min x
    outlierVector[2] = outlierVector[2] - 10; //max y
    outlierVector[3] = outlierVector[3] - 100; //max x
    //cout << "Y dist(WS): " << outlierVector[0] << "-" << outlierVector[2] << "   X dist(WS): " << outlierVector[1] << "-" << outlierVector[3] << endl;
    return outlierVector;
}


void volumeEstimate(std::vector<float> outlierVec)
{
    std::vector<Algorithms::pts> TrayVec(RSx*RSy);
    Algorithms algo;
    std::vector<Algorithms::pts> objVec(RSx*RSy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZ>);
    PointXYZ tempPoint;
    int method = 0;
    cout << endl << "Choose method of operation; PointCloud subtraction(0), OpenCV 2D image projection(1), "
            "OpenCV 2D image projection with minRect function(2), PCL triangulation(3), "
         "Accumulation Matrix(4)"<< endl;
    cin >> method;
    cout << endl;

    TrayVec = sq.dequeue();
    cout << endl << "Program is ready!! Press enter and start inserting garments.." << endl;
    cin.get();
    sq.clearQueue();

    while(true)
    {
        objVec = sq.dequeue();

        switch(method)
        {
            case 0 :
            {
                static double traysum = 0.0;
                std::vector<Algorithms::pts> TrayVec_f = algo.removeOutliers(TrayVec, outlierVec, 1280/2, 1280/2);

                for(int i = 0; i < TrayVec_f.size(); i++)
                    traysum += sqrt(pow(TrayVec_f[i].x ,2) + pow(TrayVec_f[i].y ,2) + pow(TrayVec_f[i].z ,2));


                std::vector<Algorithms::pts> objVec_f = algo.removeOutliers(objVec, outlierVec, 1280/2, 1280/2);
                double objsum = 0.0;

                for(int j = 0; j < objVec_f.size(); j++)
                    objsum += sqrt( (pow(objVec_f[j].x ,2)) + (pow(objVec_f[j].y ,2)) + (pow(objVec_f[j].z ,2)) );

                cout << "Volume: " << abs(traysum-objsum) << endl;

                break;
            }
            case 1 : //OpenCv 2Dimage projection
            {
                OpenCV ocvGarment;
                ocvGarment.loadPlane(algo.removeOutliers(TrayVec, outlierVec, 1280/2, 1280/2));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVec, 1280/2, 1280/2));
                ocvGarment.threshold('N', 10);
                ocvGarment.findBoundingBox(100, 950);
                ocvGarment.interPolate();

                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;

                break;
            }
            case 2 : //OpenCv 2Dimage projection with roatated rectangle
            {
                OpenCV ocvGarment;
                ocvGarment.loadPlane(algo.removeOutliers(TrayVec, outlierVec, 1280/2, 1280/2));

                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVec, 1280/2, 1280/2));
                ocvGarment.threshold('N', 10);
                ocvGarment.findRoatedBoundingBox(150, 950);

                cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;

                break;
            }
            case 3 :
            {
                objCloud->clear();
                for(int i = 0; i < objVec.size(); i ++)
                {
                    tempPoint.x = objVec[i].x;
                    tempPoint.y = objVec[i].y;
                    tempPoint.z = objVec[i].z;
                    objCloud->points.push_back(tempPoint);
                }
                cout << objCloud->size() << endl;

                cout << TrayVec.size() << endl;
                std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(TrayVec, outlierVec, 1280/2, 1280/2);
                PclPlane pclObj;
                cout << emptyTrayVec_f.size() << endl;
                pclObj.insertCloud(emptyTrayVec_f);
                pclObj.findPlane();

                static double area = 0.0;
                static double volume = 0.0;
                static double sumVolume;
                cout << "statics generated" << endl;


                sumVolume = 0.0;
                PointCloud<PointXYZ>::Ptr obj_cloud_f (new PointCloud<PointXYZ>);
                cout << objCloud->size() << endl;
                obj_cloud_f = pclObj.removeOutliers(objCloud, outlierVec, 1280/2, 1280/2);
                cout << obj_cloud_f->size() << endl;

                // Normal estimation*
                cout << "Starting normal estimation..." << endl;
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

                cout << "Setting up tree" << endl;
                tree->setInputCloud (obj_cloud_f);
                n.setInputCloud (obj_cloud_f);
                n.setSearchMethod (tree);
                n.setKSearch (150);
                n.compute (*normals);
                cout << "Normals computed" << endl;
                //* normals should not contain the point normals + surface curvatures

                // Concatenate the XYZ and normal fields*
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields (*obj_cloud_f, *normals, *cloud_with_normals);
                //* cloud_with_normals = cloud + normals

                cout << "Creating search tree" << endl;
                // Create search tree*
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
                tree2->setInputCloud (cloud_with_normals);

                // Initialize objects
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                pcl::PolygonMesh triangles;

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius (100);
                cout << "radius sat" << endl;
                // Set typical values for the parameters
                gp3.setMu (3.5);
                gp3.setMaximumNearestNeighbors (250);
                gp3.setMaximumSurfaceAngle(M_PI/3); // 45 degrees
                gp3.setMinimumAngle(M_PI/20); // 10 degrees
                gp3.setMaximumAngle(2*M_PI/2.5); // 120 degrees
                gp3.setNormalConsistency(false);

                cout << "gp3 done" << endl;

                // Get result
                gp3.setInputCloud (cloud_with_normals);
                gp3.setSearchMethod (tree2);
                gp3.reconstruct (triangles);

                //triangles.polygons.size();
                pcl::io::saveOBJFile("triangleMesh", triangles,5);
                cout << "Saved" << endl;

            }
            case 4 :
            {
                //PATRICK YOUR BASTARD! INSERT YOUR FUCNTION HERE
                break;
            }
            default :
            {
                cout << "Invalid option chosen!" << endl;
                break;
            }
        }
    }
}

void getFrames(std::vector<float> WS, float speed)
{
    rsCam cam(RSx,RSy,fps);
    std::vector<Algorithms::pts> tempVec(RSx*RSy);

    cam.startStream();
    frmdata rsFrame;

    double timeStamp = 0;
    double savedTimeStamp = 0;
    float dist = WS[3] - WS[1];

    float timeThresh = (dist/speed)*1000;

    while (true)
    {
        rsFrame = cam.RqFrameData();
        timeStamp = rsFrame.timestamp;
        if (timeStamp > savedTimeStamp + timeThresh)
        {
            savedTimeStamp = timeStamp;
            for(int i = 0; i < rsFrame.size; i++)
            {
                tempVec[i].x = rsFrame.vtx[i].x * 1000.0f;
                tempVec[i].y = rsFrame.vtx[i].y * 1000.0f;
                tempVec[i].z = rsFrame.vtx[i].z * 1000.0f;
            }
            sq.enqueue(tempVec);
        }
    }
}


int main (int argc, char * argv[]) try
{
    std::vector<float> workSpace;
    float convSpeed = 0; //576.3

    workSpace = initProgram();
    cout << "Enter conveyor speed" << endl;
    cin >> convSpeed; cout << endl;

    std::thread t1([&]()
    {
        (getFrames(workSpace, convSpeed));
    });
    std::thread t2([&]()
    {
        (volumeEstimate(workSpace));
    });

    t1.join();
    t2.join();

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
*/

/*
objCloud->clear();
                for(int i = 0; i < objVec.size(); i ++)
                {
                    tempPoint.x = objVec[i].x * 1000.0f;
                    tempPoint.y = objVec[i].y * 1000.0f;
                    tempPoint.z = objVec[i].z * 1000.0f;
                    objCloud->points.push_back(tempPoint);
                }

                std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(emptyTrayVec, outlierVector, 1280/2, 1280/2);
                PclPlane pclObj;
                pclObj.insertCloud(emptyTrayVec_f);
                pclObj.findPlane();

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
 */