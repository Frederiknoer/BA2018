#include "Algorithms.h"
//#include "PclPlane.h"
#include "rsCam.h"
#include "OpenCV.h"
#include <iostream>

using namespace std;

int main (int argc, char * argv[]) try
{
    int RSx = 1280, RSy = 720, fps = 30;

    rsCam Stcam(RSx,RSy,fps);

    int camPlace;
    cout << "Enter if camera is over inlet conveyor(1) or inside X-Ray(2) ..." << endl;
    cin >> camPlace;

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