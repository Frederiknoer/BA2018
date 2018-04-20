#include "Algorithms.h"
//#include "PclPlane.h"
#include "rsCam.h"
#include "OpenCV.h"

using namespace std;

int main (int argc, char * argv[]) try
{
    int RSx = 1280, RSy = 720, fps = 30;
    //Create objects
    Algorithms algo;
    OpenCV ocv;
    rsCam Stcam(RSx,RSy,fps);

    std::vector<Algorithms::pts> emptyTrayVec(RSx*RSy);
    std::vector<Algorithms::pts> objVec(RSx*RSy);

    cout << "Taking picture of empty plane" << endl;
    Stcam.startStream();
    auto rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 1000.0f;
    }
    ocv.loadPlane(algo.removeOutliers(emptyTrayVec));

    cout << "Press any key .. " << endl;
    cin.get();

    rsFrame = Stcam.RqSingleFrame();
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        objVec[i].x = rsFrame[i].x * 1000.0f;
        objVec[i].y = rsFrame[i].y * 1000.0f;
        objVec[i].z = rsFrame[i].z * 1000.0f;
    }

    ocv.create2dDepthImage(algo.removeOutliers(objVec));
    ocv.findBoundingBox(7.5);
    cout << "Volume:" << ocv.findVolumeWithinBoxes() << endl;

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