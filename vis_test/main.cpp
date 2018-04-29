#include "PclPlane.h"
#include "rsCam.h"


#define conv_velocity 576.3f		// mm / ms
#define RSx 640
#define RSy 480
#define fps  6

struct node
{
	float value;
	node *next;
}
class list
{
public:
	list()
	{
		head = NULL;
		tail = NULL;		
	}		

private:
	node *head, *tail;
}

bool initialize(rsCam& cam)			// Saves pcd files for finding normal vectors
{
	PointCloud<PointXYZ>::Ptr init_cloud (new PointCloud<PointXYZ>);
	std::cout << "Press enter to take first picture" << std::endl;
	std::cin.get();
	PointXYZ tempPoint;
	auto rsFrame = cam.RqSingleFrame();
 	for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		init_cloud->push_back(tempPoint);
    }
	pcl::io::savePCDFileASCII ("movement1.pcd", *init_cloud);
	std::cout << "File saved as movement1.pcd" << std::endl;

	init_cloud->clear();
	std::cout << "Press enter to take second picture" << std::endl;
	std::cin.get();
	rsFrame = cam.RqSingleFrame();
 	for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		init_cloud->push_back(tempPoint);
    }
	pcl::io::savePCDFileASCII ("movement2.pcd", *init_cloud);
	std::cout << "File saved as movement2.pcd" << std::endl;
	return true;
}
void movingVolumeEstimation(PclPlane& plan, rsCam& cam)		// 
{
	PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);
	double lastframe = 0.0;
	float shift = 0.0f;
	for (int i = 0; i < 6; i++)
	{
		auto framedata = cam.RqFrameData();
		if ( i > 0)
			shift = conv_velocity/(framedata.timestamp-lastframe);
		plan.InputToMultiCloud(multi_cloud,framedata,shift);
		
		lastframe = framedata.timestamp;
	}
	std::cout << "size: " << multi_cloud->size() << std::endl;
}
double NumIntegration(PclPlane& plan,PointCloud<PointXYZ>::Ptr pc, int resX, int resY)
{
	Eigen::MatrixXf AccMat(resX,resY);
	float minX = 0.0f, maxX = 0.0f,minY = 0.0f, maxY = 0.0f;
	for (int i = 0 ; i < pc->size();i++)					// Find corners
	{
		if (pc->Points[i].x < minX) minX = pc->Points[i].x;
		else if  (pc->Points[i].x > maxX) maxX = pc->Points[i].x;

		if (pc->Points[i].y < minX) miny = pc->Points[i].x;
		else if  (pc->Points[i].y > maxY) maxY = pc->Points[i].y;
	}
	float stepX = (maxX-minX)/resX, stepY = (maxY-minY)/resY;
	
	for (int i = 0; i < pc->size(); i++)0
	{
		AccMat((pc->Points[i].x-minX)/stepX,(pc->Points[i].y-minY)/stepY) = pc-Points[i].z;			// hvordan offsetter jeg det så minX,minY er 0;
	}
	
}

int main (int argc, char * argv[]) try
{
    PointCloud<PointXYZ>::Ptr empty_tray_cloud (new PointCloud<PointXYZ>);

    PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);

    //PCDReader reader;
    //reader.read ("disttest0.pcd", *empty_tray_cloud_mm);
    //reader.read ("rs_box.pcd", *rs_box_cloud_mm);
    //reader.read ("odriod_box.pcd", *odroid_box_cloud);
  
	rsCam Stcam(RSx,RSy,fps);
	Stcam.startStream();
	auto rsFrame = Stcam.RqSingleFrame();
	PointXYZ tempPoint;
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		empty_tray_cloud->push_back(tempPoint);
    }
	 cout << "Empty Tray frame taken" << endl;
    PclPlane planetest(empty_tray_cloud);
	 cout << "Finding Plane" << endl;
    planetest.findPlane();

	empty_tray_cloud->clear();
	std::cout << "Press enter to continue" << std::endl;
	std::cin.get();
	
	movingVolumeEstimation(planetest,Stcam);
	
	/*cout << multi_cloud_f->size() << endl;
	multi_cloud_f->resize(multi_cloud_f->size());
	pcl::io::savePCDFileASCII ("multi_cloud.pcd", *multi_cloud_f);*/

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
