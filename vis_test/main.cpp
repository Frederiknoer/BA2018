#include "PclPlane.h"
#include "rsCam.h"
#include "Algorithms.h"


#define conv_velocity  0.0f 	//576.3f		// mm / ms
#define RSx 640
#define RSy 480
#define fps  6


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
float NumIntegration(PclPlane& plan,PointCloud<PointXYZ>::Ptr pc, int resX, int resY)
{
	llist AccMat[resX+1][resY+1] = {};
	float sum = 0.0f;
	float minX = 0.0f, maxX = 0.0f,minY = 0.0f, maxY = 0.0f;
	for (int i = 0 ; i < pc->size();i++)					// Find corners
	{
		if (pc->points[i].x < minX) minX = pc->points[i].x;
		else if  (pc->points[i].x > maxX) maxX = pc->points[i].x;

		if (pc->points[i].y < minY) minY = pc->points[i].y;
		else if  (pc->points[i].y > maxY) maxY = pc->points[i].y;
	}
	float stepX = (maxX-minX)/(float)resX, stepY = (maxY-minY)/(float)resY;
	std::cout << minX << " - " << maxX << " : " << minY << " - " << maxY << std::endl;
	for (int i = 0; i < pc->size(); i++)
	{
		//std::cout << "x " << (int)((pc->points[i].x-minX)/stepX) << std::endl;
		//std::cout << "y " << (int)((pc->points[i].y-minY)/stepY) << std::endl;
	
		AccMat[(int)((pc->points[i].x-minX)/stepX)][(int)((pc->points[i].y-minY)/stepY)].append(plan.getDistToPlane(pc->points[i].x,pc->points[i].y,pc->points[i].z));			
	}
	std::cout << "dav " << std::endl;
	for (int i = 0; i < resX;i++)
		for(int j = 0; j < resY;j++)
		{
			sum += AccMat[i][j].average()*stepX*stepY;
			//cout << "sum " << sum << "\n";
		} 
	return sum;
}
void movingVolumeEstimation(PclPlane& plan, rsCam& cam)		// 
{
	PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);
	double firstframe = 0.0;
	//double lastframe = 0.0;
	float shift = 0.0f;
	for (int i = 0; i < 2; i++)
	{
		auto framedata = cam.RqFrameData();
		if(i == 0)
		{
			firstframe = framedata.timestamp;
			shift = 0.0f;
		}
		else
			shift = conv_velocity/(framedata.timestamp-firstframe);
		plan.InputToMultiCloud(multi_cloud,framedata,shift);
		
		//lastframe = framedata.timestamp;
	}
	pcl::io::savePCDFileASCII ("multi_cloud.pcd", *multi_cloud);
	float sum1 =  NumIntegration(plan,multi_cloud,500,500);
	std::cout << std::fixed <<  "sum: " << sum1 << std::endl;
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
	pcl::io::savePCDFileASCII ("Empty_tray", *empty_tray_cloud);
    PclPlane planetest(empty_tray_cloud);
	 cout << "Finding Plane" << endl;
    //planetest.findPlane();

	empty_tray_cloud->clear();
	std::cout << "Press enter to continue" << std::endl;
	std::cin.get();
	for (int i = 0; i < 6;i++)
	{
	auto rsFrame = Stcam.RqSingleFrame();
	PointXYZ tempPoint;
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		empty_tray_cloud->push_back(tempPoint);
    }
	pcl::io::savePCDFileASCII (std::to_string(i), *empty_tray_cloud);
	empty_tray_cloud->clear();
}
	//initialize(Stcam);
	
	//movingVolumeEstimation(planetest,Stcam);
	
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
