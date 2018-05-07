#include "Algorithms.h"
#include "PclPlane.h"
#include "OpenCV.h"
#include "SafeQueue.h"
#include <thread>
#include <iostream>


using namespace std;

#define conv_velocity 576.3f		// mm / ms
#define RSx 1280
#define RSy 720
#define fps 15

SafeQueue sq;

std::vector<float> initProgram(int camPlace)
{
    rsCam tempCam(RSx, RSy, fps);
    OpenCV ocvWS;
	if(!tempCam.startStream()) throw std::runtime_error("Couldnt start stream");;

    std::vector<float> outlierVector;

    //cout << "Taking picture of empty plane..." << endl;
    auto rsFrame = tempCam.RqSingleFrame();
    std::vector<Algorithms::pts> emptyTrayVec(RSx*RSy);
    for(int i = 0; i < RSx*RSy; i++)
    {
        emptyTrayVec[i].x = rsFrame[i].x * 1000.0f;
        emptyTrayVec[i].y = rsFrame[i].y * 1000.0f;
        emptyTrayVec[i].z = rsFrame[i].z * 100.0f; //This is only for finding workspace!!!!!
    }
    ocvWS.create2dDepthImage(emptyTrayVec);

    if(camPlace == 1)
    {
        //cout << camPlace << endl;
        ocvWS.threshold('N', 47.5);
        ocvWS.findBoundingBox(500, 1500);
        outlierVector = ocvWS.getBoundingBoxCorners();

        if(outlierVector.size() != 4)
            cout << "bad WS" << endl;

        outlierVector[0] = outlierVector[0] + 10; //min y
        outlierVector[1] = outlierVector[1] + 100; //min x
        outlierVector[2] = outlierVector[2] - 10; //max y
        outlierVector[3] = outlierVector[3] - 100; //max x
        //cout << "Y dist(WS): " << outlierVector[0] << "-" << outlierVector[2] << "   X dist(WS): " << outlierVector[1] << "-" << outlierVector[3] << endl;
        //cout << "Init done" << endl;
    }
    else if(camPlace == 2)
    {
        //cout << camPlace << endl;
        ocvWS.threshold('N', 40.5); //Still needs adjustment
        ocvWS.findBoundingBox(500, 1500);
        outlierVector = ocvWS.getBoundingBoxCorners();

        if(outlierVector.size() != 4)
            cout << "bad WS" << endl;

        outlierVector[0] = outlierVector[0]; //min y
        outlierVector[1] = outlierVector[1] + 65; //min x
        outlierVector[2] = outlierVector[2]; //max y
        outlierVector[3] = outlierVector[3] - 20; //max x
        //cout << "Y dist(WS): " << outlierVector[0] << "-" << outlierVector[2] << "   X dist(WS): " << outlierVector[1] << "-" << outlierVector[3] << endl;
        //cout << "Init done" << endl;
    }

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
	
    auto emptyTrayVec_f = sq.dequeue().vtx;

	OpenCV ocvGarment;
	PclPlane pclObj;
	PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);
// Setup for movingVolumeEstimation
	double traysum;
	long int ite;
	double firstframe;
	//std::vector<Algorithms::pts> emptyTrayVec_f = algo.removeOutliers(TrayVec, outlierVec, 1280/2, 1280/2);


	switch(method)
        {
            case 0 :{
					traysum = 0.0;
                	for(int i = 0; i < emptyTrayVec_f.size(); i++)
                    	traysum += sqrt(pow(emptyTrayVec_f[i].x ,2) + pow(emptyTrayVec_f[i].y ,2) + pow(emptyTrayVec_f[i].z ,2));
					break;}
			case 1 :{
					ocvGarment.loadPlane(emptyTrayVec_f);
					break;}
			case 2 :{
					ocvGarment.loadPlane(emptyTrayVec_f);
					break;}
			case 3 :{
					pclObj.insertCloud(emptyTrayVec_f);
					pclObj.findPlane();
					break;}
			case 4 :{
					long int ite = 0;
					double firstframe = 0.0f;
					pclObj.insertCloud(emptyTrayVec_f);
					pclObj.findPlane();
					break;}
			default:{
				throw std::runtime_error("Invalid input");
					break;}
		}
    cout << endl << "Program is ready!! Press enter and start inserting garments.." << endl;
    cin.get();
    cin.get();
    sq.clearQueue();
	
    cout << "Program is running!!" << endl;

    while(true)
    {
		auto fd = sq.dequeue();
        objVec = fd.vtx;

        switch(method)
        {
            case 0 :
            {

                //std::vector<Algorithms::pts> objVec_f = algo.removeOutliers(objVec, outlierVec, 1280/2, 1280/2);
                double objsum = 0.0;

                for(int j = 0; j < objVec.size(); j++)
                    objsum += sqrt( (pow(objVec[j].x ,2)) + (pow(objVec[j].y ,2)) + (pow(objVec[j].z ,2)) );

                cout << "Volume: " << abs(traysum-objsum) << endl;

                break;
            }
            case 1 : //OpenCv 2Dimage projection
            {
                ocvGarment.create2dDepthImageFromPlane(objVec);
                ocvGarment.threshold('N', 10);
                ocvGarment.findBoundingBox(150, 1000);
                if(!(ocvGarment.getBoundingBoxCorners().empty()))
                {
                    ocvGarment.interPolate();
                    cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
                }


                break;
            }
            case 2 : //OpenCv 2Dimage projection with roatated rectangle
            {
                ocvGarment.create2dDepthImageFromPlane(algo.removeOutliers(objVec, outlierVec, 1280/2, 1280/2));
                ocvGarment.threshold('N', 10);
                ocvGarment.findRoatedBoundingBox(150, 950);
                if(!(ocvGarment.getBoundingBoxCorners().empty()))
                {
                    //ocvGarment.interPolate();
                    cout << "Volume: " << ocvGarment.findVolumeWithinBoxes() << endl;
                }

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
                //n.setKSearch (150);
                n.setRadiusSearch(10);
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
                gp3.setMaximumNearestNeighbors (125);
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
				auto framedata = sq.dequeue();
				if ( ite%3 == 0 )
				{
					firstframe = framedata.timestamp;
				}
				float shift = (conv_velocity*(framedata.timestamp-firstframe))/1000.0f;
				pclObj.InputToMultiCloud(multi_cloud,framedata,shift,outlierVec);
				
				if (ite%3 == 2)
				{
					std::cout << multi_cloud->size() << std::endl;
					float sum1 =  pclObj.NumIntegration(multi_cloud,500,500,outlierVec);
					std::cout << std::fixed <<  "sum " <<500 << "x" <<500 << " : " << sum1 << std::endl;
					multi_cloud->clear();
				}   
				ite++;
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

void getFrames(std::vector<float> WS, float speed, int camPlace)
{
    rsCam cam(RSx,RSy,fps);
    std::vector<Algorithms::pts> tempVec(RSx*RSy);

    cam.startStream();
    frmdata rsFrame;


    double timeStamp = 0;
    double savedTimeStamp = 0;
    float dist = 0;
    if(camPlace == 1)
        dist = WS[3] - WS[1];
    else if(camPlace == 2)
        dist = WS[2] - WS[0];

    float timeThresh = (dist/speed)*1000;

    while (true)
    {
        rsFrame = cam.RqFrameData(WS);
        timeStamp = rsFrame.timestamp;
        if (timeStamp > savedTimeStamp + timeThresh)
        {
            savedTimeStamp = timeStamp;
            /*for(int i = 0; i < rsFrame.size; i++)
            {
                tempVec[i].x = rsFrame.vtx[i].x * 1000.0f;
                tempVec[i].y = rsFrame.vtx[i].y * 1000.0f;
                tempVec[i].z = rsFrame.vtx[i].z * 1000.0f;
            }*/
			std::cout << "nu" <<  std::endl;
            sq.enqueue(rsFrame);
        }
    }
}
/*void getFrames()
{
    rsCam cam(RSx,RSy,fps);
    cam.startStream();
    frmdata rsFrame;
    while (true)
    {
        rsFrame = cam.RqFrameData();
		std::cout << "nu" <<  std::endl;
        sq.enqueue(rsFrame);
    }
}*/

////////////////////
/*void measureVelocity(rsCam& cam, PclPlane & plan)
{
	PointCloud<PointXYZ>::Ptr vel_cloud (new PointCloud<PointXYZ>);
	frmdata arr[4];
	for (int i = 0;i<4;i++)
	{
		arr[i] = cam.RqFrameData();
	}
	double temp = 0.0;
	for (int i = 0; i < 4;i++)
	{
		for(int j = 0; j < (arr[i].size); j ++)
	 	{
			if (plan.getDistToPlane(arr[i].vtx[j].x*1000.0f,arr[i].vtx[j].y*1000.0f,arr[i].vtx[j].z*1000.0f) > 7.5f)
			{
				vel_cloud->push_back(PointXYZ(arr[i].vtx[j].x*1000.0f,arr[i].vtx[j].y*1000.0f,arr[i].vtx[j].z*1000.0f));
			}
    	}
	pcl::io::savePCDFileASCII ("velocity_" + std::to_string(arr[i].timestamp - temp), *vel_cloud);
	temp = arr[i].timestamp;
	}
}
bool initialize(rsCam& cam, PclPlane& plan)			// Saves pcd files for finding normal vectors
{
	PointCloud<PointXYZ>::Ptr init_cloud (new PointCloud<PointXYZ>);
	std::cout << "Press enter to take first picture" << std::endl;
	std::cin.get();
	PointXYZ tempPoint;
	auto rsFrame = cam.RqSingleFrame();
 	for(int i = 0; i < (RSx*RSy); i ++)
    {
		if (plan.getDistToPlane(rsFrame[i].x*1000.0f,rsFrame[i].y*1000.0f,rsFrame[i].z*1000.0f) > 7.5f)
		{
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		init_cloud->push_back(tempPoint);
		}
    }
	pcl::io::savePCDFileASCII ("movement1.pcd", *init_cloud);
	std::cout << "File saved as movement1.pcd" << std::endl;

	init_cloud->clear();
	std::cout << "Press enter to take second picture" << std::endl;
	std::cin.get();
	rsFrame = cam.RqSingleFrame();
 	for(int i = 0; i < (RSx*RSy); i ++)
    {
		if (plan.getDistToPlane(rsFrame[i].x*1000.0f,rsFrame[i].y*1000.0f,rsFrame[i].z*1000.0f) > 7.5f)
		{
        tempPoint.x = rsFrame[i].x * 1000.0f;
        tempPoint.y = rsFrame[i].y * 1000.0f;
        tempPoint.z = rsFrame[i].z * 1000.0f;
		init_cloud->push_back(tempPoint);
		}
    }
	pcl::io::savePCDFileASCII ("movement2.pcd", *init_cloud);
	std::cout << "File saved as movement2.pcd" << std::endl;
	return true;
}
void visualizingIntegration(PclPlane& plan,PointCloud<PointXYZ>::Ptr pc, int resX, int resY)
{
	PointCloud<PointXYZ>::Ptr visualizePC (new PointCloud<PointXYZ>);
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

	for (int i = 0; i < pc->size(); i++)
	{
		//std::cout << "x " << (int)((pc->points[i].x-minX)/stepX) << std::endl;
		//std::cout << "y " << (int)((pc->points[i].y-minY)/stepY) << std::endl;
//		AccMat[(int)((pc->points[i].x-minX)/stepX)][(int)((pc->points[i].y-minY)/stepY)].insertSort(plan.getDistToPlane(pc->points[i].x,pc->points[i].y,pc->points[i].z));			
		AccMat[(int)((pc->points[i].x-minX)/stepX)][(int)((pc->points[i].y-minY)/stepY)].insertSort(pc->points[i].z);		
	}
	std::cout << "her" << endl;
	for (int i = 0; i <= resX;i++)
		for(int j = 0; j <= resY;j++)
		{
			/*if (AccMat[i][j].isEmpty() && i > 0 && j > 0 && i && i < resX+1 && j < resY+1)
			{
				AccMat[i][j].append(AccMat[i-1][j].average());
				AccMat[i][j].append(AccMat[i][j-1].average());
				AccMat[i][j].append(AccMat[i+1][j].average());
				AccMat[i][j].append(AccMat[i][j+1].average());
			}
			if (AccMat[i][j].isEmpty() && i > 0 && j > 0 && i && i < resX && j < resY)
			{
				AccMat[i][j].insertSort(AccMat[i-1][j].median());
				AccMat[i][j].insertSort(AccMat[i][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j].median());
				AccMat[i][j].insertSort(AccMat[i][j+1].median());

				AccMat[i][j].insertSort(AccMat[i-1][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j+1].median());
				AccMat[i][j].insertSort(AccMat[i-1][j+1].median());
			}
			visualizePC->push_back(PointXYZ(float(i)*stepX,float(j)*stepY,AccMat[i][j].median()));
		} 
	pcl::io::savePCDFileASCII ("visualizeIntegration.pcd", *visualizePC);
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
	//std::cout << std::fixed << "stepX: " << stepX << " , stepY: " << stepY << std::endl; 
	//std::cout << minX << " - " << maxX << " : " << minY << " - " << maxY << std::endl;
	for (int i = 0; i < pc->size(); i++)
	{
		//std::cout << "x " << (int)((pc->points[i].x-minX)/stepX) << std::endl;
		//std::cout << "y " << (int)((pc->points[i].y-minY)/stepY) << std::endl;
//		AccMat[(int)((pc->points[i].x-minX)/stepX)][(int)((pc->points[i].y-minY)/stepY)].insertSort(plan.getDistToPlane(pc->points[i].x,pc->points[i].y,pc->points[i].z));			
		AccMat[(int)((pc->points[i].x-minX)/stepX)][(int)((pc->points[i].y-minY)/stepY)].insertSort(pc->points[i].z);		
	}
	std::cout << "her" << endl;
	for (int i = 0; i <= resX;i++)
		for(int j = 0; j <= resY;j++)
		{
			/*if (AccMat[i][j].isEmpty() && i > 0 && j > 0 && i && i < resX && j < resY)			// Average filtering
			{
				AccMat[i][j].append(AccMat[i-1][j].average());
				AccMat[i][j].append(AccMat[i][j-1].average());
				AccMat[i][j].append(AccMat[i+1][j].average());
				AccMat[i][j].append(AccMat[i][j+1].average());

				AccMat[i][j].append(AccMat[i-1][j-1].average());
				AccMat[i][j].append(AccMat[i+1][j-1].average());
				AccMat[i][j].append(AccMat[i+1][j+1].average());
				AccMat[i][j].append(AccMat[i-1][j+1].average());
			}
			if (AccMat[i][j].isEmpty() && i > 0 && j > 0 && i && i < resX && j < resY)				// Median filtering
			{
				AccMat[i][j].insertSort(AccMat[i-1][j].median());
				AccMat[i][j].insertSort(AccMat[i][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j].median());
				AccMat[i][j].insertSort(AccMat[i][j+1].median());

				AccMat[i][j].insertSort(AccMat[i-1][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j-1].median());
				AccMat[i][j].insertSort(AccMat[i+1][j+1].median());
				AccMat[i][j].insertSort(AccMat[i-1][j+1].median());
			}
			sum += AccMat[i][j].median()*stepX*stepY;
			//cout << "sum " << sum << "\n";
		} 
	return sum;
}
void movingVolumeEstimation(PclPlane& plan, rsCam& cam)		// 
{
	PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);
	double firstframe = 0.0;
	//double lastframe = 0.0;
	firstframe = framedata.timestamp;
	float shift = 0.0f;
	while (true)
	{
		auto framedata = sq.dequeue();
		shift = (conv_velocity*(framedata.timestamp-firstframe))/1000.0f;
		plan.InputToMultiCloud(multi_cloud,framedata,shift);
		

		//lastframe = framedata.timestamp;
	}
	pcl::io::savePCDFileASCII ("multi_cloud", *multi_cloud);
	for(int i = 0; i < 10; i++)
	{
		float sum1 =  NumIntegration(plan,multi_cloud,250+50*i,250+50*i);
		std::cout << std::fixed <<  "sum " <<250+50*i << "x" <<250+50*i << " : " << sum1 << std::endl;
	}
	//visualizingIntegration(plan,multi_cloud,500,500);
}
*/
int main (int argc, char * argv[]) try
{
    std::vector<float> workSpace;
    float convSpeed = 0; //576.3
    int camPlace = 1;
    cout << "Enter if camera is over inlet conveyor(1) or inside X-Ray(2):" << endl;
    cin >> camPlace;
    cout << "Starting Initialization.." << endl;
    workSpace = initProgram(camPlace);
    cout << "Enter conveyor speed (mm/s) :" << endl;
    cin >> convSpeed; cout << endl;

    std::thread t1([&]()
    {

        (getFrames(workSpace, convSpeed, camPlace));
    });
    std::thread t2([&]()
    {
        (volumeEstimate(workSpace));
		
    });

    t1.join();
    t2.join();



/*int main (int argc, char * argv[]) try
{
    PointCloud<PointXYZ>::Ptr empty_tray_cloud (new PointCloud<PointXYZ>);

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
    planetest.findPlane();

	empty_tray_cloud->clear();
	std::cout << "Press enter to continue" << std::endl;
	std::cin.get();

	measureVelocity(Stcam,planetest);
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
