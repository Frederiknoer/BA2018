#include "PclPlane.h"
#include "rsCam.h"



int main (int argc, char * argv[]) try
{
    int RSx = 640, RSy = 480, fps = 6;

    //PointCloud<PointXYZ>::Ptr empty_tray_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr empty_tray_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr empty_tray_cloud_f_mm (new PointCloud<PointXYZ>);

    //PointCloud<PointXYZ>::Ptr rs_box_cloud (new PointCloud<PointXYZ>);
    
    //PointCloud<PointXYZ>::Ptr odroid_box_cloud (new PointCloud<PointXYZ>);
    //PointCloud<PointXYZ>::Ptr odroid_box_cloud_f (new PointCloud<PointXYZ>);

    PointCloud<PointXYZ>::Ptr multi_cloud (new PointCloud<PointXYZ>);

    PointCloud<PointXYZ>::Ptr multi_cloud_f (new PointCloud<PointXYZ>);
    PointCloud<PointXYZRGB>::Ptr common_cloud (new PointCloud<PointXYZRGB>);


    //PCDReader reader;
    //reader.read ("disttest0.pcd", *empty_tray_cloud_mm);
    //reader.read ("rs_box.pcd", *rs_box_cloud_mm);
    //reader.read ("odriod_box.pcd", *odroid_box_cloud);
  
	rsCam Stcam(RSx,RSy,fps);
	Stcam.startStream();
	auto rsFrame1 = Stcam.RqSingleFrame();
cout << "hej" << endl;
PointXYZ tempPoint;
    for(int i = 0; i < (RSx*RSy); i ++)
    {
        tempPoint.x = rsFrame1[i].x * 1000.0f;
        tempPoint.y = rsFrame1[i].y * 1000.0f;
        tempPoint.z = rsFrame1[i].z * 1000.0f;
		empty_tray_cloud->push_back(tempPoint);
    }
	 cout << "Empty Tray frame taken" << endl;
    PclPlane planetest(empty_tray_cloud);
	 cout << "Finding Plane" << endl;
    planetest.findPlane();
	
	std::cout << "Press enter to continue" << std::endl;

	std::cin.get();
	float vel = 576.3f/fps;
	//PointXYZ tempPoint;
cout << planetest.nX[0] << " - " << planetest.nX[1] <<" - " <<planetest.nX[2] << endl;
	for (int i = 0; i < 6;i++)
	{	
		cout << "Frame: " << i << endl;
		auto rsFrame = Stcam.RqSingleFrame();
		for(int j = 0; j < (RSx*RSy); j++)
		{
		//cout << rsFrame[j].x*1000.0f*(planetest.nX[0]+planetest.nX[1]+planetest.nX[2]) << endl;
			if (planetest.getDistToPlane(rsFrame[j].x,rsFrame[j].y,rsFrame[j].z) > 10.0)
				//multi_cloud->push_back(PointXYZ(rsFrame[j].x*1000.0*(planetest.nX[0]+planetest.nX[1]+planetest.nX[2])+vel,rsFrame[j].y*1000.0*(planetest.nY[0]+planetest.nY[1]+planetest.nY[2]),rsFrame[j].z*1000.0*(planetest.nZ[0]+planetest.nZ[1]+planetest.nZ[2])));	//Kan være z skal være distance to plane i stedet.
				multi_cloud->push_back(PointXYZ(rsFrame[j].x*1000.0*(planetest.nX[0]+planetest.nX[1]+planetest.nX[2])+vel,rsFrame[j].y*1000.0*(planetest.nY[0]+planetest.nY[1]+planetest.nY[2]),planetest.getDistToPlane(rsFrame[j].x*1000,rsFrame[j].y*1000,rsFrame[j].z*1000)));
		}
	}
	std::vector<float> corners = {-200.0f,-200.0f,400.0f,150.0f};
cout << multi_cloud->size() << endl;
	multi_cloud_f = planetest.removeOutliers(multi_cloud,corners,0.0f,0.0f );
	//planetest.visualizeCloud(dense_cloud)
	/*PclPlane comPlane;
    common_cloud = comPlane.mergeCloudsColor(multi_cloud,'r', empty_tray_cloud,'b');
	comPlane.visualizeColorCloud(common_cloud);*/
	cout << multi_cloud_f->size() << endl;
	multi_cloud_f->resize(multi_cloud_f->size());
	pcl::io::savePCDFileASCII ("multi_cloud.pcd", *multi_cloud_f);
	/*cout << "AAAAAAAAAAAARRRRRRRH" << endl;
for (int i = 0; i < 30 ; i++){
	cout << 1337*i << endl;
	cout << " X: " << planetest.plane_cloud->points[1337*i].x << " y: " << planetest.plane_cloud->points[1337*i].y << " Z: " << planetest.plane_cloud->points[1337*i].z << endl;
	cout << "Dist to plane" << planetest.getDistToPlane(planetest.plane_cloud->points[1337*i].x,planetest.plane_cloud->points[1337*i].y,planetest.plane_cloud->points[1337*i].z) << endl;
	}
    empty_tray_cloud_f_mm = planetest.removeOutliers(empty_tray_cloud_mm, 'm');
    rs_box_cloud_f_mm = planetest.removeOutliers(rs_box_cloud_mm, 'm');
    cout << "Ouliers Gone!" << endl;*/
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

   /* planetest.mergeSort(rs_box_cloud_f_mm, rs_box_cloud_f_mm->size());
    float xMin = planetest.sorted_x->points.front().x;
    float xMax = planetest.sorted_x->points.back().x;
    float yMin = planetest.sorted_y->points.front().y;
    float yMax = planetest.sorted_y->points.back().y;
    float yDist = abs(yMin - yMax);
    float xDist = abs(xMin - xMax);
    cout << "Number of points in plane cloud:  " << planetest.plane_cloud->points.size() << endl;
    cout <<  "Cloud Height: " << xDist << endl;
    cout <<  "Cloud Width:  " << yDist << endl;


    int imgY = ((abs(yMin))+yMax)+1;
    int imgX = ((abs(xMin))+xMax)+1;
    float imgCloud[imgX][imgY] = {0.0f};

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

        if(dist > 5)
            imgCloud[row][col] = dist;
    }

    int blackSpots = 0;
    int minBarrierX = imgX, maxBarrierX = 0, minBarrierY = imgY, maxBarrierY = 0;
    for(int x = 0; x < imgX; x++)
        for(int y = 0; y < imgY; y++)
        {
            if(imgCloud[x][y] == 0)
                blackSpots++;
            if(imgCloud[x][y] > 0)
            {
                if(x > maxBarrierX)
                    maxBarrierX = x;
                else if(x < minBarrierX)
                    minBarrierX = x;

                if(y > maxBarrierY)
                    maxBarrierY = y;
                else if(y < minBarrierY)
                    minBarrierY = y;
            }
        }
    cout << "Black Spots before resizing: " << blackSpots << endl;
    cout << "Min x: " << minBarrierX << "  Max x: " << maxBarrierX << endl;
    cout << "Min y: " << minBarrierX << "  Max y: " << maxBarrierX << endl;

    double volume = 0;
    blackSpots = 0;
    for(int x = minBarrierX; x < maxBarrierX; x++)
        for(int y = minBarrierY; y < maxBarrierY; y++)
        {
            if(imgCloud[x][y] == 0)
                blackSpots++;
            volume += imgCloud[x][y];
        }

    cout << "Black spots after resizing: " << blackSpots << endl;
    cout << "Volume: " << volume << endl;
    //planetest.visualizeCloud(empty_tray_cloud_f_mm);
    //planetest.visualizeColorCloud(planetest.mergeCloudsColor(empty_tray_cloud_mm, 'r', empty_tray_cloud_f_mm, 'g'));

    //common_cloud->clear();
    //PclPlane comPlane;
    //common_cloud = comPlane.mergeCloudsColor(planetest.plane_cloud,'g', org_empty_tray_cloud,'b');
    //comPlane.visualizeColorCloud(common_cloud);
    //comPlane.visualizeCloud(planetest.plane_cloud);
	planetest.CalculateTrajectory(0.0,0.0,0.0);
	planetest.CalculateTrajectory( 500.0,134.3,34.5);

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
