
//
// Created by fred on 3/12/18.
//

#include "PclPlane.h"


PclPlane::PclPlane(){}

PclPlane::PclPlane(PointCloud<PointXYZ>::Ptr in_cloud)
{
    //input_cloud->clear();
    input_cloud = in_cloud;
    cout << "Cloud Loaded to Class ..." << endl;
}

void PclPlane::insertCloud(PointCloud<PointXYZ>::Ptr in_cloud)
{
    input_cloud = in_cloud;
    cout << "Cloud Loaded to Class ..." << endl;
}

void PclPlane::findPlane()
{
    PointCloud<PointXYZ>::Ptr
            cloud_filtered (new PointCloud<PointXYZ>),
            cloud_p (new PointCloud<PointXYZ>),
            cloud_f (new PointCloud<PointXYZ>);
    cloud_filtered->clear(); cloud_f->clear(); cloud_p->clear();
    cloud_filtered = input_cloud;


    cout << "Starting Plane Estimation via RANSAC ..." << endl;

    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());
    // Create the segmentation object
    SACSegmentation<PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (7.5); //THIS IS IN MILLIMETER IF THE PC IS IN METERS CHANGE !!!!!!!!!!!!!

    // Create the filtering object
    ExtractIndices<PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        //cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl;

        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    cout << "RANSAC Done!" << endl;
    //plane_cloud->clear();
    plane_cloud = cloud_p;

    auto dataSize = (int)plane_cloud->size();

    Eigen::MatrixXf mat(dataSize,3);
    Eigen::VectorXf vec(dataSize);
    for (int i = 0; i < dataSize; i++)
    {
        mat(i, 0) = plane_cloud->points[i].x;
        mat(i, 1) = plane_cloud->points[i].y;
        mat(i, 2) = 1.0f;
        vec(i) = -plane_cloud->points[i].z;
    }
    //cout <<  mat.colPivHouseholderQr().solve(vec) << endl;
    //auto eig = mat.colPivHouseholderQr().solve(vec).eval();
    auto eig = mat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vec).eval();
    cout << endl << "Plane Coefficients: " << endl;
    cout << "Coeff A: " << eig(0) << endl
         << "Coeff B: " << eig(1) << endl
         << "Coeff C: " << 1 << endl
         << "Coeff D: " << eig(2) << endl;
    coeffA = eig(0);
    coeffB = eig(1);
    coeffC = 1.0f;
    coeffD = eig(2);

    float residuals = 0.0f;
    for (int i = 0; i < plane_cloud->points.size(); i ++)
    {
        residuals += pow(getDistToPlane(plane_cloud->points[i].x, plane_cloud->points[i].y, plane_cloud->points[i].z),2);
    }
    residuals = sqrt(residuals);
    residuals = residuals/plane_cloud->size();
    cout << "Average resiudals:  " << residuals << endl;
    PointCloud<PointXYZ>::Ptr c_cloud (new PointCloud<PointXYZ>);
    PointXYZ coeffPoint;
    for(int i = 0; i < plane_cloud->points.size(); i++)
    {
        coeffPoint.x = plane_cloud->points[i].x;
        coeffPoint.y = plane_cloud->points[i].y;
        coeffPoint.z = coeffA*coeffPoint.x + coeffB*coeffPoint.y + coeffD;
        c_cloud->push_back(coeffPoint);
    }
    coeff_cloud = c_cloud;
}

float PclPlane::getDistToPlane(float x, float y, float z)
{
    float distNum = coeffA*x + coeffB*y + coeffC*z + coeffD;
    float sumOfSquares = (coeffA*coeffA)+(coeffB*coeffB)+(coeffC*coeffC);
    float distDenum = sqrt(sumOfSquares);

    if((distNum/distDenum) > 0.0)
        return 0.0f;
    cout << "Distance to Plane = " << abs(distNum/distDenum) << endl;
    return abs(distNum/distDenum);
}

void PclPlane::visualizeColorCloud(PointCloud<PointXYZRGB>::Ptr visCloud)
{
    visualization::CloudViewer viewer ("Input Cloud");
    viewer.showCloud (visCloud);
    while (!viewer.wasStopped());
}

void PclPlane::visualizeCloud(PointCloud<PointXYZ>::Ptr visCloud)
{
    visualization::CloudViewer viewer ("Input Cloud");
    viewer.showCloud (visCloud);
    while (!viewer.wasStopped());
}

void PclPlane::visualizePlaneCloud()
{
    visualization::CloudViewer viewer ("Plane Cloud");
    viewer.showCloud (plane_cloud);
    while (!viewer.wasStopped());
}
PointCloud<PointXYZRGB>::Ptr PclPlane::mergeCloudsColor(PointCloud<PointXYZ>::Ptr cloud1, char color1, PointCloud<PointXYZ>::Ptr cloud2, char color2)
{
    PointCloud<PointXYZRGB>::Ptr mergedCloud (new PointCloud<PointXYZRGB>);
    PointXYZRGB tempPoint;

    for(int i = 0; i < cloud1->points.size(); i++)
    {
        tempPoint.x = cloud1->points[i].x;
        tempPoint.y = cloud1->points[i].y;
        tempPoint.z = cloud1->points[i].z;
        if(color1 == 'r')
            tempPoint.r = 255;
        if(color1 == 'g')
            tempPoint.g = 255;
        if(color1 == 'b')
            tempPoint.b = 255;

        mergedCloud->points.push_back(tempPoint);
        tempPoint.r = 0;
        tempPoint.g = 0;
        tempPoint.b = 0;
    }

    for(int i = 0; i < cloud2->points.size(); i++)
    {
        tempPoint.x = cloud2->points[i].x;
        tempPoint.y = cloud2->points[i].y;
        tempPoint.z = cloud2->points[i].z;
        if(color2 == 'r')
            tempPoint.r = 255;
        if(color2 == 'g')
            tempPoint.g = 255;
        if(color2 == 'b')
            tempPoint.b = 255;

        mergedCloud->points.push_back(tempPoint);
        tempPoint.r = 0;
        tempPoint.g = 0;
        tempPoint.b = 0;
    }

    return mergedCloud;

}

PointCloud<PointXYZ>::Ptr PclPlane::removeOutliers(PointCloud<PointXYZ>::Ptr outlier_cloud, char in)
{
    PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>);

    cloud_filtered->clear();
    if (in == 's')
    {
        cout << "Statistical removal" << endl;
        StatisticalOutlierRemoval<PointXYZ> sor;
        sor.setInputCloud (outlier_cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);
    }
    else if (in == 'c')
    {
        cout << "Conditional removal" << endl;
        // build the condition
        ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>());

        range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("x", ComparisonOps::GT, -0.50)));
        range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("x", ComparisonOps::LT, 0.38)));
        range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("y", ComparisonOps::GT, -0.175))); //good
        range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("y", ComparisonOps::LT, 0.215))); //21
        //range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("z", ComparisonOps::LT, zLimit)));

        // build the filter
        ConditionalRemoval<PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(outlier_cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter(*cloud_filtered);
    }
    else if (in == 'm')
    {
        int midPoint = (int)((plane_cloud->points.size())/2);
        float zLimit = plane_cloud->points[midPoint].z + 5.0f;
        cout << "zLimit :  " << zLimit << endl;
        float x,y,z;
        for(int i = 0; i < outlier_cloud->points.size(); i++)
        {
            x = outlier_cloud->points[i].x;
            y = outlier_cloud->points[i].y;
            z = outlier_cloud->points[i].z;
            if ((x > -500 && x < 380 && y > -175 && y < 215 && z > 0))
                cloud_filtered->push_back(outlier_cloud->points[i]);
        }
    }
    return cloud_filtered;
}

void PclPlane::mergeSort(PointCloud<PointXYZ>::Ptr sortCloud, int vecSize)
{
    PointCloud<PointXYZ>::Ptr sortedC(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr sortedC2(new PointCloud<PointXYZ>);
    //sorted_x->clear();
    //sorted_y->clear();

    PointXYZ* arrx = new PointXYZ[vecSize]();
    PointXYZ* arry = new PointXYZ[vecSize]();

    for (int i = 0; i < sortCloud->size(); i++)
    {
        arrx[i] = sortCloud.get()->points[i];
        arry[i] = sortCloud.get()->points[i];
    }

    arrx = mergeSortX(arrx, 1, vecSize);
    for (int i = 0; i < sortCloud->size(); i++)
        sortedC->push_back(arrx[i]);
    sorted_x = sortedC;

    arry = mergeSortY(arry,1,vecSize);
    for (int i = 0; i < sortCloud->size(); i++)
        sortedC2->push_back(arry[i]);
    sorted_y = sortedC2;


    delete[] arrx;
    delete[] arry;
}

PointXYZ* PclPlane::mergeSortX(PointXYZ arr[], int l, int r)						// recursive mergeSort function
{
    if (l < r)
    {
        int mid = (int)floor(l + (r - l) / 2);		// were sub arrays should split
        int lef = mid - l + 1;						// start index of subarray
        int rig = r - mid;							// end index of subarray

        PointXYZ* L = new PointXYZ[lef]();
        PointXYZ* R = new PointXYZ[rig]();

        for (int i = 0; i < lef; i++)
            L[i] = arr[ i];
        for (int j = 0; j < rig; j++)
            R[j] = arr[lef + j];

        L = mergeSortX(L, l, mid);
        R = mergeSortX(R, mid +1 , r);

        PointXYZ* temp = mergeX(L, lef, R, rig);

        return temp;
    }

    return arr;
}
PointXYZ* PclPlane::mergeX(PointXYZ arr1[], int arr1Size, PointXYZ arr2[], int arr2Size)
{

    PointXYZ* temp = new PointXYZ[arr1Size + arr2Size]();
    int i, j, k;
    i = 0;
    j = 0;
    k = 0;
    while (i < arr1Size && j < arr2Size)
    {
        if (arr1[i].x > arr2[j].x)
        {
            temp[k].x = arr2[j].x;
            j++;
        }
        else
        {
            temp[k].x = arr1[i].x;
            i++;
        }
        k++;
    }
    while (j < arr2Size)
    {
        temp[k].x = arr2[j].x;
        j++;
        k++;
    }
    while (i < arr1Size)
    {
        temp[k].x = arr1[i].x;
        i++;
        k++;
    }
    return temp;
}
PointXYZ* PclPlane::mergeSortY(PointXYZ arr[], int l, int r)						// recursive mergeSort function
{
    if (l < r)
    {
        int mid = (int)floor(l + (r - l) / 2);		// were sub arrays should split
        int lef = mid - l + 1;						// start index of subarray
        int rig = r - mid;							// end index of subarray

        PointXYZ* L = new PointXYZ[lef]();
        PointXYZ* R = new PointXYZ[rig]();

        for (int i = 0; i < lef; i++)
            L[i] = arr[ i];
        for (int j = 0; j < rig; j++)
            R[j] = arr[lef + j];

        L = mergeSortY(L, l, mid);
        R = mergeSortY(R, mid +1 , r);

        PointXYZ* temp = mergeY(L, lef, R, rig);

        return temp;
    }

    return arr;
}
PointXYZ* PclPlane::mergeY(PointXYZ arr1[], int arr1Size, PointXYZ arr2[], int arr2Size)
{

    PointXYZ* temp = new PointXYZ[arr1Size + arr2Size]();
    int i, j, k;
    i = 0;
    j = 0;
    k = 0;
    while (i < arr1Size && j < arr2Size)
    {
        if (arr1[i].y > arr2[j].y)
        {
            temp[k].y = arr2[j].y;
            j++;
        }
        else
        {
            temp[k].y = arr1[i].y;
            i++;
        }
        k++;
    }
    while (j < arr2Size)
    {
        temp[k].y = arr2[j].y;
        j++;
        k++;
    }
    while (i < arr1Size)
    {
        temp[k].y = arr1[i].y;
        i++;
        k++;
    }
    return temp;
}


PclPlane::~PclPlane() {}
