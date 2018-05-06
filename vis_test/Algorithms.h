//
// Created by fred on 4/19/18.
//

#ifndef VIS_TEST_ALGORITHMS_H
#define VIS_TEST_ALGORITHMS_H


#include <vector>
#include <cmath>
#include <eigen3/Eigen/SVD>

class Algorithms {
public:

    struct pts
    {
        float x;
        float y;
        float z;
    };


    Algorithms();
    std::vector<pts> mergeSortX(std::vector<pts>);
    std::vector<pts> mergeSortY(std::vector<pts>);
    std::vector<pts> removeOutliers(std::vector<pts> inputVec, std::vector<float> corners, float xDisplacement, float yDisplacement);

    void leastSquarSVD(std::vector<pts>);
    float getDistToPlane(float x, float y, float z);


    float coeffA = 0.0f, coeffB = 0.0f, coeffC = 0.0f, coeffD = 0.0f;


private:
    pts* mergeSortX(pts* arr, int l, int r);
    pts* mergeX(pts* arr, int l, pts* r, int m);
    pts* mergeSortY(pts* arr, int l, int r);
    pts* mergeY(pts* arr, int l, pts* r, int m);


};


#endif //VIS_TEST_ALGORITHMS_H
