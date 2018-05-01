//
// Created by fred on 4/19/18.
//

#ifndef VIS_TEST_ALGORITHMS_H
#define VIS_TEST_ALGORITHMS_H


#include <vector>
#include <cmath>
#include <eigen3/Eigen/SVD>
#include <iostream>
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
struct node
{
	float value = 0.0f;
	node* next;
};
class llist
{
public:
	llist()
	{
		head = NULL;
		tail = NULL;
	}
	void append(float value)
	{
		node *temp = new node;
		temp->value = value;
		temp->next = NULL;
		if (head == NULL)
		{
			head = temp;
			tail = temp;
			temp = NULL;
		}
		else
		{
			tail->next = temp;
			tail = temp;
		}
	}
	void insertSort(float value)
	{
		node *temp = new node;
		if (head->value > value)
		{
			temp->value = value;
			temp->next = head;
			head = temp;
		}
		else
		{
			temp = head;
			while (temp->next != NULL && temp->next->value < value)
			{
				temp = temp->next;
			}
			node *temp2 = new node;
			temp2->value = value;
			temp2->next = temp->next;
			temp->next = temp2;
			
		}
	}
	float median()
	{
		node *temp = head;
		node *temp2 = head;
		if (head == NULL)
			return 0.0f;
		while (temp2 != NULL && temp2->next != NULL)
		{
			temp = temp->next;
			temp2 = temp2->next->next;
		}
		return temp->value;
	}
	float average()
	{
		if (head == NULL)
			return 0.0f;
		node *temp = new node;
		temp = head;
		float sum = 0.0f;
		int ite = 0;
		while (temp != NULL)
		{
			sum += temp->value;
			ite++;
			temp = temp->next;
		}
		std::cout << sum / (float)ite << std::endl;
		return sum / (float)ite;
	}
	/*void display()
	{
		node *temp = new node;
		temp = head;
		while (temp != NULL)
		{
			std::cout << temp->value << "\t";
			temp = temp->next;
		}
	}*/
private:
	node *head, *tail;
};

#endif //VIS_TEST_ALGORITHMS_H
