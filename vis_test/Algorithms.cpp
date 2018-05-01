//
// Created by fred on 4/19/18.
//


#include "Algorithms.h"

Algorithms::Algorithms() {}

std::vector<Algorithms::pts> Algorithms::removeOutliers(std::vector<pts> inputVec, std::vector<float> corners, float xDisplacement, float yDisplacement)
{
    std::vector<pts> filteredVec;
    filteredVec.clear();

    float minX = corners[0] - xDisplacement;
    float maxX = corners[1] - xDisplacement;
    float minY = corners[2] - yDisplacement;
    float maxY = corners[3] - yDisplacement;

    float x,y,z;
    for(int i = 0; i < inputVec.size(); i++)
    {
        x = inputVec[i].x;
        y = inputVec[i].y;
        z = inputVec[i].z;
        if ((x > minX && x < maxX && y > minY && y < maxY && z > 0)) //x > -500 && x < 380 && y > -175 && y < 215 && z > 0
            filteredVec.push_back(inputVec[i]);
    }
    return filteredVec;
}


void Algorithms::leastSquarSVD(std::vector<pts> input)
{
    int dataSize = input.size();
    Eigen::MatrixXf mat(dataSize,3);
    Eigen::VectorXf vec(dataSize);
    for (int i = 0; i < dataSize; i++)
    {
        mat(i, 0) = input[i].x;
        mat(i, 1) = input[i].y;
        mat(i, 2) = 1.0f;
        vec(i) = -input[i].z;
    }
    auto eig = mat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vec).eval();
    coeffA = eig(0);
    coeffB = eig(1);
    coeffC = 1.0f;
    coeffD = eig(2);
}

float Algorithms::getDistToPlane(float x, float y, float z)
{
    float distNum = coeffA*x + coeffB*y + coeffC*z + coeffD;
    float sumOfSquares = (coeffA*coeffA)+(coeffB*coeffB)+(coeffC*coeffC);
    float distDenum = sqrt(sumOfSquares);

    if((distNum/distDenum) > 0.0)
        return 0.0f;
    return std::abs(distNum/distDenum);
}
/*
std::vector<Algorithms::pts> Algorithms::mergeSortX(std::vector<pts> input)
{
    int size = input.size();
    std::vector<pts> sortedVec;
    sortedVec.clear();

    pts* arr = new pts[size]();

    for (int i = 0; i < size; i++)
        arr[i] = input[i];

    arr = mergeSortX(arr, 1, size);
    for (int i = 0; i < size; i++)
        sortedVec.push_back(arr[i]);

    delete[] arr;
    return sortedVec;
}

std::vector<Algorithms::pts> Algorithms::mergeSortY(std::vector<pts> input)
{
    int size = input.size();
    std::vector<pts> sortedVec;
    sortedVec.clear();

    pts* arr = new pts[size]();

    for (int i = 0; i < size; i++)
        arr[i] = input[i];

    arr = mergeSortY(arr, 1, size);

    for (int i = 0; i < size; i++)
        sortedVec.push_back(arr[i]);

    delete[] arr;
    return sortedVec;
}

Algorithms::pts* Algorithms::mergeSortX(pts *arr, int l, int r)
{
    if (l < r)
    {
        int mid = (int)floor(l + (r - l) / 2);		// were sub arrays should split
        int lef = mid - l + 1;						// start index of subarray
        int rig = r - mid;							// end index of subarray

        pts* L = new pts[lef]();
        pts* R = new pts[rig]();

        for (int i = 0; i < lef; i++)
            L[i] = arr[ i];
        for (int j = 0; j < rig; j++)
            R[j] = arr[lef + j];

        L = mergeSortX(L, l, mid);
        R = mergeSortX(R, mid +1 , r);

        pts* temp = mergeX(L, lef, R, rig);

        return temp;
    }

    return arr;
}

Algorithms::pts * Algorithms::mergeX(pts *arr, int l, pts *r, int m)
{
    pts* temp = new pts[l + m]();
    int i, j, k;
    i = 0;
    j = 0;
    k = 0;
    while (i < l && j < m)
    {
        if (arr[i].x > r[j].x)
        {
            temp[k].x = r[j].x;
            j++;
        }
        else
        {
            temp[k].x = arr[i].x;
            i++;
        }
        k++;
    }
    while (j < m)
    {
        temp[k].x = r[j].x;
        j++;
        k++;
    }
    while (i < l)
    {
        temp[k].x = arr[i].x;
        i++;
        k++;
    }
    return temp;
}

Algorithms::pts * Algorithms::mergeSortY(pts *arr, int l, int r)
{
    if (l < r)
    {
        int mid = (int)floor(l + (r - l) / 2);		// were sub arrays should split
        int lef = mid - l + 1;						// start index of subarray
        int rig = r - mid;							// end index of subarray

        pts* L = new pts[lef]();
        pts* R = new pts[rig]();

        for (int i = 0; i < lef; i++)
            L[i] = arr[ i];
        for (int j = 0; j < rig; j++)
            R[j] = arr[lef + j];

        L = mergeSortY(L, l, mid);
        R = mergeSortY(R, mid +1 , r);

        pts* temp = mergeY(L, lef, R, rig);

        return temp;
    }

    return arr;
}

Algorithms::pts * Algorithms::mergeY(pts *arr, int l, pts *r, int m)
{
    pts* temp = new pts[l + m]();
    int i, j, k;
    i = 0;
    j = 0;
    k = 0;
    while (i < l && j < m)
    {
        if (arr[i].y > r[j].y)
        {
            temp[k].y = r[j].y;
            j++;
        }
        else
        {
            temp[k].y = arr[i].y;
            i++;
        }
        k++;
    }
    while (j < m)
    {
        temp[k].y = r[j].y;
        j++;
        k++;
    }
    while (i < l)
    {
        temp[k].y = arr[i].y;
        i++;
        k++;
    }
    return temp;
}*/
//////////////////////////////////////////// 
/*struct node
{
	float value;
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
			
		while (temp2 != NULL && temp2->next != NULL)
		{
			temp = temp->next;
			temp2 = temp2->next->next;
		}
		return temp->value;
	}
	float average()
	{
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
		return sum / ite;
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
	}
private:
	node *head, *tail;
};
*/

