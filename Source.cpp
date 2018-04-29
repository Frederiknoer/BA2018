#include <iostream>
class list;

struct node
{
	float value;
	node* next;
};
class list
{
public:
	list()
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
	void display()
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


int main()
{
	list l;
	l.append(0.0f);
	l.append(1.0f);
	l.append(2.0f);
	l.append(5.0f);

	l.display();

	std::cout << l.median() << std::endl;
	l.insertSort(10.0f);
	std::cout << std::endl;
	l.display();
	std::cout << l.median() << std::endl;

	
	return 0;
}