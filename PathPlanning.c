// Online C compiler to run C program online
#include <stdio.h>
#include <stdlib.h>

struct PositionNode {
	float xpos;
	float ypos;
	int pauseTime;
	struct PositionNode* next;
};
typedef struct PositionNode PositionNode;



void appendPosition(PositionNode** head, float xpos, float ypos, int msPauseTime)
{
	PositionNode* newNode = (PositionNode*)malloc(sizeof(PositionNode));

	newNode->pauseTime = msPauseTime;
	newNode->xpos = xpos;
	newNode->ypos = ypos;
	newNode->next = NULL;

	PositionNode* p = *head;

	if (*head == NULL)
		*head = newNode;

	else {//traverse to the end
		while (p->next != NULL) {
			p = p->next;
		}
		p->next = newNode;
	}
}

void insertPosition(PositionNode** head, float xpos, float ypos, int msPauseTime, int pos) {

	PositionNode* newNode = (PositionNode*)malloc(sizeof(PositionNode));

	newNode->pauseTime = msPauseTime;
	newNode->xpos = xpos;
	newNode->ypos = ypos;
	newNode->next = NULL;

	if (*head == NULL) {
		*head = newNode;
		return;
	}

	PositionNode* p = *head;
	PositionNode* nextHead = p->next;
	int i;

	if (pos == 0) {
		newNode->next = *head;
		*head = newNode;
		return;
	}

	for (i = 1; i < pos; i++) {
		nextHead = nextHead->next;
		p = p->next;

		if (nextHead->next == NULL) {
			nextHead->next = newNode;
			return;
		}

	}

	p->next = newNode;
	newNode->next = nextHead;
}

void deletePosition(PositionNode** head, int pos) {

	if (*head == NULL)
		return;

	PositionNode* p = *head;
	PositionNode* nextHead = p->next;

	if (pos == 0) {
		free(p);
		*head = nextHead;
		return;
	}

	int i;

	for (i = 1; i < pos; i++) {
		nextHead = nextHead->next;
		p = p->next;
		if (nextHead->next == NULL) {
			free(nextHead);
			p->next = NULL;
			return;
		}
	}

	p->next = nextHead->next;
	free(nextHead);

}

void traverse(PositionNode* head) {
	PositionNode* p = head;
	while (p != NULL) {
		printf("%f, %f, %d \n", p->xpos, p->ypos, p->pauseTime);
		p = p->next;
	}
	printf("\n");
}

int main() {

	PositionNode* headList = NULL;
	FILE* fileptr;

	fileptr = fopen("PositionData", "r");

	if (fileptr == NULL) {
		printf("Can not open file.");
	}
	else
	{
		float xAppend;
		float yAppend;
		float deltaTAppend;
		while (fscanf(fileptr, "%f %f %d\n", xAppend, yAppend, deltaTAppend) != EOF)
		{
			appendPosition(&headList, xAppend, yAppend, deltaTAppend);
		}

		fclose(fileptr);
	}

	traverse(headList);

	deletePosition(&headList, 5);
	deletePosition(&headList, 2);
	deletePosition(&headList, 0);
	traverse(headList);

	insertPosition(&headList, 0.3, 0.2, 1, 0);
	insertPosition(&headList, 0.3, 0.1, 3, 2);
	insertPosition(&headList, 0.3, 0.1, 6, 5);
	traverse(headList);

	printf("\n");

	return 0;
}