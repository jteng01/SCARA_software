#include <stdlib.h>

struct PositionList{
	float coordinates[2];
	float time;
	struct PositionList* next;
};
typedef struct PositionList PositionList;



void AppendPosition(PositionList * head, float xpos, float ypos, int time) 
{
	PositionList * new = (PositionList*) malloc(sizeof(PositionList));
	new->time = time;
	new->coordinates[0] = xpos;
	new->coordinates[1] = ypos;
	
	if (head == NULL)
		head = new;
	else
		while (new->next != NULL)
			new = new->next;




}