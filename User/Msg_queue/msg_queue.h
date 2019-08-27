#ifndef __MSG_QUEUE
#define __MSG_QUEUE

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>

/*单链队列－－队列的链式存储结构 */
	
typedef struct QNode
{
	CanTxMsg 		data;
	struct 			QNode *next;
}QNode,*QueuePtr;

typedef struct
{
	QueuePtr front,rear; 
}LinkQueue;

uint8_t InitQueue(LinkQueue *Q);

uint8_t DestroyQueue(LinkQueue *Q);

uint8_t ClearQueue(LinkQueue *Q);

uint8_t QueueEmpty(LinkQueue Q);

uint8_t QueueLength(LinkQueue Q);

uint8_t GetHead_Q(LinkQueue Q,CanTxMsg *e);

uint8_t EnQueue(LinkQueue *Q,CanTxMsg e);

uint8_t DeQueue(LinkQueue *Q,CanTxMsg *e);

uint8_t QueueTraverse(LinkQueue Q);

uint8_t Init_CanMsg_Queue(void);


#endif  /*__MSG_QUEUE*/












