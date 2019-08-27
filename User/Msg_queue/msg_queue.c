/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "msg_queue.h"
#include "debug_usart.h"

LinkQueue Can1Msg_Queue;
LinkQueue Can2Msg_Queue;

/*�������*/
uint8_t InitQueue(LinkQueue *Q)
{
	(*Q).front=(*Q).rear=(QueuePtr)malloc(sizeof(QNode));
	if(!(*Q).front)
		return 0;
	(*Q).front->next=NULL;
	return 1;
}

/*���ٶ���*/
uint8_t DestroyQueue(LinkQueue *Q)
{ 
	while((*Q).front)
	{
	 (*Q).rear=(*Q).front->next;
	 free((*Q).front);
	 (*Q).front=(*Q).rear;
	}
	return 1;
}

/*��ն���*/
uint8_t ClearQueue(LinkQueue *Q)
{ 
	QueuePtr p,q;
	(*Q).rear=(*Q).front;
	p=(*Q).front->next;
	(*Q).front->next=NULL;
	while(p)
	{
	 q=p;
	 p=p->next;
	 free(q);
	}
	return 1;
}

/*����Ϊ�շ���1�����򷵻�0*/
uint8_t QueueEmpty(LinkQueue Q)
{ 
	if(Q.front==Q.rear)
	 return 1;
	else
	 return 0;
}

/*���ض��г���*/
uint8_t QueueLength(LinkQueue Q)
{ 
	int i=0;
	QueuePtr p;
	p=Q.front;
	while(Q.rear!=p)
	{
	 i++;
	 p=p->next;
	}
	return i;
}

/*��ȡ����ͷ*/
uint8_t GetHead_Q(LinkQueue Q,CanTxMsg *e) 
{ 
	QueuePtr p;
	if(Q.front==Q.rear)
	 return 0;
	p=Q.front->next;
	*e=p->data;
	return 1;
}

/*������*/
uint8_t EnQueue(LinkQueue *Q,CanTxMsg e)
{ 
	QueuePtr p=(QueuePtr)malloc(sizeof(QNode));
	if(!p) /* �洢����ʧ�� */
		return 0;
	p->data=e;
	p->next=NULL;
	(*Q).rear->next=p;
	(*Q).rear=p;
	return 1;
}
/*������*/
uint8_t DeQueue(LinkQueue *Q,CanTxMsg *e)
{ 
	QueuePtr p;
	if((*Q).front==(*Q).rear)
	 return 0;
	p=(*Q).front->next;
	*e=p->data;
	(*Q).front->next=p->next;
	if((*Q).rear==p)
	 (*Q).rear=(*Q).front;
	free(p);
	return 1;
}

/*��������*/
uint8_t QueueTraverse(LinkQueue Q)
{ 
	QueuePtr p;
	p=Q.front->next;
	while(p)
	{
		for(uint8_t i=0;i<8;i++)
		{
			printf("%d ",p->data.Data[i]);
		}
		printf("%d \n",p->data.StdId);
	  p=p->next;
	}
	return 1;
}

uint8_t Init_CanMsg_Queue(void)
{
	//��ʼ������
	if(!(InitQueue(&Can1Msg_Queue)&&InitQueue(&Can2Msg_Queue)))
		return 0;
	return 1;
}






















