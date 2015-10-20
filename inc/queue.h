#ifndef QUEUE_H
#define QUEUE_H

#define QUEUE_BUFFER_SIZE 127

#include "types.h"

typedef struct {
    uint8 buffer[QUEUE_BUFFER_SIZE + 1];
    int head;
    int tail;
} Queue_T;

/*
 * init_queue - ��ʼ������
 *
 * @q: Ŀ�����
 */
void init_queue(Queue_T *q);
/*
 * get_queue_size - ���г���
 *
 * @q: Ŀ�����
 */
int get_queue_size(Queue_T *q);
/*
 * is_queue_empty - ��ѯ�����Ƿ�Ϊ��
 *
 * @q: Ŀ�����
 * return: TRUE-��
 */
bool is_queue_empty(Queue_T *q);
/*
 * is_queue_full - ��ѯ�����Ƿ�full
 *
 * @q: Ŀ�����
 * return: TRUE-��
 */
bool is_queue_full(Queue_T *q);
/*
 * enqueue - �����ݲ����β
 *
 * @q: Ŀ�����
 * @data: ����
 * return: TRUE-�ɹ��������
 */
bool enqueue(Queue_T *q, uint8 data);
/*
 * dequeue - �������ݳ���
 *
 * @q: Ŀ�����
 * @re[OUT]: �������
 * return: TRUE-�ɹ�����
 */
bool dequeue(Queue_T *q, uint8 *re);

void clear_queue(Queue_T *q);
#endif
