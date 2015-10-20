#include "queue.h"

/*
 * init_queue - ��ʼ������
 *
 * @q: Ŀ�����
 */
void init_queue(Queue_T *q) {
    q->head = 0;
    q->tail = 0;
}

static void fix_queue_edge(Queue_T *q) {
    if ((QUEUE_BUFFER_SIZE + 1) == q->tail)
        q->tail = 0;
    if ((QUEUE_BUFFER_SIZE + 1) == q->head)
        q->head = 0;
}

/*
 * get_queue_size - ���г���
 *
 * @q: Ŀ�����
 */
int get_queue_size(Queue_T *q) {
    fix_queue_edge(q);
    
    if (q->head < q->tail)
        return q->tail - q->head;
    else if (q->head == q->tail)
        return 0;
    else
        return (QUEUE_BUFFER_SIZE - q->head) + q->tail +1;
}
/*
 * is_queue_empty - ��ѯ�����Ƿ�Ϊ��
 *
 * @q: Ŀ�����
 * return: TRUE-��
 */
bool is_queue_empty(Queue_T *q) {
    fix_queue_edge(q);
    return (q->head == q->tail) ? TRUE : FALSE;
}
/*
 * is_queue_full - ��ѯ�����Ƿ�full
 *
 * @q: Ŀ�����
 * return: TRUE-��
 */
bool is_queue_full(Queue_T *q) {
    return (QUEUE_BUFFER_SIZE <= get_queue_size(q)) ? TRUE : FALSE;
}
/*
 * enqueue - �����ݲ����β
 *
 * @q: Ŀ�����
 * @data: ����
 * return: TRUE-�ɹ��������
 */
bool enqueue(Queue_T *q, uint8 data) {
    // ��������
    if (is_queue_full(q))
        return FALSE;
    
    q->buffer[q->tail] = data;
    q->tail++;
    return TRUE;
}
/*
 * dequeue - �������ݳ���
 *
 * @q: Ŀ�����
 * @re[OUT]: �������
 * return: TRUE-�ɹ�����
 */
bool dequeue(Queue_T *q, uint8 *re) {
    // ����Ϊ��
    if (is_queue_empty(q))
        return FALSE;

    *re = q->buffer[q->head];
    q->head++;
    return TRUE;
}

void clear_queue(Queue_T *q) {
	q->head = 0;
	q->tail = 0;
}
