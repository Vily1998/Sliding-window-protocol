#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER  3000
#define ACK_TIMER 1000
#define MAX_SEQ 63
#define NR_BUFS ((MAX_SEQ + 1) / 2)

struct FRAME
{
	unsigned char kind;			/* FRAME_DATA, FRAME_ACK, FRAME_NAK */
	unsigned char ack;			/* ACK���,��ʹ��NAK֡Ҳ��ʾ�Ѿ��յ���֡�������� */
	unsigned char seq;			/* ����֡����� */
	unsigned char data[PKT_LEN];
	unsigned int  padding;
};

int no_nak = 1;
static int phl_ready = 0;
static unsigned char ack_expected = 0; //���ʹ����½�
static unsigned char next_frame_to_send = 0; //���ʹ����Ͻ� + 1
static unsigned char frame_expected = 0; //���մ����½�
static unsigned char too_far = NR_BUFS; //���մ����Ͻ�
static unsigned char out_buf[NR_BUFS][PKT_LEN]; //���ͻ���
static unsigned char in_buf[NR_BUFS][PKT_LEN]; //���ջ���
int arrived[NR_BUFS]; //ָ�����ջ���������
static unsigned char nbuffered = 0; //ָ����ǰ���ͻ�����Ŀ


static int between(unsigned char a, unsigned char b, unsigned char c)
{
	return ( ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)));
}

static void put_frame(unsigned char *frame, int len) //��֡���CRCУ��Ͳ�������㷢�͸�֡
{
	*(unsigned int *)(frame + len) = crc32(frame, len);
	send_frame(frame, len + 4); //������㷢�������4λУ��͵�֡
	phl_ready = 0;
}

static void s_frame(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, unsigned char buffer[][PKT_LEN])//��������֡��data/ack/nak)�Ĵ�����
{
	struct FRAME s;
	s.kind = fk;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);//�����ϴ��յ���֡���

	if (fk == FRAME_DATA)//��������֡
	{
		s.seq = frame_nr;//����֡�����
		memcpy(s.data, buffer[frame_nr % NR_BUFS], PKT_LEN);//��֡��������ֵ
		dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
		put_frame((unsigned char *)&s, 3 + PKT_LEN);//����3���������ֽڣ����֡���ȼ�3
		start_timer(frame_nr % NR_BUFS, DATA_TIMER);
	}
	else if (fk == FRAME_NAK)//����NAK֡
	{
		no_nak = 0;
		dbg_frame("Send NAK  %d\n", s.ack);
		put_frame((unsigned char *)&s, 2); // NAKֻ֡��kind��ack�ֶΣ�����Ϊ2
	}
	else if (fk == FRAME_ACK)//����ACK֡
	{
		dbg_frame("Send ACK  %d\n", s.ack);
		put_frame((unsigned char *)&s, 2); // ACKֻ֡��kind��ack�ֶΣ�����Ϊ2
	}

	phl_ready = 0;
	stop_ack_timer(); //ֻҪ����һ֡�ˣ�ACK��ʱ���Ϳ���ֹͣ��
}

int main(int argc, char **argv)
{
	int event, arg;
	struct FRAME r;
	int len = 0;
	int i;

	for(i = 0;i <= NR_BUFS - 1;i ++)//��ʼ��
		arrived[i] = 0;
	protocol_init(argc, argv); //���л�����ʼ��
    lprintf("Designed by ������, build: " __DATE__"  "__TIME__"\n");
	disable_network_layer();

	while (1)
	{
		 event = wait_for_event(&arg); 
		 switch(event)
		 {
			 case NETWORK_LAYER_READY: //�������Ҫ���͵ķ���
				 dbg_event("Network_Layer_Ready;\n");
				 get_packet(out_buf[next_frame_to_send % NR_BUFS]); //����������һ�����ݰ�
				 nbuffered++;
				 dbg_frame("----Send DATA %d %d ;\n", next_frame_to_send, (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
				 s_frame(FRAME_DATA,next_frame_to_send,frame_expected,out_buf);
				 next_frame_to_send=(next_frame_to_send + 1) % ( MAX_SEQ + 1);
				 break;

			 case PHYSICAL_LAYER_READY:
				 phl_ready = 1;
				 break;

			 case FRAME_RECEIVED: //�յ�һ֡
				 dbg_event("Frame_Received;\n");
				 len = recv_frame((unsigned char *)&r, sizeof r);
				 if (len < 5 || crc32((unsigned char *)&r, len) != 0) //CRCУ�鷢�ִ���
				 { 
					 dbg_frame("****Receiver Error , Bad CRC checksum;\n");
					 if (no_nak)
					 {
						 dbg_frame("----Send NAK %d ;\n", (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
						 s_frame(FRAME_NAK, 0, frame_expected, out_buf);
					 }
					 break;
				 }

				 if(r.kind == FRAME_DATA)
				 {
					 dbg_frame("----Recv DATA %d %d , ID %d\n", r.seq, r.ack, *(short *)r.data);
					 if ((r.seq != frame_expected) && no_nak) //����֡������Ҫ������֡��δ���͹�NAK������NAK֡
					 {
						 dbg_frame("----Send NAK %d ;\n", (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
						 s_frame(FRAME_NAK, 0, frame_expected, out_buf);
					 } 
					 else
						 start_ack_timer(ACK_TIMER);
					 if(between(frame_expected, r.seq, too_far) && (arrived[r.seq % NR_BUFS] == 0))//��������֡���ڽ��մ��ڷ�Χ����δ���չ���֡������
					 {
						 arrived[r.seq % NR_BUFS] = 1;
						 memcpy(in_buf[r.seq % NR_BUFS], r.data, len-7); //���������ֽڣ��ĸ�У���ֽ�
						 while(arrived[frame_expected % NR_BUFS]) //�����մ����½��֡�ѵ���ʱ���������Ͻ������
						 { 
							put_packet(in_buf[frame_expected % NR_BUFS], len - 7);
							no_nak = 1;
							arrived[frame_expected % NR_BUFS] = 0;
							frame_expected = (frame_expected + 1) % (MAX_SEQ + 1); //���մ����½� + 1
							too_far = (too_far + 1) % (MAX_SEQ + 1); //���մ����Ͻ� + 1
							start_ack_timer(ACK_TIMER);
						 }
					 }
				 }

				 if( (r.kind == FRAME_NAK) && between(ack_expected,(r.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) // ����֡ΪNAK֡�Ҹ�֡����������Ҫ��֡�ڷ��ʹ��ڷ�Χ�ڣ�����������Ҫ��֡
				 {
					 dbg_frame("----Recv NAK %d \n", r.ack);
					 dbg_frame("----Send DATA %d %d ;\n", (r.ack + 1) % (MAX_SEQ + 1), (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
					 s_frame(FRAME_DATA, (r.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf);
				 }

				 if(r.kind == FRAME_ACK) //�յ���ACK֡
				 {
					 dbg_frame("----Recv ACK %d\n", r.ack);				
				 }
				 while(between(ack_expected, r.ack, next_frame_to_send)) //ѡ���ش����ۼ�ȷ��
				 {
					 nbuffered = nbuffered - 1;
					 stop_timer(ack_expected % NR_BUFS);
					 ack_expected=(ack_expected + 1) % (MAX_SEQ + 1);
				 }
				 break;

			 case DATA_TIMEOUT: 
				 dbg_event("---- DATA %d timeout\n", arg); 
				 s_frame(FRAME_DATA, ack_expected, frame_expected, out_buf);
				 break;

			 case ACK_TIMEOUT: //ACK��ʱ����ʱ�����ٵȴ��Ӵ�ȷ�ϣ���������һ��ACK֡
				 dbg_event("---- ACK %d timeout\n", arg); 
				 s_frame(FRAME_ACK, 0, frame_expected, out_buf);
				 break;
		 }

		 if(nbuffered < NR_BUFS &&  phl_ready)
			 enable_network_layer();
		 else
			 disable_network_layer();

	}
}
