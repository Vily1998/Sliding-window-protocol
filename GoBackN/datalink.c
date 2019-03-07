#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2800
#define ACK_TIMER 240
#define MAX_SEQ 7 //最大发送序号 

struct FRAME
{
	unsigned char kind;			/* FRAME_DATA, FRAME_ACK, FRAME_NAK */
	unsigned char ack;			/* ACK序号,即使是NAK帧也表示已经收到的帧的最大序号 */
	unsigned char seq;			/* 数据帧的序号 */
	unsigned char data[PKT_LEN];
	unsigned int  padding;
};

static int phl_ready = 0;
static int no_nak = 1;
static unsigned char ack_expected = 0; //发送窗口下界
static unsigned char next_frame_to_send = 0; //发送窗口上界 + 1
static unsigned char frame_expected = 0; //接收窗口
static unsigned char nbuffered = 0; //用于指明当前发送缓存数目，nbuffered必须小于等于MAX_SEQ
static unsigned char out_buf[MAX_SEQ + 1][PKT_LEN], in_buf[PKT_LEN]; //发送缓存和接收缓存

static void put_frame(unsigned char *frame, int len) //给帧添加CRC校验和并向物理层发送该帧
{
	*(unsigned int *)(frame + len) = crc32(frame, len);
	send_frame(frame, len + 4); //向物理层发送添加了4位校验和的帧
	phl_ready = 0;
}

static int between(unsigned char a, unsigned char b, unsigned char c) //判断b是否落在a、c窗口中
{
	return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void s_frame(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, unsigned char buffer[][PKT_LEN])//发送三种帧（data/ack/nak)的处理函数
{
	struct FRAME s;
	s.kind = fk;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);//计算上次收到的帧序号

	if (fk == FRAME_DATA)//发送数据帧
	{
		s.seq = frame_nr;//数据帧的序号
		memcpy(s.data, buffer[frame_nr], PKT_LEN);//给帧的数据域赋值
		dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
		put_frame((unsigned char *)&s, 3 + PKT_LEN);//多了3个控制域字节，因此帧长度加3
		start_timer(frame_nr, DATA_TIMER);
	}
	else if (fk == FRAME_NAK)//发送NAK帧
	{
		no_nak = 0;
		dbg_frame("Send NAK  %d\n", s.ack);
		put_frame((unsigned char *)&s, 2); // NAK帧只有kind和ack字段，长度为2
	}
	else if (fk == FRAME_ACK)//发送ACK帧
	{
		dbg_frame("Send ACK  %d\n", s.ack);
		put_frame((unsigned char *)&s, 2); // ACK帧只有kind和ack字段，长度为2
	}

	phl_ready = 0;
	stop_ack_timer(); //只要发出一帧了，ACK计时器就可以停止了
}

int main(int argc, char **argv)
{
	int event, arg;
	struct FRAME f;
	int len = 0;
	int i;

	protocol_init(argc, argv); //运行环境初始化
	lprintf("Designed by 张绍磊, build: " __DATE__"  "__TIME__"\n");
	enable_network_layer();

	while (1)
	{
		event = wait_for_event(&arg);
		switch (event)
		{
		case NETWORK_LAYER_READY: //网络层有要发送的分组
			dbg_event("Network_Layer_Ready;\n");
			get_packet(out_buf[next_frame_to_send]); //从网络层接收一个数据包
			nbuffered++;
			dbg_frame("----Send DATA %d %d ;\n", next_frame_to_send, (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
			s_frame(FRAME_DATA, next_frame_to_send, frame_expected, out_buf);
			next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1);
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;

		case FRAME_RECEIVED: //收到一帧
			dbg_event("Frame_Received;\n");
			len = recv_frame((unsigned char *)&f, sizeof f); //从物理层收到一帧,len为收到的帧的长度
			if (len < 5 || crc32((unsigned char *)&f, len) != 0) //发现错误
			{
				dbg_frame("****Receiver Error , Bad CRC checksum;\n");
				if (no_nak)
				{
					dbg_frame("----Send NAK %d ;\n", (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
					s_frame(FRAME_NAK, 0, frame_expected, out_buf);
				}
				break;
			}
			while (between(ack_expected, f.ack, next_frame_to_send)) //累计确认
			{
				nbuffered--;
				stop_timer(ack_expected);
				ack_expected = (ack_expected + 1) % (MAX_SEQ + 1);
			}
			if (f.kind == FRAME_DATA) //收到的是数据帧
			{
				dbg_frame("----Recv DATA %d %d , ID %d\n", f.seq, f.ack, *(short *)f.data);
				if ((f.seq != frame_expected) && no_nak) // 若该帧不是想要的数据帧且未发送过NAK，则发送NAK帧
				{
					dbg_frame("----Send NAK %d ;\n", (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
					s_frame(FRAME_NAK, 0, frame_expected, out_buf);
				}
				else
					start_ack_timer(ACK_TIMER);
				if (f.seq == frame_expected) //收到的帧落在接受窗口中
				{
					dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
					memcpy(in_buf, f.data, len - 7); //要得到帧中的数据，要去掉3字节的控制字段和4字节的CRC校验和
					put_packet(in_buf, len - 7); //将数据包传给网络层
					no_nak = 1;
					frame_expected = (frame_expected + 1) % (MAX_SEQ + 1);
					start_ack_timer(ACK_TIMER);
				}
			}
			if ((f.kind == FRAME_NAK) && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) //收到了NAK帧，缓存中的帧要全部重发
			{
				dbg_frame("Recv NAK %d\n", f.ack);
				next_frame_to_send = ack_expected;
				for (i = 1; i <= nbuffered; i++)
				{
					s_frame(FRAME_DATA, next_frame_to_send, frame_expected, out_buf);
					next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1);
				}
			}
			if (f.kind == FRAME_ACK) //收到的是ACK帧，对ACK的处理在上面已完成
				dbg_frame("Recv ACK  %d\n", f.ack);
			break;

		case DATA_TIMEOUT: //数据的定时器超时，缓存中的帧要全部重发
			dbg_event("**** DATA %d timeout\n", arg);
			next_frame_to_send = ack_expected;
			for (i = 1; i <= nbuffered; i++)
			{
				s_frame(FRAME_DATA, next_frame_to_send, frame_expected, out_buf);
				next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1);
			}
			break;

		case ACK_TIMEOUT: //ACK定时器超时，不再等待捎带确认，单独发送一个ACK帧
			dbg_event("**** ACK %d timeout\n", arg);
			s_frame(FRAME_ACK, 0, frame_expected, out_buf);
			break;
		}

		if (nbuffered < MAX_SEQ && phl_ready)
			enable_network_layer();
		else
			disable_network_layer();
	}
}