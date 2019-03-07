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
	unsigned char ack;			/* ACK序号,即使是NAK帧也表示已经收到的帧的最大序号 */
	unsigned char seq;			/* 数据帧的序号 */
	unsigned char data[PKT_LEN];
	unsigned int  padding;
};

int no_nak = 1;
static int phl_ready = 0;
static unsigned char ack_expected = 0; //发送窗口下界
static unsigned char next_frame_to_send = 0; //发送窗口上界 + 1
static unsigned char frame_expected = 0; //接收窗口下界
static unsigned char too_far = NR_BUFS; //接收窗口上界
static unsigned char out_buf[NR_BUFS][PKT_LEN]; //发送缓存
static unsigned char in_buf[NR_BUFS][PKT_LEN]; //接收缓存
int arrived[NR_BUFS]; //指明接收缓冲区满空
static unsigned char nbuffered = 0; //指明当前发送缓存数目


static int between(unsigned char a, unsigned char b, unsigned char c)
{
	return ( ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)));
}

static void put_frame(unsigned char *frame, int len) //给帧添加CRC校验和并向物理层发送该帧
{
	*(unsigned int *)(frame + len) = crc32(frame, len);
	send_frame(frame, len + 4); //向物理层发送添加了4位校验和的帧
	phl_ready = 0;
}

static void s_frame(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, unsigned char buffer[][PKT_LEN])//发送三种帧（data/ack/nak)的处理函数
{
	struct FRAME s;
	s.kind = fk;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);//计算上次收到的帧序号

	if (fk == FRAME_DATA)//发送数据帧
	{
		s.seq = frame_nr;//数据帧的序号
		memcpy(s.data, buffer[frame_nr % NR_BUFS], PKT_LEN);//给帧的数据域赋值
		dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
		put_frame((unsigned char *)&s, 3 + PKT_LEN);//多了3个控制域字节，因此帧长度加3
		start_timer(frame_nr % NR_BUFS, DATA_TIMER);
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
	struct FRAME r;
	int len = 0;
	int i;

	for(i = 0;i <= NR_BUFS - 1;i ++)//初始化
		arrived[i] = 0;
	protocol_init(argc, argv); //运行环境初始化
    lprintf("Designed by 张绍磊, build: " __DATE__"  "__TIME__"\n");
	disable_network_layer();

	while (1)
	{
		 event = wait_for_event(&arg); 
		 switch(event)
		 {
			 case NETWORK_LAYER_READY: //网络层有要发送的分组
				 dbg_event("Network_Layer_Ready;\n");
				 get_packet(out_buf[next_frame_to_send % NR_BUFS]); //从网络层接收一个数据包
				 nbuffered++;
				 dbg_frame("----Send DATA %d %d ;\n", next_frame_to_send, (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
				 s_frame(FRAME_DATA,next_frame_to_send,frame_expected,out_buf);
				 next_frame_to_send=(next_frame_to_send + 1) % ( MAX_SEQ + 1);
				 break;

			 case PHYSICAL_LAYER_READY:
				 phl_ready = 1;
				 break;

			 case FRAME_RECEIVED: //收到一帧
				 dbg_event("Frame_Received;\n");
				 len = recv_frame((unsigned char *)&r, sizeof r);
				 if (len < 5 || crc32((unsigned char *)&r, len) != 0) //CRC校验发现错误
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
					 if ((r.seq != frame_expected) && no_nak) //若该帧不是想要的数据帧且未发送过NAK，则发送NAK帧
					 {
						 dbg_frame("----Send NAK %d ;\n", (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
						 s_frame(FRAME_NAK, 0, frame_expected, out_buf);
					 } 
					 else
						 start_ack_timer(ACK_TIMER);
					 if(between(frame_expected, r.seq, too_far) && (arrived[r.seq % NR_BUFS] == 0))//若该数据帧落在接收窗口范围内且未接收过该帧，接收
					 {
						 arrived[r.seq % NR_BUFS] = 1;
						 memcpy(in_buf[r.seq % NR_BUFS], r.data, len-7); //三个控制字节，四个校验字节
						 while(arrived[frame_expected % NR_BUFS]) //当接收窗口下界的帧已到达时，按次序上交网络层
						 { 
							put_packet(in_buf[frame_expected % NR_BUFS], len - 7);
							no_nak = 1;
							arrived[frame_expected % NR_BUFS] = 0;
							frame_expected = (frame_expected + 1) % (MAX_SEQ + 1); //接收窗口下界 + 1
							too_far = (too_far + 1) % (MAX_SEQ + 1); //接收窗口上界 + 1
							start_ack_timer(ACK_TIMER);
						 }
					 }
				 }

				 if( (r.kind == FRAME_NAK) && between(ack_expected,(r.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) // 若该帧为NAK帧且该帧表明的所需要的帧在发送窗口范围内，发送其所需要的帧
				 {
					 dbg_frame("----Recv NAK %d \n", r.ack);
					 dbg_frame("----Send DATA %d %d ;\n", (r.ack + 1) % (MAX_SEQ + 1), (frame_expected + MAX_SEQ) % (MAX_SEQ + 1));
					 s_frame(FRAME_DATA, (r.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf);
				 }

				 if(r.kind == FRAME_ACK) //收到了ACK帧
				 {
					 dbg_frame("----Recv ACK %d\n", r.ack);				
				 }
				 while(between(ack_expected, r.ack, next_frame_to_send)) //选择重传的累计确认
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

			 case ACK_TIMEOUT: //ACK定时器超时，不再等待捎带确认，单独发送一个ACK帧
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
