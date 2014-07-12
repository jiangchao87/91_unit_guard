
#include "list.h" 

#define   uchar  unsigned char
#define   uint   unsigned int

uchar  agn_int7037t;									// 如果初始化7037失败 以后每10秒重新初始化1次
uchar  rst_3s;
uchar  fsyc;
unsigned int uuuu;

uchar  timebase,ms10,sec1s,min60m;						// 
uchar  key_vl,key_vlBF,key_tup,key_td;					// 按键
#define SL_STATE_NUMBER	3

enum  RL_state{received=0, rl_empty=1};				// R 接收  状态  0已经收到  1空
enum  SL_state{needsend=0, waitACK=1, sl_empty=2};		// S 发送        0需要发送  1等ACK  2空

volatile list_head	receive_list[2];		 			// 接收表 表头
volatile list_head	send_list[SL_STATE_NUMBER];			// 发送表 表头

 
uint   ad_value;
uchar  ad_tmp,ad_tmpbf;									// 室内温度

uchar  volume,volumeBF;									// 声音

uchar  pwer_t;											// 电源按键按下1秒    
uint   rst_armt;										// 5分钟没有收到arm数据
uchar  SD_sec,SD_min,SD_hour,SD_day,SD_mth,SD_year;	// 时钟1302
uchar  T1302_year,T1302_mth,T1302_week,T1302_day,T1302_hour,T1302_min,T1302_sec;

uchar  safe_zt[8],safe_bf[8],pot_a,pa_sm_t;			// 安防状态
uchar  safe_jj,safe_jjbf,safe_jtm,safe_jbz;

uchar  kScr_hlbz;										// 校准屏目
uint   kScr_t;

uchar  rut_zt,tlphn_zt;									// 线路状态 电话分机状态

uchar  ledzt[8],Rlayzt[8];								// led指示灯状态 继电器状态

uchar  Telarm[20],Telmdl[20];							// 电话号码缓冲区 [0]存长度
uchar  Dtmfarm[20],Dtmfmdl[20];						// Dtmf号码缓冲区

uchar  bw_armbf,bw_telbf;								// 
//--------------------------------------------------------
// Arm 通信
volatile uchar rsvbufi[35],rsvbufm[35],rsv_p,rsvover,headf5bz,rsv_ack;// 串口接收	

volatile uchar Sndbuf[5][35],snd_p,snd_frm,snd_BWenNo;			// 发送Sndbuf[5][31]是计时器
// Sndbuf[x][6]报文序号  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]发送的123次数 Sndbuf[x][30]计时200ms  

//---------------------------------------------------------
// 电话模块
volatile uchar RxTelbufi[30], RxTelbufm[30],RxTel_p,RxTelov,RxTelf5bz;// 串口接收	

volatile uchar TxTelbuf[5][33],Tel_p,Tel_frm,Tel_BWenNo;		// 发送Sndbuf[5][31]是计时器
// Sndbuf[x][6]报文序号  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]发送的123次数 Sndbuf[x][30]计时200ms  




#define		_PIN_TE_RX	 			PTFD_PTFD0 		// 与电话模块通信，输入
#define		_PIN_TE_TX	 			PTFD_PTFD1 		// 与电话模块通信  输出

#define		_PIN_5465_UD_ 			PTCD_PTCD4 		// 声音控制 数字电位器
#define		_PIN_5465_CS_ 			PTCD_PTCD6		// 声音控制 数字电位器

#define		_STATE_51_RESET			0				//  
#define		_STATE_51_NORMAL		1				//  

#define  	_PIN_CkScreen	 		PTBD_PTBD4		// 输入 检测 校准屏幕

#define  	_PIN_RESET_51 			PTDD_PTDD0		// 复位 arm

#define 	_PIN_AW60_UART_EN		PTFD_PTFD6		// 2031  预留

//----------------------------------------------------
// LED 指示灯
#define  	_PIN_LED_MR	 		PTCD_PTCD2			// 免扰        
#define  	_PIN_LED_JJAF 		PTCD_PTCD0			// 紧急指示灯  
#define  	_PIN_LED_TX	 		PTCD_PTCD1			// 通信       

  
// 安防输入
#define	    SAFETY_SENSOR_0	    PTAD_PTAD0			// 安防 8路 输入
#define	    SAFETY_SENSOR_1		PTAD_PTAD1
#define	    SAFETY_SENSOR_2		PTAD_PTAD2
#define	    SAFETY_SENSOR_3		PTAD_PTAD3
#define	    SAFETY_SENSOR_4		PTAD_PTAD4
#define	    SAFETY_SENSOR_5		PTAD_PTAD5
#define	    SAFETY_SENSOR_6		PTAD_PTAD6
#define	    SAFETY_SENSOR_7		PTAD_PTAD7
 
// 触摸按键
//#define	PIN_KM		    	PTFD_PTFD0			// 开门
#define	    PIN_WC		    	PTGD_PTGD3			// 外出
#define	    PIN_TH		    	PTDD_PTDD5			// 通话
//#define   PIN_JJ		    	PTFD_PTFD3			// 紧急


#define	MAX_CHANGE_VOL_TIMES		3				// 最大 改变 声音 次数

//              volstate = {MAX_CHANGE_VOL_TIMES, 0, 0, 0, {0,2,6,20,21,22,23,24,25,26,27,28,29,30,31} };
//              volstate = {MAX_CHANGE_VOL_TIMES, 0, 0, 0,  {0,2,6,20,21,22,23,24,25,26,27,28,29,30,31} };
// 数字电位器脉冲表  第5脚输出电压
// 1--0.595   2--0.898   3--1.090   4--1.224   5--1.321   6--1.395   7--1.453   8--1.500   9--1.538
//10--1.569  11--1.595  12--1.617  13--1.634  14--1.649  15--1.661  16--1.673  20--1.687  25--1.650


typedef struct _TimerCount_ 
{
	volatile _UINT	delay1ms;
	_UINT	reset_delay;
	_UINT	sensor_delay;
} _TimerCount_;
volatile _TimerCount_ time_counter = { 0, 0, 0};

//uchar MenQianJi;

// 1-0.129  2-0.230  3-[0.308]4-0.386  5-[0.442]
// 6-0.499  7-0.547  8-0.588  9-0.625 10-0.658
//11-0.689 12-0.717 13-0.742 14-0.765 15-0.787
//16-0.807 17-0.826 18-0.844 19-0.862 20-0.878
//21-0.894 22-0.909 23-0.924 24-0.939 25-0.955
//26-0.970 27-0.986 28-1.003 29-1.022 30-1.043   31-1.066

// 1-15档声音
const unsigned char Volv_lvel[15][2] = 
{
	{0,	0x16},				// 0
	
	{18,0x00},				// 1
	{19,0x00},
	{20,0x00},
	{21,0x00},		
	{22,0x00},	
	
	{23,0x00},				// 6		
	{24,0x00},				// 7
	{25,0x00},	                         
	{26,0x00},	
	{27,0x00},				// 10
	{28,0x00},				//
};

 const unsigned char Volv_lvel_99999[15][2] = 
{
	{0,	0x16},				// 0
	
	{20,0x00},				// 1                 
	{21,0x00},
	{22,0x00},
	{23,0x00},		
	{24,0x00},	
	
	{25,0x00},				// 6		
	{26,0x00},				// 7
	{27,0x00},	
	{28,0x00},	
	{29,0x00},				// 10
	{29,0x00},				//
};

// 不同通话设备[门前机]，喇叭音量 和 mic灵敏度不同
const unsigned char Dvc_Lbalvl[] = {0x1C,0x1F,0x1D,0x1E,0x1F,0x00,0x01,0x03,0x05,0x07,0x09};
const unsigned char Dvc_miclvl[] = {0x00,0x1F,0x00,0x01,0x07,0x00,0x01,0x1f,0x00,0x01,0x1f,0x00,0x01,0x1f,0x00,0x01,0x1f,0x00,0x01,0x1f,0x00};

// 1 智能终端	  2 服务器    3 单元门口机    4 小门口机     5 安保机     06 远程控制手机
// 7 远程网页控制 8 电话      9 小区门口机   10 物业管理机  11 待定       12 数字别墅门口机
//13 数字分机    14 移动终端 15 电力服务器   16 超市客户端  17 停车场管理服务器
//18 网关        19 求实别墅门口机           20 9000 终端CPU2
// 0x16[-20]  0x17[-18]  0x18[-16]  0x19[-14]  0x1A[-12]  0x1B[-10]  0x1C[-8]  0x1D[-6]  0x1E[-4]  0x1F[-2]
// 0x00[0]    0x01[2]    0x02[4]    0x03[6]    0x04[8]    0x05[10]   0x06[12]  0x07[14]  0x08[16]  0x09[18]  0x0A[20]

//#pragma CONST_SEG  DEFAULT_ROM
_UINT  temp_mv[] =
{   
	//981, 979, 977, 974, 971, 968, 965, //962, 959, 955,	//-39--30
	
	//962, 959, 955,				   						//-32  -30
	//951, 947, 
	943, 939, 935, 931, 926, 922, 917, 912, 				//-29--20						 
	907, 901, 895, 890, 884, 877, 871, 865, 858, 852, 		//-19--10
	845, 837, 830, 822, 815, 807, 798, 790, 782, 774, 		//-9-0
	
	759, 756, 748, 739, 729, 720, 711, 701, 692, 682, 		//0-10
	672, 662, 653, 643, 633, 623, 613, 603, 593, 583, 		//11-20
	572, 562, 553, 543, 533, 523, 513, 503, 494, 484, 		//21-30
	
	474, 465, 456, 446, 437, 428, 419, 410, 401, 393, 		//31-40
	384, 376, 367, 359, 351, 343, 335, 328, 320, 313, 		//41-50
	306, 299, 292, 285, 278, 272, 265, 259, 252, 246, 		//51-60
	241, 235, 229, 224, 219, 213, 208, 203, 198, 193, 		//61-70
	189, 184, 179, 175, 171, 167, 163, 159, 155, 151  		//71-80
};



void saving_clock(void);


_UBYTE   _global_error_;
_UBYTE   msgflag;

// 定义led显示0123的状态的缓冲区下标   	// 紧急 停止 通话 开门 , 未用 通讯 报警 免扰
#define   Tongxun   	2
#define   Baojing   	1
#define   Mianrao		0



// 1 查询时 时钟的数据未化为二进制 还是BCD码
// 2 ACK   01  02
// 3 关机指令执行


