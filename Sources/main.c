
// 2014 0410 增加通话按键 PG5不可用，可能板子问题  PG3可用，调试OK

#include <hidef.h> 			/* for EnableInterrupts macro */
#include "derivative.h" 	/* include peripheral declarations */

//#include "ic_washer.h"
#include "ZDevice.h"          
       
//#include	"sfr_r82l.h"        
#include	"main.h"				//             
#include	"typedefine.h"			//        
#include	"ML7037.h"				// 
#include	"protocol.h"			// 

#include	"key.h"					//                               
//#include	"cstartdef.h" 			//        
#include	<stdlib.h>				// 
#include	<string.h>				//        
                          

//  run several num NOP instructions
// Parameters	 :_SBYTE num,represent to run num circles of NOP instruction 
void  NopDelay(unsigned int num)
{                                                              
	for(;num!=0;num--)                
	{                                                     
		asm("NOP");                                             
	}                                      
}                          


//=============================================================================
// 增加音量    数字电位器   CS=1时 UD上升沿
// @vol:actual parameter of this function is the volume differences 
void volume_increment( _UBYTE  vol)
{                             
	_UBYTE i; 
	if( 0 == vol) 						// 当UD=1时  CS下降沿
		return; 						//increment Mode
	_PIN_5465_UD_ = 0;					// 声音控制芯片脚
	_PIN_5465_CS_ = 1;                	// 声音控制 片选		                                                                           
	_PIN_5465_UD_ = 1;					// 声音控制芯片脚
	_PIN_5465_CS_ = 0;                 	//when the UD is high at the edge of CS from high to low 
	for( i=0; i<vol; i++)
	{ 
		_PIN_5465_UD_ = 0;
		_PIN_5465_UD_ = 1;             	//the vol increases at the edge of UD from low to high
	} 
	_PIN_5465_CS_ = 1; 					// 声音控制 片选	
}
   
   

// 降低音量   数字电位器   CS=1时 UD下降沿
// @vol:actual parameter of this function is the volume differences 
 void volume_decrement( _UBYTE  vol)
 {                                                                                 
	_UBYTE i;							// 当UD=0时  CS下降沿
    
	if( 0 == vol) 
		return;   						//decrement Mode
	_PIN_5465_UD_ = 1;					// 声音控制芯片脚
	_PIN_5465_CS_ = 1;					// 声音控制 片选	
	_PIN_5465_UD_ = 0;					// 声音控制芯片脚
	_PIN_5465_CS_ = 0;                 // when the UD is low at the edge of CS from high to low 
	for( i=0; i<vol-1; i++)
	{
		_PIN_5465_UD_ = 1;				// 声音控制芯片脚
		_PIN_5465_UD_ = 0;             	// the vol decreases at the edge of UD from low to high
	}
	_PIN_5465_UD_ = 1;					// 声音控制芯片脚
	_PIN_5465_CS_ = 1;					// 声音控制 片选	
}  

  
void volume_dwq_to( _UBYTE count)
{
	//_UBYTE ii;
	if( count > 32 ) 
		return; 
	volume_decrement(35);           	// make sure the volume has been decreased to zero 
	volume_increment(count);
} 

//=================================================================
// 看 安防 电源 等是否有改变
void check_change(void)
{                                          
	uchar i,ck;
	
	
	ck = 0;
	for(i=0;i<8;i++)
	{ 
		if(safe_bf[i]!=safe_zt[i])
			ck = 1; 
	} 
	for(i=0;i<8;i++) 						// 数据备份
		safe_bf[i] = safe_zt[i]; 
	if(ck==1)
		prepare_sndarm_pack(Safe_ztai);		// 安防数据 
	if(safe_jjbf!=safe_jj)
	{
		safe_jjbf = safe_jj;
		prepare_sndarm_pack(Safe_ztai);		// 安防数据 		
	}
	
	if(kScr_hlbz>200)						// 电源键检测 PWKCHK  连续200次相同才有效
	{
		if(kScr_hlbz==0)
			prepare_sndarm_pack(Pwer_offkey);
	}                                       // 
}
  


//==============================================================
// 根据ledzt[]转换为相应的cpu驱动引脚的输出
uchar led_do_acd(uchar no)
{
	switch(ledzt[no])
	{
		case 0:
		case 2:								// 关
			return 0; 
		case 1:								// 开
			return 1;  
		case 3:
			if(timebase<50)					// 指示灯闪烁时基
				return 0; 
			else
				return 1;  
	}
	return 0;
} 

// 驱动Led 要注意 与 led_do_acd(1) 的参数一致 0-7还是1-8
void  led_operate( void)
{	
	if( led_do_acd(Tongxun)==0 )
		_PIN_LED_TX = 1; 					// 通信
	else 
		_PIN_LED_TX = 0; 					// 0灯点亮  1熄灭

	if( led_do_acd(Mianrao)==0 )
		_PIN_LED_MR = 1; 					// 免[打]扰
	else 
		_PIN_LED_MR = 0; 

	if( led_do_acd(Baojing)==0 )
		_PIN_LED_JJAF = 1; 					// 紧急  安防                      
	else                                                             
		_PIN_LED_JJAF = 0;                                                        
}




//=========================================================================
// 复位Arm  reset the host 
void reset_host(void)
{
	_UBYTE i; 
	
	//SCI1C2_TE = 0;                               	//transmission disbale
	//SCI1C2_RE = 0;                                //receiving disable

	//SCI2C2_TE = 0;                                //transmission disbale
	//SCI2C2_RE = 0;                                //receiving disable

	for( i =0; i < 2; i++)                        	// RL_STATE_NUMBER
		INIT_LIST_HEAD( &receive_list[i]);

	for( i =0; i < SL_STATE_NUMBER; i++) 
		INIT_LIST_HEAD( &send_list[i]); 

	// TPM2SC_TOIE=0;
	// traic	= 0x03; 							//timer interrupt control register  priority level 3?

	//reset the host by reset pin
	

	msgflag = 0;
	time_counter.delay1ms = 0;
	time_counter.reset_delay = 0;
	time_counter.sensor_delay = 0;
	_global_error_ = 0;

	
	//volstate.change_times = MAX_CHANGE_VOL_TIMES + 3;
	volume_dwq_to(Volv_lvel[6][0]);
	
	
 	SCI1C2_TE = 0; 						// 关闭电话通讯
 	SCI1C2_RE = 1; 
	
 	SCI2C2_TE = 1;                                                                       
	SCI2C2_RE = 1;  
	
}

//=========================================================================
// 复位Arm  reset the host 
void reset_arm(void)
{	
	 
	_PIN_RESET_51 = _STATE_51_RESET;				// 复位 arm   0
	
	time_counter.delay1ms = 0;
	while( time_counter.delay1ms < 10)
	{
		__RESET_WATCHDOG();
	}
	 
	_PIN_RESET_51 = _STATE_51_NORMAL;				// 复位 arm	  1
	time_counter.delay1ms = 0;
	while( time_counter.delay1ms < 1000)
	{
		__RESET_WATCHDOG();
	}
	 
	_PIN_RESET_51 = _STATE_51_RESET;				// 0
}

//================================================================================
// 寄存器初始化
void sfrInit() 
{                        
	//enable watchdog
   	SOPT = 0XD3;						// 1101 0011
	//SPM
    SPMSC1 = 0X40;      
   	SPMSC2 = 0X30;          
	//ICG
    ICGC1 = 0XF8;
   	ICGC2 = 0X00;
  	while(!ICGS1_ERCS) 
	{
		__RESET_WATCHDOG();
	} 					       			//EXTERNAL CLOCK STABLE
	
	while(!ICGS1_LOCK) 
	{
		__RESET_WATCHDOG();
	}					        		//FLL LOCKED
	
	//SCI1  串口1                        
	SCI1C2 = 0x00;
	(void)(SCI1S1 == 0);				 /* Dummy read of the SCIS1 register to clear flags */
	(void)(SCI1D == 0);					 /* Dummy read of the SCI2D register to clear flags */
	SCI1S2 = 0x00; 
    SCI1BD = 104; 			       		//104--9600 BD
    SCI1C1 = 0X00;
	SCI1C2 = 0X2C;						// TIE TCIE RIE=1 ILIE, TE=1 RE=1 RW SBK
    SCI1C3 = 0X00;  
 
	//SCI2 串口2
	SCI2C2 = 0x00;
	(void)(SCI2S1 == 0);
	(void)(SCI2D == 0);
	SCI2S2 = 0x00; 
    SCI2BD = 104;        				//104--9600 BD
    SCI2C1 = 0X00;
	SCI2C2 = 0X2C;						// TIE TCIE RIE=1 ILIE, TE=1 RE=1 RW SBK
    SCI2C3 = 0X00;    
	
	//ADC
	APCTL1 = 0X00;
	APCTL2 = 0X40;						//enable channel 14
	
	//	APCTL3 = 0X00;
    ADC1CFG = 0X38;
	ADC1SC2 = 0X00;
	ADC1SC1 = 0X1F;
	
	//TPM
    TPM2SC = 0;							// 不能随便关闭
 	TPM2MOD = 0X03E7;
	TPM2SC = 0X4C; 

	//TPM2SC = 0x00;                       /* Stop and reset counter */
	//TPM2MOD = 0x03E7;                    /* Period value setting */
	//(void)(TPM2SC == 0);                 /* Overflow int. flag clearing (first part) */
	/* TPM2SC: TOF=0,TOIE=1,CPWMS=0,CLKSB=0,CLKSA=1,PS2=1,PS1=0,PS0=0 */
	//TPM2SC = 0x4C;                       /* Int. flag clearing (2nd part) and timer control register setting */
	/* ### Init_COP init code */
	//SRS = 0xFF;                          /* Clear WatchDog counter */

	//上拉电阻
    PTAPE = 0B11111111; 
    PTBPE = 0B00010000;
    PTCPE = 0B00100000;
    PTDPE = 0B11000010;
    PTEPE = 0B00100010;
    PTFPE = 0B00000001;
	PTGPE = 0B01100000;
	//慢速禁止
    PTASE = 0B00000000; 
    PTBSE = 0B00000000;
    PTCSE = 0B00000000;
    PTDSE = 0B00000000;
    PTESE = 0B00000000;
    PTFSE = 0B00000000;
    PTGSE = 0B00000000;
	//数据
    PTAD = 0B11111111;  
    PTBD = 0B00110000;
    PTCD = 0B01101000;
    PTDD = 0B01000000;
    PTED = 0B11110011;
    PTFD = 0B10000011;
    PTGD = 0B01100111;
	//方向
    PTADD = 0B00000000; 
    PTBDD = 0B11101111; 
    PTCDD = 0B11011111; 
    PTDDD = 0B00011111;
    PTEDD = 0B11011101;
    PTFDD = 0B11111110;
    PTGDD = 0B10010111; 
	NopDelay(4);
}

 
//===================================================================
// ICG interrupt service function 
/* void interrupt 4 ICG_LOCK_FAILURE(void)
{
    ICGC1 = 0XF8;
   	ICGC2 = 0X00;
  	while(!ICGS1_ERCS) {__RESET_WATCHDOG();}        //EXTERNAL CLOCK STABLE
	while(!ICGS1_LOCK) {__RESET_WATCHDOG();}        //FLL LOCKED
}*/


// 定时器中断  1毫秒
void interrupt 14 T2(void)
{
	//_UBYTE i;
	TPM2SC &= 0x7F;						//clear Timer overflow flag
    TPM2MOD = 0x03E7;					//Timer = 999

	time_counter.delay1ms < 2000? time_counter.delay1ms++:0;
	
	ms10++;								// 10ms 
	
/*
	TPM2SC &= 0x7F;						//clear Timer overflow flag
    TPM2MOD = 0x03E7;					//Timer = 999

	udelay++;
	if(udelay == 1000)
		mdelay++;
	if(mdelay == 1000)
		sdelay++;
	
*/
}


//=====================================================
// SCI1 interrupt service function 
void interrupt 16 SCI1_ERROR(void)
{
	uchar i;
	i = SCI1S1;
	return;
}
  


// 串口1接收 
interrupt 17 void SCI1_R(void)
{ 
	/* CY8C22545-24AXI send data to mc9s08ac60 via UART1*/
	/* RX data saved in uart1_rxbuff[] */ 

	SCI1S1 &= 0xDF;
	if((rx_ready == 1)&&(rx_readout == 0))
		return;
	if((p_rx < 4) && (rx_ready == 0))
	{
		rx_byte = SCI1D;
	    uart1_rx_buff[p_rx++] = rx_byte;
	    rx_ready = 0;
	}
	if(p_rx == 4)
	{
	    p_rx = 0;
	    rx_readout = 0;
	    rx_ready = 1;
	}
	return;
}

// 串口1发送 
//interrupt 18 void SCI1_T(void)
//{ 
//}

//SCI2 interrupt service function 
interrupt 19 void SCI2_ERROR(void)
{
	uchar i;
	i = SCI2S1;
	return; 
} 



//==================================================
// 串口2接收 ------- Arm
interrupt 20 void SCI2_R(void)
{
	_UBYTE  rsls = 0;	
	SCI2S1 &= 0xDF;									// TDRE TC RDRF IDLE OR NF FE PF
	
	rsls = SCI2D;
	if(rsvover>0)									//receive done rsvover = 1
		return;
	if(rsls==0xf5)									//data header double 0xF5
	{
		if(headf5bz>0)
			rsv_p = 0;								// 指针归零
		headf5bz = 1; 								//data header 0xF5 flag headf5bz = 1
	}
	else
		headf5bz = 0;
	
	if(rsv_p<30)
	{
		rsvbufi[rsv_p] = rsls;						//data body 
		rsv_p++;
	}
	if( (rsvbufi[1]+2)==rsv_p )						//data length = rsvbuffi[1]+2(header)
		rsvover = 1;								//receive done
}

// 串口2发送  ------- Arm
interrupt 21 void SCI2_T(void)
{
	uchar i;
	SCI2S1 &= 0xBF;									// TDRE TC RDRF IDLE OR NF FE PF
	i = Sndbuf[snd_frm][1];
	if(snd_p<=i+1)									// F5 F5 08 00 00 00 00 01 01 08 FD
	{
		SCI2D = Sndbuf[snd_frm][snd_p];	
		snd_p++;
	}
	else
	{ 
		SCI2C2_TE = 0;
		SCI2C2_TCIE = 0;
		snd_p = 0; 
		snd_frm = 10;								// 表示可以处理下一个帧		
	} 
}


static unsigned char calculate_check_sum(int len,unsigned char* buf)
{
	unsigned char check_sum = 0;
	int i;
	for(i = 0; i < len; i++)
	{
		check_sum += buf[i];
	}
	return check_sum;
}


int send_key_value(uchar key_value)
{
	key_vl = key_value;
	if( (key_vl>0))
	{
		prepare_sndarm_pack(Key_4);						//save key value to SCI1 send buffer
		key_vlBF = key_vl;
		return 0;
	}
	else
		return -1;
}
 
void  key_do(void)
{
	uint temp_key = 0;
	uchar key_value = 0;
	//uchar key_code[4] = {0};
	
	if(rx_ready)
	{
		memcpy(key_code,uart1_rx_buff,4);
		memset(uart1_rx_buff,0,4);
		rx_readout = 1;
		rx_ready = 0;
	}
	else
	{
		rx_readout = 0;
		return;													//memory copy failed
	}
	if((key_code[0] != 0x5A)||
		(calculate_check_sum(2,&key_code[1]) != key_code[3]))
		return;													//invalid data
	else
	{
		temp_key = key_code[1]<<8;
		temp_key = temp_key | key_code[2];
		switch (temp_key)
		{
			case RCV_KEY_DUAL:
				key_value = UH_KEY_DUAL;
				break;
			case RCV_KEY_0:
				key_value = UH_KEY_0;
				break;
			case RCV_KEY_1:
				key_value = UH_KEY_1;
				break;
			case RCV_KEY_2:
				key_value = UH_KEY_2;
				break;
			case RCV_KEY_3:
				key_value = UH_KEY_3;
				break;
			case RCV_KEY_4:
				key_value = UH_KEY_4;
				break;
			case RCV_KEY_5:
				key_value = UH_KEY_5;
				break;
			case RCV_KEY_6:
				key_value = UH_KEY_6;
				break;
			case RCV_KEY_7:
				key_value = UH_KEY_7;
				break;
			case RCV_KEY_8:
				key_value = UH_KEY_8;
				break;
			case RCV_KEY_9:
				key_value = UH_KEY_9;
				break;
			case RCV_KEY_UP:
				key_value = UH_KEY_UP;
				break;
			case RCV_KEY_DOWN:
				key_value = UH_KEY_DOWN;
				break;
			case RCV_KEY_CANCEL:
				key_value = UH_KEY_CANCEL;
				break;
			case RCV_KEY_OK:
				key_value = UH_KEY_OK;
				break;
			case RCV_KEY_CENTER:
				key_value = UH_KEY_CENTER;
				break;
			case RCV_KEY_PASSWORD:
				key_value = UH_KEY_PASSWORD;
				break;
			default:
				key_value = UH_KEY_RELEASE;
				break;
		}
		if(send_key_value(key_value) != 0)
			return;						//invalid key or error!
	}
}


//======================================================================================
//======================================================================================
//======================================================================================
void main(void) 
{
 	uchar m0; 
	
 	//unsigned int mi;
	EnableInterrupts; 								// enable interrupts  
	// include your code here  
	//asm ("fclr I");								//disable the response of MCU
	DisableInterrupts;        
	
	sfrInit();										// 初始化cpu
	
	EnableInterrupts;
	//asm ("fset I");           					//enable the response of MCU

	reset_host();									// 初始化配置自己的cpu
	
	init7037(); 									// 初始化回声 消除芯片
	
	
	// F5 F5 08 00 00 00 00 01 01 08 FD
	snd_frm = 10;
	Tel_frm = 10;
	
	// F5 F5 08 00 00 00 00 04 55 59 FD  挂机
	Sndbuf[0][0] = 0xf5;
	Sndbuf[0][1] = 0x08;
	Sndbuf[0][6] = 0x04;
	Sndbuf[0][7] = 0x55;
	Sndbuf[0][8] = 0x59;
	Sndbuf[0][9] = 0xfd;
	
	
	volume_dwq_to(Volv_lvel[6][0]);						//set default volume
	
	// _PIN_REL_ALL = 1;
	fsyc = 0;											// 刚上电 发送一次串口数据
	rst_3s = 0;
	
	for(;;) 
	{  
		__RESET_WATCHDOG(); 								// feeds the dog   看门狗 要 测试一下 
       


		if(ms10>=10)										// 10 ms
		{
			ms10 = 0;
			//if(tel_2s<250)								// 2秒内收到arm拨号只拨1次
			//	tel_2s++;
			// Sndbuf[x][6]报文序号  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]发送的123次数 Sndbuf[x][snd200ms]计时200ms  
	
			key_do();										// 按键处理	
			
			for(m0=0;m0<5;m0++)								// 发送时基
			{
				if(Sndbuf[m0][snd200ms]<250)
					Sndbuf[m0][snd200ms]++;
				if(TxTelbuf[m0][snd200ms]<250)          
					TxTelbuf[m0][snd200ms]++;
			}
			if(timebase<100)
				timebase++;								// led指示灯闪烁时基 
			else
				timebase = 0;            
			
			sec1s++; 
			if(sec1s>=100)								// 1s                              
			{				
				sec1s = 0;                       
				min60m++;
				
				
				if(min60m>60)                       
					min60m = 0;                           
				
				if(agn_int7037t>0)
					agn_int7037t--;						// 初始化7037失败，10秒后重新初始化
				if(agn_int7037t==1)
				{
					agn_int7037t = 0;
					init7037(); 						// 初始化回声 消除芯片					
				}
				
				if(rst_3s<200)							// 上点3秒 重起arm一次
					rst_3s++;
				rst_armt++;				
				if( (rst_armt%15)==0 )					// 收不到arm数据后每60秒
				{
					prepare_sndarm_pack(Safe_ztai);		// 报告一次安防状态
					// init7037(); 						// 初始化回声 消除芯片  %%%%%%%%%%%%%% ，每15秒初始化一次7037 测试初始化函数用，正常要屏蔽
				}
				if(rst_armt>=80)						// 收不到arm数据表示arm不正常 重新启动
				{
					rst_armt = 0;
					m0 = 5;
					while(m0>1)							// 停止喂狗 等待重新启动
						; 
				}
				
				//rst_3s = 250;
				if(rst_3s==3)							// 收不到arm数据表示arm不正常 重新启动
				{
					rst_armt = 0;
					rst_3s = 4;
					reset_arm();						// 复位arm		
				} 
            }        
		}  
		
		read_afkey();									// 读安防接口的状态 
		//read_Krst_scrn();								// 复位和校准屏幕
		
		check_change();									// 检查温度 安防等数据是否有变化
		
		respond2Host();									// arm 接收转存并处理
		send2host();		  							// arm 调配发送的帧  启动发送
		
		led_operate();   								// 指示灯驱动  
	}   
}

// 2012 0510 将原来5分钟通信失败重起arm 改为1分钟   原来每1分钟握手1次改为15秒
//=============================== End ================================================
//
//           _________                               _____________
//          |         |                mic      13,14|            |
//          |         |               ------------>--|->-         |
//          |  5000   |                              |   |        |
//  Arm 网线|         |	                          8  |   |        |
// ---------|       15|---<---------------<-------<--|<--         |
// ---------|         |                     |        |            |
//          |         |                     |        |    7037    |
//          |         |                     |    6,7 |            |  9
//          |       12|-->------------------|----->--|----->------|------>喇叭
//          |         |        |            |        |            |
//           ---------         |            |         ------------           
//                             |            | 
//                             |            |
//                             |           ___
//                             |           |R|       J17  J24
//                             |           | |        _____
//                             |            -        |     | 非可视分机
//                             |            |-----<--|     |
//                             |              MIN_IN |     | 门前机
//                             |                     |     |
//                             |                     |     |
//                             |              Lout1  |     |
//                             |------------------->-|     |
//                                                   |     |
//                                                    -----  
//    
//
//===================================================================================
