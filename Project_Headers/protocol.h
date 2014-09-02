
#ifndef __H_PROTOCOL__
  #define __H_PROTOCOL__
  #include "typedefine.h"
  #include "list.h"

  #define   snd321     29
  #define   snd200ms   30

//=====================================
// 从机-->Arm  指令
#define    Answer_zt      0x02				// 查询返回 
#define    ACK_slvtx	  0x03 				// ACK
#define    Safe_ztai      0x09				// 安防
#define    Tmp_zt         0x0A				// 温度
#define    Tlph_com       0x0F				// 电话
#define    Rout_zt        0x10				// 线路
#define    Dtmf_com       0x11 				// DTMF
#define    Pwer_offkey    0x12 				// 按下关机开关
#define    Check_Screen   0x17 				//
#define    Key_4		  0x18 				//
 
// Arm-->从机  指令
#define	   HOST_QUERY			0X01		// 查询
#define	   HOST_ACK				0x03		// 主机反馈ACK
#define	   HOST_SHUTDOWN		0x04		// 关机
#define    HOST_SETTIME			0x05		// 设定时间
#define	   HOST_VOLUME			0x06		// 音量
#define	   HOST_Led				0x07		// 指示灯开关
#define	   HOST_Rly				0x08		// 继电器开关
#define	   HOST_TakPhn			0x0B		// 摘机
#define	   HOST_Hangphn			0x0C		// 挂机
#define	   HOST_Dial			0x0D		// 拨号
#define	   HOST_Dtmf			0x0E		// 发出DTMF
#define	   HOST_RstArm			0x13		// 复位arm
#define	   HOST_EchoCoOn		0x14		// 回声消除开
#define	   HOST_EchoCOff		0x15		// 回声消除关闭
#define	   HOST_DvcType			0x16		// 呼叫设备类型

#define	   COMMAND_7037			0x21		// 


extern uchar  volume;	
void volume_dwq_to( _UBYTE count);
void reset_host(void);
void get_current_time(void);

//=====================================================================
// 电话模块发给单片机
#define		TEL_answer					0X52			// 电话模块回复
#define		TEL_ack						0X53			// ACK
#define		TEL_come_call				0X57			// 来电
#define		TEL_dtmf					0X58			// DTMF
#define		TEL_routstat				0X59			// 线路状态

// 单片机发给电话模块
#define		CTEL_ask					0X51			// 查询4
#define		CTEL_open					0X54			// 摘机
#define		CTEL_halloff				0X55			// 挂机
#define		CTEL_dial					0X56			// 拨号
#define     CDtmf_dial                  0x5f            // DTMF拨号


//=============================================================
// 调配发送的帧  启动发送
void  send2host(void)
{
	uchar i;
	if(snd_frm<6)
		return;
	// Sndbuf[x][6]报文序号  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]发送的123次数 Sndbuf[x][snd200ms]计时200ms 
	for(i=0;i<5;i++)
	{
		if(Sndbuf[i][snd321]>0)
		{
			if(Sndbuf[i][snd200ms]>20)
			{
				SCI2C2_TE = 1;	   				// 使能串口
				SCI2C2_TCIE = 1;				// 传送结束 中断使能
				
				Sndbuf[i][snd321]--;
				Sndbuf[i][snd200ms] = 0;
				snd_frm = i; 
				snd_p = 0;
				
				SCI2D = 0xF5;		  			// pChar[ (*pPos)++ ];		
				return;
			}
		}
	}
}




// 准备 应答/发送 数据包 
void  prepare_sndarm_pack(uchar rg)
{
	uchar i,j,sum; 
	uchar t1,t2; 
	uint yy;
	// Sndbuf[x][6]报文序号  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]发送的123次数 Sndbuf[x][snd200ms]计时200ms  
	j = 10;
	for(i=0;i<5;i++)
	{
		if(Sndbuf[i][snd321]==0)
		{
			j = i;
			break;
		}
	}
	if(j==10)										// 所有发送缓冲区都满
		return;
	
	Sndbuf[j][0] = 0xF5;							// header
	Sndbuf[j][1] = 0x00;							// length
	Sndbuf[j][2] = 0x00;							// reserved
	Sndbuf[j][3] = 0x00;							// reserved
	Sndbuf[j][4] = 0x00;							// reserved
	Sndbuf[j][5] = 0x00;							// reserved
	snd_BWenNo++;
	if(snd_BWenNo==0)
		snd_BWenNo = 1;
	Sndbuf[j][6] = snd_BWenNo;						// 报文序号 
	Sndbuf[j][7] = rg;								// 指令   
	
	switch(rg)
	{
		case  Answer_zt:            				// 查询返回 
			Sndbuf[j][1] = 0x1B;					// 长度
			Sndbuf[j][6] = rsv_ack;					// 收到的arm的序号
			Sndbuf[j][8] = rut_zt;					// 线路状态
			Sndbuf[j][9] = ad_tmp;					// 室内温度
			Sndbuf[j][10] = tlphn_zt;				// 电话分机状态 
			
			yy = 2000; 
			yy += (T1302_year>>4)*10; 
			yy += (T1302_year&0x0f); 			 
			Sndbuf[j][11] = yy&0xff;			   	// 年月日 时分秒 
			Sndbuf[j][12] = yy>>8;		
			Sndbuf[j][13] = T1302_mth;		 		
			Sndbuf[j][14] = T1302_day;			 	
			Sndbuf[j][15] = T1302_hour;			
			Sndbuf[j][16] = T1302_min;			 	
			Sndbuf[j][17] = T1302_sec;				// 安防状态
			for(i=0;i<5;i++)
			{
				t1 = Sndbuf[j][13+i]>>4;
				t2 = Sndbuf[j][13+i]&0x0f;
				Sndbuf[j][13+i] = t1*10 + t2;		// 将时间BCD码转换为16进制
			}
			for(i=0;i<8;i++)
				Sndbuf[j][18+i] = safe_zt[i]; 
			Sndbuf[j][26] = safe_jj; 				// 紧急引脚
			Sndbuf[j][snd321] = 1;					// 可以发送1次
			break;   
		
		case ACK_slvtx:								// ACK
			Sndbuf[j][1] = 0x09;	
			Sndbuf[j][6] = rsv_ack;					// 收到的arm的序号
			Sndbuf[j][8] = 2;						// 2表示错误
			Sndbuf[j][snd321] = 1;					// 可以发送1次
			break; 
		
		case  Safe_ztai:              				// 安防  
			Sndbuf[j][1] = 0x11;					// 长度
			for(i=0;i<8;i++)
				Sndbuf[j][8+i] = safe_zt[i];  
			Sndbuf[j][16] = safe_jj; 				// 紧急按钮
			
			Sndbuf[j][snd321] = 1;					// 可以发送3次
			break; 
		
		case Pwer_offkey:							// 按下电源按键
		case Check_Screen:							// 检验屏幕
			Sndbuf[j][1] = 0x08;	  
			Sndbuf[j][snd321] = 3;					// 可以发送3次
			break;
		case TOUCH_FOUR_KEY:
			Sndbuf[j][1] = 0x0A;					// length = (4+1+1+2+1+1)=10
			Sndbuf[j][8] = key_value>>8;			// MSB
			Sndbuf[j][9] = key_value&0xFF; 			// LSB
			Sndbuf[j][snd321] = 3;					// 可以发送3次			
			break;
		
		case TOUCH_KEY:
			Sndbuf[j][1] = 0x0A;					// length = (4+1+1+2+1+1)=10
			Sndbuf[j][8] = key_code[1];				// MSB
			Sndbuf[j][9] = key_code[2]; 			// LSB
			Sndbuf[j][snd321] = 3;					// 可以发送3次			
			break;
	}
	if(Sndbuf[j][1]>0)
	{ 
		Sndbuf[j][snd200ms] = 240;					// 随时可以发送
		sum = 0;
		for(i=1;i<Sndbuf[j][1];i++)					// F5 F5 08 00 00 00 00 01 01 08 FD
			sum ^= Sndbuf[j][i];
		i = Sndbuf[j][1];
		Sndbuf[j][i] = sum;
		Sndbuf[j][i+1] = 0xfd;
	}
}
 
 
 
//-----------------------------------------------------------------------------------------
// 将接收的led状态转换为0 1 2 3，以备存入ledzt[]
uchar led_rx_onoff(uchar rz,uchar no)
{
	uchar i;
	i = no*2;							// 0  1 2 3
	return (rz>>i)&0x03;
}

// 应答主机  接收转存并处理
void  respond2Host(void)
{
	uchar i,len,sum; 
	if(rsvover==0)
		return; 
	rsv_p = 0;
	for(i=0;i<32;i++)  
		rsvbufm[i] = rsvbufi[i];  
		
	for(i=0;i<32;i++) 
		rsvbufi[i] = 0;  								//  清接收缓冲区
		
	rsvover = 0;
	
	len = rsvbufm[1];									// F5 F5 08 00 00 00 00 01 01 08 FD
	sum = 0;
	for(i=1;i<len;i++)
		sum ^= rsvbufm[i];
	if( (sum!=rsvbufm[len])||(rsvbufm[len+1]!=0xfd) )
		return;
	
	if(bw_armbf==rsvbufm[6])							// 报文序号相同的不处理
		return;
	bw_armbf = rsvbufm[6];
	
	rst_armt = 1;										// 收到arm数据表示arm工作正常，不用重新启动  =0会很快发送安防数据
	
	rsv_ack =  rsvbufm[6];

	switch( rsvbufm[7] )
	{
		case HOST_QUERY:								// 查询 
			prepare_sndarm_pack(Answer_zt);				// 准备查询的发送包
			break;
			
		case HOST_ACK:									// 主机反馈ACK
			for(i=0;i<5;i++)							// F5 F5 09 00 00 00 00 01 03 01 0A FD
			{
				if(Sndbuf[i][6]==rsvbufm[6])			// 报文序号 
					Sndbuf[i][snd321] = 0; 
			}	// Sndbuf[x][snd321]发送的123次数 Sndbuf[x][snd200ms]计时200ms   
			break; 
		
		case HOST_SHUTDOWN:								// 关机
			prepare_sndarm_pack(Answer_zt);				// F5 F5 08 00 00 00 00 01 04 0D FD
			rst_armt = 295;								// 如果按键断开又闭合了，5秒后重新启动Arm
			uuuu = 5;
			break;
		
		//-----------------------------------------------------
		case HOST_VOLUME:								// 音量 F5 F5 09 00 00 00 00 01 06 02 0C FD
			prepare_sndarm_pack(Answer_zt);				//      F5 F5 09 00 00 00 00 01 06 00 0E FD 
			volume = rsvbufm[8];						// 
			if(volume>10)
				volume = 10;
			if( (volume==0)&&(volumeBF!=volume) )
				init7037(); 	
			volumeBF = volume;
			volume_dwq_to(Volv_lvel[volume][0]);	//      F5 F5 09 00 00 00 00 01 06 0E 00 FD 
			break;
			
		case HOST_DvcType:								// 呼叫设备类型  类型不同， MIC增益不同
			i = rsvbufm[8];
			prepare_sndarm_pack(Answer_zt);				// ACK 
			break;
		//-----------------------------------------------------------
		case HOST_Led:									// 指示灯开关
			prepare_sndarm_pack(Answer_zt);				// ACK   F5 F5 0C 00 00 00 00 01 07 01 01 00 0F 05 FD
			for(i=0;i<4;i++)
			{
				sum = led_rx_onoff(rsvbufm[10],i);		 // 紧急 停止 通话 开门 , 未用 通讯 报警 免扰
				if( sum > 0 )
					ledzt[4+i] = sum;
				sum = led_rx_onoff(rsvbufm[11],i);
				if( sum > 0 )
					ledzt[i] = sum;
			}
			
			//if(ledzt[Baojing]==3)
			{
				//volume_dwq_to(Volv_lvel[10][0]);	//      F5 F5 09 00 00 00 00 01 06 0E 00 FD 
				//write7037byte(ADDRESS_ML7037_CR4,Volv_lvel[10][1]);
			}
			break;
		
		//-----------------------------------------------
		case HOST_RstArm:								// 复位arm
			prepare_sndarm_pack(Answer_zt);				// ACK 
			reset_host();
			break;
		
		case HOST_EchoCoOn:								// 回声消除开
			prepare_sndarm_pack(Answer_zt);				// ACK 
			write7037byte(ADDRESS_ML7037_CR12,0x4C);
			break;
		
		case HOST_EchoCOff:								// 回声消除关闭
			prepare_sndarm_pack(Answer_zt);				// ACK 
			write7037byte(ADDRESS_ML7037_CR12,0x04);
			break;
		
		case COMMAND_7037:								// 
			switch(rsvbufm[8])
			{				
			 	case 3:									// POWER_DOWN_7037:
					power_down7037();
					break;
			 	case 4:									// INITIAL_7037:
					init7037();
					break; 
			}
			break; 
	} 
}
 


#endif



