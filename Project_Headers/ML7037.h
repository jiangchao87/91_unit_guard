

#ifndef __ML7037_H_
  #define __ML7037_H_

#include "typedefine.h "

// 引脚定义
#define	 _PIN_PDN_7037		PTBD_PTBD7  			// 7037 下电脚 输入电平0有效
#define	 _PIN_DEN_7037		PTED_PTED4				// 
#define	 _PIN_EXCK_7037		PTED_PTED7				// 
#define	 _PIN_DIN_7037		PTED_PTED5				// 
#define	 _PIN_DOUT_7037		PTED_PTED6				// 

#define	 _7037_DATA_ENABLE_		0			// 引脚=0才可以操作7037
#define	 _7037_DATA_DISABLE		1

// 
typedef struct
{
	_UBYTE address;
	_UBYTE data;	
} _ML7037_REG_;


enum __ML7037_ADDRESS1__ 
{
	ADDRESS_ML7037_CR0 = 0X00,
	ADDRESS_ML7037_CR1 ,
	ADDRESS_ML7037_CR2 ,
	ADDRESS_ML7037_CR3 ,
	ADDRESS_ML7037_CR4 ,
	ADDRESS_ML7037_CR5 ,
	ADDRESS_ML7037_CR6 ,
	ADDRESS_ML7037_CR7 ,
	ADDRESS_ML7037_CR8 ,
	ADDRESS_ML7037_CR9 ,
	ADDRESS_ML7037_CR10 ,
	ADDRESS_ML7037_CR11 ,
	ADDRESS_ML7037_CR12 ,
	ADDRESS_ML7037_CR13 ,
	ADDRESS_ML7037_CR14 ,
	ADDRESS_ML7037_CR15 ,
	ADDRESS_ML7037_CR16 ,
	ADDRESS_ML7037_CR17 ,
	ADDRESS_ML7037_CR18 ,
	ADDRESS_ML7037_CR19 ,
	ADDRESS_ML7037_CR20 ,
	ADDRESS_ML7037_CR21 ,
	ADDRESS_ML7037_CR22 ,
	ADDRESS_ML7037_CR23 ,
	ADDRESS_ML7037_CR24 ,
	ADDRESS_ML7037_CR25 ,
	ADDRESS_ML7037_CR26 ,
	ADDRESS_ML7037_CR27,
	ADDRESS_ML7037_CR28,
	ADDRESS_ML7037_CR29,
	ADDRESS_ML7037_CR30,
	ADDRESS_ML7037_CR31 = 0X1F
};


enum __ML7037_ADDRESS2__  
{
	ADDRESS_ML7037_GPCR0 = 0X20,
	ADDRESS_ML7037_GPCR1,
	ADDRESS_ML7037_GPCR2,
	ADDRESS_ML7037_GPCR3,
	ADDRESS_ML7037_GPCR4,
	ADDRESS_ML7037_GPCR5,
	ADDRESS_ML7037_GPCR6,
	ADDRESS_ML7037_GPCR7,
	ADDRESS_ML7037_GPCR8 = 0X28
};



_ML7037_REG_ regs1_7037[] = 
{
	{ADDRESS_ML7037_CR1,	0x80},			// 
	
	{ADDRESS_ML7037_CR6,	0x0F},		//  0f00 = 1FC6
	{ADDRESS_ML7037_CR7,	0x00},
	{ADDRESS_ML7037_CR8,	0x1F},
	{ADDRESS_ML7037_CR9,	0xC6},
	{ADDRESS_ML7037_CR1,	0x80},
 
	{ADDRESS_ML7037_CR6,	0x0B},			// 0bb5 = 0009
	{ADDRESS_ML7037_CR7,	0xB5},
	{ADDRESS_ML7037_CR8,	0x00},
	{ADDRESS_ML7037_CR9,	0x09},
	{ADDRESS_ML7037_CR1,	0x80},
  
	{ADDRESS_ML7037_CR6,	0x0B},			// 0bb6 = 0012
	{ADDRESS_ML7037_CR7,	0xB6},
	{ADDRESS_ML7037_CR8,	0x00},
	{ADDRESS_ML7037_CR9,	0x12},
	{ADDRESS_ML7037_CR1,	0x80},
  
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f8b = 7000
	{ADDRESS_ML7037_CR7,	0x8B},
	{ADDRESS_ML7037_CR8,	0x70},
	{ADDRESS_ML7037_CR9,	0x00},
	{ADDRESS_ML7037_CR1,	0x80},
 
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f8c = 0800
	{ADDRESS_ML7037_CR7,	0x8C},
	{ADDRESS_ML7037_CR8,	0x08},
	{ADDRESS_ML7037_CR9,	0x00},
	{ADDRESS_ML7037_CR1,	0x80},
  
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f96 = 0001
	{ADDRESS_ML7037_CR7,	0x96},
	{ADDRESS_ML7037_CR8,	0x00},
	{ADDRESS_ML7037_CR9,	0x01},
	{ADDRESS_ML7037_CR1,	0x80}, 		// 查规格书没有发现这些位置和数据  屏蔽后好像没有影响 20110804郇
  
  //ALC 9db
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f11 = 0200
	{ADDRESS_ML7037_CR7,	0x11},
	{ADDRESS_ML7037_CR8,	0x02},
	{ADDRESS_ML7037_CR9,	0x00},
	{ADDRESS_ML7037_CR1,	0x80},
  
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f0a = 11e3
	{ADDRESS_ML7037_CR7,	0x0A},
	{ADDRESS_ML7037_CR8,	0x11},
	{ADDRESS_ML7037_CR9,	0xE3},
	{ADDRESS_ML7037_CR1,	0x80},
 
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f0b = 0caa
	{ADDRESS_ML7037_CR7,	0x0B},
	{ADDRESS_ML7037_CR8,	0x0C},
	{ADDRESS_ML7037_CR9,	0xAA},
	{ADDRESS_ML7037_CR1,	0x80},
 
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f0c = 0658
	{ADDRESS_ML7037_CR7,	0x0C},
	{ADDRESS_ML7037_CR8,	0x06},
	{ADDRESS_ML7037_CR9,	0x58},
	{ADDRESS_ML7037_CR1,	0x80},
   
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f14 = 05a6
	{ADDRESS_ML7037_CR7,	0x14},
	{ADDRESS_ML7037_CR8,	0x05},
	{ADDRESS_ML7037_CR9,	0xA6},
	{ADDRESS_ML7037_CR1,	0x80},
 
	{ADDRESS_ML7037_CR6,	0x0F},			// 0f15 = 016a
	{ADDRESS_ML7037_CR7,	0x15},
	{ADDRESS_ML7037_CR8,	0x01},   
	{ADDRESS_ML7037_CR9,	0x6A},
	{ADDRESS_ML7037_CR1,	0x80},
	
  //CR ==================================================
	{ADDRESS_ML7037_CR2,	0x10},			// 
	
	//{ADDRESS_ML7037_CR3,	0x00},			// 91_Unit_Panel<---->91终端    7037语音信号发送增益
	{ADDRESS_ML7037_CR3,	0x0E},			// 91_Unit_Panel<---->非可视分机    7037语音信号发送增益
	
	//{ADDRESS_ML7037_CR4,	0x03},			// 终端
	{ADDRESS_ML7037_CR4,	0x1e},			// 单元机     自己喇叭增益  声音大小调节 
	
	//{ADDRESS_ML7037_CR5,	0x08},			// 91_Unit_Panel<---->91终端    跟mic声音大小相关
	{ADDRESS_ML7037_CR5,	0x05},			// 91_Unit_Panel<---->非可视分机     跟mic声音大小相关
	
    //----------------------------------------------------
	//{ADDRESS_ML7037_CR11,	0x46},			// 线
	{ADDRESS_ML7037_CR11,	0x4c},			//
	
	//-----------------------------------------------------
	//{ADDRESS_ML7037_CR12,	0x4C},			//	声  回声消除启动 
	{ADDRESS_ML7037_CR12,	0x4c},			//
    //-----------------------------------------------------
	
	{ADDRESS_ML7037_CR13,	0x00},			//  
	//{ADDRESS_ML7037_CR13,	0x55},	            
	
	{ADDRESS_ML7037_CR14,	0x88},			//  
	//{ADDRESS_ML7037_CR14,	0x00},
  
	{ADDRESS_ML7037_CR16,	0x83},			//  
	{ADDRESS_ML7037_CR17,	0x82},			// 
	{ADDRESS_ML7037_CR18,	0x02},			// 
	
	{ADDRESS_ML7037_CR20,	0x00},			//  高频低频都不衰减  直通
 
	{ADDRESS_ML7037_CR0,	0x0D}			// 
	//{ADDRESS_ML7037_CR0,	0x0F}
}; 




//=======================================================================================
extern void  NopDelay(unsigned int num);
// 读7037一字节
// SOC sends address to and receives the status of the register from 7037
// @address: the address of the 7037 register 
_UBYTE read7037byte( _UBYTE address)
{                
	_UBYTE i, rev7037data = 0, snd7037data = 0;

	snd7037data = address |0x80;              	//set the MSB of address high

	_PIN_EXCK_7037 = 1;
	_PIN_DEN_7037 = _7037_DATA_ENABLE_;			// = 0  引脚=0才可以操作7037
	
	for( i=0; i<8; i++)
	{                                         	//send address to 7037 bit by bit through setting SOCPIN low or high
		_PIN_EXCK_7037 = 1;
		_PIN_DOUT_7037 = snd7037data & 0x80? 1:0;		
		_PIN_EXCK_7037 = 0;
		snd7037data <<= 1;
	} 
	_PIN_EXCK_7037 = 1; 
	for( i=0; i<8; i++)
	{                                                                           
		_PIN_EXCK_7037 = 0;
		rev7037data <<= 1;
		_PIN_EXCK_7037 =1;
		NopDelay(4);
		rev7037data |= _PIN_DIN_7037 == 1? 1:0;		
	} 
	_PIN_DEN_7037 = _7037_DATA_DISABLE;			// 1   引脚=1不可以操作7037
	_PIN_EXCK_7037 = 0;
	_PIN_DOUT_7037 = 0; 
	return ~rev7037data;                                
}


// 写7037一字节
// *SOC sends address and data to 7037
// *@address:the address of the 7037 register
// *@data:the data will be written into the 7037  
void write7037byte(_UBYTE address, _UBYTE data)
{
	_UBYTE i;
	_UINT snd7037data = 0;
	
	snd7037data = address;
	snd7037data = (snd7037data <<8)|data;

	_PIN_EXCK_7037 = 1;
	_PIN_DEN_7037 = _7037_DATA_ENABLE_;		// = 0   引脚=0才可以操作7037

	for( i=0; i<16; i++)
	{
		_PIN_EXCK_7037 =1;
		_PIN_DOUT_7037 = snd7037data & 0x8000? 1:0;
		_PIN_EXCK_7037 = 0;
		snd7037data <<= 1;
	}
	_PIN_EXCK_7037 =1;

	_PIN_DEN_7037 = _7037_DATA_DISABLE;		// 1   引脚=1不才可以操作7037
	_PIN_EXCK_7037 = 0;
	_PIN_DOUT_7037 = 0; 
}


// read the 7037 register after write it to check write correctly or not
// *write read check = WRC 
_SBYTE WRC7037byte( _ML7037_REG_ * pML7037REG, _UBYTE count)
{                
	_UBYTE i;

	for( i =0; i < count; i++)
	{
		write7037byte( pML7037REG[i].address, pML7037REG[i].data);
		if( ADDRESS_ML7037_CR1 == pML7037REG[i].address )
		{                                               //check the CR being written is CR1 or not
			time_counter.delay1ms = 0;
			while( (read7037byte(ADDRESS_ML7037_CR1) !=0) && (time_counter.delay1ms < 100))	// 填10刚刚上电失败  原来1000
			{
				__RESET_WATCHDOG();
			}
		}
		else
		{
			if( pML7037REG[i].data != read7037byte( pML7037REG[i].address))
			{                  							//to make sure the data get is the data send
				agn_int7037t = 10;								// 初始化7037失败，10秒后重新初始化
				return -1;
			}
		}
	}
	return 0;
}
  
  

// 初始化 7037 
void init7037(void)
{
	_PIN_DEN_7037	=	_7037_DATA_DISABLE;        		// DEN active low    
	
	_PIN_PDN_7037	=	0;                              // 7037 power down mode
	NopDelay(200);										// > 1us
	_PIN_PDN_7037	=	1;
	//NopDelay(200);
	time_counter.delay1ms = 0;
	while(time_counter.delay1ms<1000)
	{
		__RESET_WATCHDOG();
	}
	
	_PIN_PDN_7037	=	0;								// PDN_7037输入低电平  
	time_counter.delay1ms = 0;
	while(time_counter.delay1ms<1000)
	{
		__RESET_WATCHDOG();
	} 


	time_counter.delay1ms = 0;
	while( ( read7037byte(ADDRESS_ML7037_CR10)!=0x80 ) && (time_counter.delay1ms<500) )
	{
		__RESET_WATCHDOG();
	}
	if(time_counter.delay1ms>495)
		agn_int7037t = 10;								// 初始化7037失败，10秒后重新初始化

	if( WRC7037byte(regs1_7037, sizeof(regs1_7037)/sizeof(_ML7037_REG_) ) != 0 )
	{     							// 大概 14 
		;   
	}

	time_counter.delay1ms = 0;
	while( (read7037byte(ADDRESS_ML7037_CR10)!=0) && (time_counter.delay1ms < 500) )
	{
		__RESET_WATCHDOG();
	}
	if(time_counter.delay1ms>495)
		agn_int7037t = 10;								// 初始化7037失败，10秒后重新初始化
}


// 下电7037 
void power_down7037(void)
{
	_PIN_DEN_7037 = _7037_DATA_DISABLE;					// = 1   引脚=1才可以操作7037
	
	_PIN_PDN_7037 = 1;									// PDN_7037输入低电平  
	_PIN_EXCK_7037= 0;
	_PIN_DIN_7037 = 0;
	_PIN_DOUT_7037= 0;
}

#endif
