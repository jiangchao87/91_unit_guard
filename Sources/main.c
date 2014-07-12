
// 2014 0410 ����ͨ������ PG5�����ã����ܰ�������  PG3���ã�����OK

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
// ��������    ���ֵ�λ��   CS=1ʱ UD������
// @vol:actual parameter of this function is the volume differences 
void volume_increment( _UBYTE  vol)
{                             
	_UBYTE i; 
	if( 0 == vol) 						// ��UD=1ʱ  CS�½���
		return; 						//increment Mode
	_PIN_5465_UD_ = 0;					// ��������оƬ��
	_PIN_5465_CS_ = 1;                	// �������� Ƭѡ		                                                                           
	_PIN_5465_UD_ = 1;					// ��������оƬ��
	_PIN_5465_CS_ = 0;                 	//when the UD is high at the edge of CS from high to low 
	for( i=0; i<vol; i++)
	{ 
		_PIN_5465_UD_ = 0;
		_PIN_5465_UD_ = 1;             	//the vol increases at the edge of UD from low to high
	} 
	_PIN_5465_CS_ = 1; 					// �������� Ƭѡ	
}
   
   

// ��������   ���ֵ�λ��   CS=1ʱ UD�½���
// @vol:actual parameter of this function is the volume differences 
 void volume_decrement( _UBYTE  vol)
 {                                                                                 
	_UBYTE i;							// ��UD=0ʱ  CS�½���
    
	if( 0 == vol) 
		return;   						//decrement Mode
	_PIN_5465_UD_ = 1;					// ��������оƬ��
	_PIN_5465_CS_ = 1;					// �������� Ƭѡ	
	_PIN_5465_UD_ = 0;					// ��������оƬ��
	_PIN_5465_CS_ = 0;                 // when the UD is low at the edge of CS from high to low 
	for( i=0; i<vol-1; i++)
	{
		_PIN_5465_UD_ = 1;				// ��������оƬ��
		_PIN_5465_UD_ = 0;             	// the vol decreases at the edge of UD from low to high
	}
	_PIN_5465_UD_ = 1;					// ��������оƬ��
	_PIN_5465_CS_ = 1;					// �������� Ƭѡ	
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
// �� ���� ��Դ ���Ƿ��иı�
void check_change(void)
{                                          
	uchar i,ck;
	
	
	ck = 0;
	for(i=0;i<8;i++)
	{ 
		if(safe_bf[i]!=safe_zt[i])
			ck = 1; 
	} 
	for(i=0;i<8;i++) 						// ���ݱ���
		safe_bf[i] = safe_zt[i]; 
	if(ck==1)
		prepare_sndarm_pack(Safe_ztai);		// �������� 
	if(safe_jjbf!=safe_jj)
	{
		safe_jjbf = safe_jj;
		prepare_sndarm_pack(Safe_ztai);		// �������� 		
	}
	
	if(kScr_hlbz>200)						// ��Դ����� PWKCHK  ����200����ͬ����Ч
	{
		if(kScr_hlbz==0)
			prepare_sndarm_pack(Pwer_offkey);
	}                                       // 
}
  


//==============================================================
// ����ledzt[]ת��Ϊ��Ӧ��cpu�������ŵ����
uchar led_do_acd(uchar no)
{
	switch(ledzt[no])
	{
		case 0:
		case 2:								// ��
			return 0; 
		case 1:								// ��
			return 1;  
		case 3:
			if(timebase<50)					// ָʾ����˸ʱ��
				return 0; 
			else
				return 1;  
	}
	return 0;
} 

// ����Led Ҫע�� �� led_do_acd(1) �Ĳ���һ�� 0-7����1-8
void  led_operate( void)
{	
	if( led_do_acd(Tongxun)==0 )
		_PIN_LED_TX = 1; 					// ͨ��
	else 
		_PIN_LED_TX = 0; 					// 0�Ƶ���  1Ϩ��

	if( led_do_acd(Mianrao)==0 )
		_PIN_LED_MR = 1; 					// ��[��]��
	else 
		_PIN_LED_MR = 0; 

	if( led_do_acd(Baojing)==0 )
		_PIN_LED_JJAF = 1; 					// ����  ����                      
	else                                                             
		_PIN_LED_JJAF = 0;                                                        
}




//=========================================================================
// ��λArm  reset the host 
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
	
 	SCI1C2_TE = 0; 						// �رյ绰ͨѶ
	SCI1C2_RE = 0; 
	
 	SCI2C2_TE = 1;                                                                       
	SCI2C2_RE = 1;  	
}

//=========================================================================
// ��λArm  reset the host 
void reset_arm(void)
{	
	 
	_PIN_RESET_51 = _STATE_51_RESET;				// ��λ arm   0
	
	time_counter.delay1ms = 0;
	while( time_counter.delay1ms < 10)
	{
		__RESET_WATCHDOG();
	}
	 
	_PIN_RESET_51 = _STATE_51_NORMAL;				// ��λ arm	  1
	time_counter.delay1ms = 0;
	while( time_counter.delay1ms < 1000)
	{
		__RESET_WATCHDOG();
	}
	 
	_PIN_RESET_51 = _STATE_51_RESET;				// 0
}

//================================================================================
// �Ĵ�����ʼ��
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
	
	//SCI1  ����1                        
	SCI1C2 = 0x00;
	(void)(SCI1S1 == 0);				 /* Dummy read of the SCIS1 register to clear flags */
	(void)(SCI1D == 0);					 /* Dummy read of the SCI2D register to clear flags */
	SCI1S2 = 0x00; 
    SCI1BD = 104; 			       		//52--19200 BD
    SCI1C1 = 0X00;
	SCI1C2 = 0X2C;						// TIE TCIE RIE ILIE, TE RE RW SBK
    SCI1C3 = 0X00;  
 
	//SCI2 ����2
	SCI2C2 = 0x00;
	(void)(SCI2S1 == 0);
	(void)(SCI2D == 0);
	SCI2S2 = 0x00; 
    SCI2BD = 104;        				//52--19200 BD
    SCI2C1 = 0X00;
	SCI2C2 = 0X2C;
    SCI2C3 = 0X00;    
	
	//ADC
	APCTL1 = 0X00;
	APCTL2 = 0X40;						//enable channel 14
	
	//	APCTL3 = 0X00;
    ADC1CFG = 0X38;
	ADC1SC2 = 0X00;
	ADC1SC1 = 0X1F;
	
	//TPM
    TPM2SC = 0;							// �������ر�
 	TPM2MOD = 0X03E7;
	TPM2SC = 0X4C; 

	//TPM2SC = 0x00;                       /* Stop and reset counter */
	//TPM2MOD = 0x03E7;                    /* Period value setting */
	//(void)(TPM2SC == 0);                 /* Overflow int. flag clearing (first part) */
	/* TPM2SC: TOF=0,TOIE=1,CPWMS=0,CLKSB=0,CLKSA=1,PS2=1,PS1=0,PS0=0 */
	//TPM2SC = 0x4C;                       /* Int. flag clearing (2nd part) and timer control register setting */
	/* ### Init_COP init code */
	//SRS = 0xFF;                          /* Clear WatchDog counter */

	//��������
    PTAPE = 0B11111111; 
    PTBPE = 0B00010000;
    PTCPE = 0B00100000;
    PTDPE = 0B11000010;
    PTEPE = 0B00100010;
    PTFPE = 0B00000001;
	PTGPE = 0B01100000;
	//���ٽ�ֹ
    PTASE = 0B00000000; 
    PTBSE = 0B00000000;
    PTCSE = 0B00000000;
    PTDSE = 0B00000000;
    PTESE = 0B00000000;
    PTFSE = 0B00000000;
    PTGSE = 0B00000000;
	//����
    PTAD = 0B11111111;  
    PTBD = 0B00110000;
    PTCD = 0B01101000;
    PTDD = 0B01000000;
    PTED = 0B11110011;
    PTFD = 0B10000011;
    PTGD = 0B01100111;
	//����
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


// ��ʱ���ж�  1����
void interrupt 14 T2(void)
{
	//_UBYTE i;
	TPM2SC &= 0X7F;						//read tpm2sc first,then write it
    TPM2MOD = 0X03E7;

	time_counter.delay1ms < 2000? time_counter.delay1ms++:0;
	
	ms10++;								// 10ms 
}


//=====================================================
// SCI1 interrupt service function 
void interrupt 16 SCI1_ERROR(void)
{
	uchar i;
	i = SCI1S1;
	return;
}
  

// ����1���� ------- �绰
interrupt 17 void SCI1_R(void)
{ 
	//__uart1_receive__();
	_UBYTE  rsls = 0;	
	SCI1S1 &= 0xDF; 								// TDRE TC RDRF IDLE OR NF FE PF
	rsls = SCI1D;
	if(RxTelov>0)
		return;
	
	if(rsls==0xf5)
	{
		if(RxTelf5bz>0)
			RxTel_p = 0;							// ָ�����
		RxTelf5bz = 1;
	}
	else
		RxTelf5bz = 0;
	
	if(RxTel_p<30)
	{
		RxTelbufi[RxTel_p] = rsls;	
		if(RxTel_p<32)
			RxTel_p++;
	}
	if( (RxTelbufi[1]+2)==RxTel_p)					// F5 F5 08 00 00 00 00 14 57 4B FD 
		RxTelov = 1; 
}

// ����1���� ------- �绰
interrupt 18 void SCI1_T(void)
{ 
	// __uart1_trance__();	
	uchar i;
	SCI1S1 &= 0xBF; 								// TDRE TC RDRF IDLE OR NF FE PF
	i = TxTelbuf[Tel_frm][1];
	if(Tel_p<=i+1)									// F5 F5 08 00 00 00 00 01 01 08 FD
	{
		SCI1D = TxTelbuf[Tel_frm][Tel_p];	
		Tel_p++;
	}
	else
	{ 
		SCI1C2_TE = 0;
		SCI1C2_TCIE = 0;
		Tel_p = 0; 
		Tel_frm = 10;								// ��ʾ���Դ�����һ��֡		
		_PIN_AW60_UART_EN = 0;						// 485 תΪ����
	} 
}

//SCI2 interrupt service function 
interrupt 19 void SCI2_ERROR(void)
{
	uchar i;
	i = SCI2S1;
	return; 
} 



//==================================================
// ����2���� ------- Arm
interrupt 20 void SCI2_R(void)
{
	_UBYTE  rsls = 0;	
	SCI2S1 &= 0xDF;									// TDRE TC RDRF IDLE OR NF FE PF
	
	rsls = SCI2D;
	if(rsvover>0)
		return;
	if(rsls==0xf5)
	{
		if(headf5bz>0)
			rsv_p = 0;								// ָ�����
		headf5bz = 1; 
	}
	else
		headf5bz = 0;
	
	if(rsv_p<30)
	{
		rsvbufi[rsv_p] = rsls;
		rsv_p++;
	}
	if( (rsvbufi[1]+2)==rsv_p )
		rsvover = 1;
}

// ����2����  ------- Arm
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
		snd_frm = 10;								// ��ʾ���Դ�����һ��֡		
	} 
}
 
 
void  key_do(void)
{
	uchar  k;
	k = 0;	
	//if(!PIN_JJ)									// ����
	//	k = 1;
	if(!PIN_TH)										// ͨ��
		k = 2;
	//if(!PIN_KM)									// ����
	//	k = 3;
	//if(!PIN_WC)									// ���
	//	k = 4;

	if(k==0)										// û���κΰ�������
	{
		if(key_tup<200)
			key_tup++;								// û�м�����  ��ʱ��1
		key_td = 0;
		if(key_tup==4)
		{
			if(key_vl>0)
				key_vl |= 0x10;
		}
	}
	else											// �м�����
	{
		key_tup = 0;
		if(key_td<200)
			key_td++;
		if(key_td==4)
			key_vl = k;
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
	
	sfrInit();										// ��ʼ��cpu
	
	EnableInterrupts;
	//asm ("fset I");           					//enable the response of MCU

	reset_host();									// ��ʼ�������Լ���cpu
	
	init7037(); 									// ��ʼ������ ����оƬ
	
	
	// F5 F5 08 00 00 00 00 01 01 08 FD
	snd_frm = 10;
	Tel_frm = 10;
	
	// F5 F5 08 00 00 00 00 04 55 59 FD  �һ�
	Sndbuf[0][0] = 0xf5;
	Sndbuf[0][1] = 0x08;
	Sndbuf[0][6] = 0x04;
	Sndbuf[0][7] = 0x55;
	Sndbuf[0][8] = 0x59;
	Sndbuf[0][9] = 0xfd;
	
	TxTelbuf[0][0] = 0xf5;
	TxTelbuf[0][1] = 0x08;                    
	TxTelbuf[0][2] = 0x00;
	TxTelbuf[0][5] = 0x00;
	Tel_p = 0;
	
	volume_dwq_to(Volv_lvel[6][0]);
	
	// _PIN_REL_ALL = 1;
	fsyc = 0;											// ���ϵ� ����һ�δ�������
	rst_3s = 0;
	
	for(;;) 
	{  
		__RESET_WATCHDOG(); 							// feeds the dog   ���Ź� Ҫ ����һ�� 
       
		//TxTelbuf[0][3] = rsv_p;						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		//TxTelbuf[0][4] = rsvover;
		//for(m0=0;m0<15;m0++)
		//	TxTelbuf[0][m0+6] = rsvbufi[m0]; 			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		//TxTelbuf[0][2] = TxTelbuf[0][snd321];
		//TxTelbuf[0][3] = TxTelbuf[1][snd321];
		//TxTelbuf[0][4] = Tel_frm;
		//TxTelbuf[0][5] = TxTelbuf[0][snd200ms];		// Ԥ��
		
		if(ms10>=10)									// 10 ms
		{
			ms10 = 0;
			//if(tel_2s<250)								// 2�����յ�arm����ֻ��1��
			//	tel_2s++;
			// Sndbuf[x][6]�������  Sndbuf[x][27] = 0xfd  Sndbuf[x][snd321]���͵�123���� Sndbuf[x][snd200ms]��ʱ200ms  
			
			key_do();									// ��������
			if( (key_vl>0)&&(key_vlBF != key_vl) )
			{
				prepare_sndarm_pack(Key_4);				// �м����� ���ͼ�ֵ
				key_vlBF = key_vl;
			}
			
			
			for(m0=0;m0<5;m0++)						// ����ʱ��
			{
				if(Sndbuf[m0][snd200ms]<250)
					Sndbuf[m0][snd200ms]++;
				if(TxTelbuf[m0][snd200ms]<250)          
					TxTelbuf[m0][snd200ms]++;
			}
			if(timebase<100)
				timebase++;								// ledָʾ����˸ʱ�� 
			else
				timebase = 0;            
			
			sec1s++; 
			if(sec1s>=100)								// 1s                              
			{
				//_PIN_LED_TX = !_PIN_LED_TX;
				//_PIN_LED_MR = !_PIN_LED_MR;
	  			//_PIN_LED_JJAF = !_PIN_LED_JJAF;			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				
				
				sec1s = 0;                       
				min60m++;
				
				// _PIN_LED_AF = !_PIN_LED_AF;
				
				// get_current_time();					// ��1302ʱ��
				
				
				if(min60m>60)                       
					min60m = 0;                           
				
				if(agn_int7037t>0)
					agn_int7037t--;						// ��ʼ��7037ʧ�ܣ�10������³�ʼ��
				if(agn_int7037t==1)
				{
					agn_int7037t = 0;
					init7037(); 						// ��ʼ������ ����оƬ					
				}
				
				/*if(fsyc==0)                                   
				{
					fsyc = 1; 
					
					SCI2C2_TE = 1;	   					// ʹ�ܴ���  TIE TCIE RIE ILIE TE RE RWU
					SCI2C2_TCIE = 1;					// ���ͽ��� �ж�ʹ�� 
					snd_frm = 0; 
					snd_p = 0; 
					SCI2D = 0xaa;		  				// pChar[ (*pPos)++ ];	 
					
					
					SCI1C2_TE = 1;	   					// ʹ�ܴ���  TIE TCIE RIE ILIE TE RE RWU
					SCI1C2_TCIE = 1;					// ���ͽ��� �ж�ʹ��
					Tel_p = 0; 							// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					Tel_frm = 0;						// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%		
					SCI1D = 0xbb;	
				}*/
				
				if(rst_3s<200)							// �ϵ�3�� ����armһ��
					rst_3s++;
				rst_armt++;				
				if( (rst_armt%15)==0 )					// �ղ���arm���ݺ�ÿ60��
				{
					prepare_sndarm_pack(Safe_ztai);		// ����һ�ΰ���״̬
					// init7037(); 						// ��ʼ������ ����оƬ  %%%%%%%%%%%%%% ��ÿ15���ʼ��һ��7037 ���Գ�ʼ�������ã�����Ҫ����
				}
				if(rst_armt>=80)						// �ղ���arm���ݱ�ʾarm������ ��������
				{
					rst_armt = 0;
					m0 = 5;
					while(m0>1)							// ֹͣι�� �ȴ���������
						; 
				}
				
				//rst_3s = 250;
				if(rst_3s==3)							// �ղ���arm���ݱ�ʾarm������ ��������
				{
					rst_armt = 0;
					rst_3s = 4;
					reset_arm();						// ��λarm		
				} 
            }        
		}  
		
		read_afkey();									// �������ӿڵ�״̬ 
		read_Krst_scrn();								// ��λ��У׼��Ļ
		
		check_change();									// ����¶� �����������Ƿ��б仯
		
		respond2Host();									// arm ����ת�沢����
		send2host();		  							// arm ���䷢�͵�֡  ��������
		
		led_operate();   								// ָʾ������  
	}   
}

// 2012 0510 ��ԭ��5����ͨ��ʧ������arm ��Ϊ1����   ԭ��ÿ1��������1�θ�Ϊ15��
//=============================== End ================================================
//
//           _________                               _____________
//          |         |                mic      13,14|            |
//          |         |               ------------>--|->-         |
//          |  5000   |                              |   |        |
//  Arm ����|         |	                          8  |   |        |
// ---------|       15|---<---------------<-------<--|<--         |
// ---------|         |                     |        |            |
//          |         |                     |        |    7037    |
//          |         |                     |    6,7 |            |  9
//          |       12|-->------------------|----->--|----->------|------>����
//          |         |        |            |        |            |
//           ---------         |            |         ------------           
//                             |            | 
//                             |            |
//                             |           ___
//                             |           |R|       J17  J24
//                             |           | |        _____
//                             |            -        |     | �ǿ��ӷֻ�
//                             |            |-----<--|     |
//                             |              MIN_IN |     | ��ǰ��
//                             |                     |     |
//                             |                     |     |
//                             |              Lout1  |     |
//                             |------------------->-|     |
//                                                   |     |
//                                                    -----  
//    
//
//===================================================================================
