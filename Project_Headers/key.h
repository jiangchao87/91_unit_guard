
// 安防接口检查
void read_afkey(void)
{
	uchar i,j,aa;
	aa = PTAD;
	if(pot_a==aa)
		pa_sm_t++;
	else
		pa_sm_t = 0;				// 连续200次相同才有效
	pot_a = aa;
	
	if(pa_sm_t>200)
	{
		pa_sm_t = 0;
		for(i=0;i<8;i++)
		{
			j = (aa>>i)&0x01;         
			if(j>0)                 
				j = 0;
			else
				j = 1;
			safe_zt[i] = j;
		}
	}  
	safe_jtm++;
}


// 复位和校准屏幕
void read_Krst_scrn(void)
{ 
	if(kScr_t<250)
		kScr_t++;
	if(_PIN_CkScreen)				// K1电源检查  开关闭合=1   断开电容充满电后=0
	{
		if(kScr_hlbz==0)
			kScr_hlbz = 0;
		kScr_hlbz = 1; 
	}
	else
	{
		if(kScr_hlbz>0) 
			kScr_t = 0;  
		kScr_hlbz = 0;	
	} 
}

