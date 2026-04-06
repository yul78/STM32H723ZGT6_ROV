#include "mid_gps.h"
//本程序只供学习使用
//ALIENTEK STM32F103C8T6开发板	
/* 历史代码中保留的时区配置表。 */
char SetTimeZone[25] = {0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xf1,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c};

/**
 * @brief 查找第cx个逗号相对当前缓冲区起始地址的偏移。
 * @param buf NMEA语句缓冲区。
 * @param cx 目标逗号序号。
 * @retval 逗号偏移；如果未找到则返回0xFF。
 */
static uint8_t MID_GPS_CommaPos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

/**
 * @brief 计算m的n次方。
 * @param m 底数。
 * @param n 指数。
 * @retval 计算结果。
 */
static uint32_t MID_GPS_Pow10(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**
 * @brief 将NMEA字段中的数字字符串转换为整数。
 * @param buf 数字字段起始地址。
 * @param dx 返回小数位数。
 * @retval 转换后的整数值。
 */
int MID_GPS_StrToNum(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=MID_GPS_Pow10(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=MID_GPS_Pow10(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*MID_GPS_Pow10(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 

/**
 * @brief 解析GPGSV语句中的可见卫星信息。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseGpgsv(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	if ((gpsx == NULL) || (buf == NULL)) return;
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	if (p1 == NULL) return;
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=MID_GPS_CommaPos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=MID_GPS_StrToNum(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		if (p1 == NULL) break;
		for(j=0;j<4;j++)
		{	  
			posx=MID_GPS_CommaPos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=MID_GPS_StrToNum(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=MID_GPS_CommaPos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=MID_GPS_StrToNum(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=MID_GPS_CommaPos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=MID_GPS_StrToNum(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=MID_GPS_CommaPos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=MID_GPS_StrToNum(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}

/**
 * @brief 解析GPGGA语句中的海拔和参与定位的卫星数。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseGpgga(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	if ((gpsx == NULL) || (buf == NULL)) return;
//	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");
	p1=(uint8_t*)strstr((const char *)buf,"GGA");
	if (p1 == NULL) return;
	posx=MID_GPS_CommaPos(p1,9);							
						
	if(posx!=0XFF)gpsx->altitude=MID_GPS_StrToNum(p1+posx,&dx); //得到海拔高度
	posx=MID_GPS_CommaPos(p1,7);								
	if(posx!=0XFF)gpsx->posslnum=MID_GPS_StrToNum(p1+posx,&dx);  //得到卫星数目
}

/**
 * @brief 解析GPGSA语句中的定位模式和精度信息。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseGpgsa(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	if ((gpsx == NULL) || (buf == NULL)) return;
	p1=(uint8_t*)strstr((const char *)buf,"GSA");
	if (p1 == NULL) return;
	posx=MID_GPS_CommaPos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=MID_GPS_StrToNum(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=MID_GPS_CommaPos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=MID_GPS_StrToNum(p1+posx,&dx);
		else break; 
	}				  
	posx=MID_GPS_CommaPos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=MID_GPS_StrToNum(p1+posx,&dx);  
	posx=MID_GPS_CommaPos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=MID_GPS_StrToNum(p1+posx,&dx);  
	posx=MID_GPS_CommaPos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=MID_GPS_StrToNum(p1+posx,&dx);  
}

/**
 * @brief 解析RMC语句中的时间、经纬度和速度。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseRmc(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	float rs;  
	uint32_t temp;
	int itemp;
	if ((gpsx == NULL) || (buf == NULL)) return;
	p1=(uint8_t*)strstr((const char *)buf,"RMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	if (p1 == NULL) return;
	posx=MID_GPS_CommaPos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=MID_GPS_StrToNum(p1+posx,&dx)/MID_GPS_Pow10(10,dx);	 	//得到UTC时间,去掉ms
		
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	
	}
	posx=MID_GPS_CommaPos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{	
		temp=MID_GPS_StrToNum(p1+posx,&dx);		 	 
		gpsx->latitude=temp/MID_GPS_Pow10(10,dx+2);	//得到°
		rs=temp%MID_GPS_Pow10(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*MID_GPS_Pow10(10,5)+(rs*MID_GPS_Pow10(10,5-dx))/60;//转换为° 
	}
	posx=MID_GPS_CommaPos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)
	{
		gpsx->nshemi=*(p1+posx);
	}			 
	posx=MID_GPS_CommaPos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=MID_GPS_StrToNum(p1+posx,&dx);		 	 
		gpsx->longitude=temp/MID_GPS_Pow10(10,dx+2);	//得到°
		rs=temp%MID_GPS_Pow10(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*MID_GPS_Pow10(10,5)+(rs*MID_GPS_Pow10(10,5-dx))/60;//转换为° 
	}
	posx=MID_GPS_CommaPos(p1,6);								//东经还是西经
	if(posx!=0XFF)
	{

		gpsx->ewhemi=*(p1+posx);		 
	}	
	posx=MID_GPS_CommaPos(p1,7);								//得到速率
	if(posx!=0XFF)
	{
		temp=MID_GPS_StrToNum(p1+posx,&dx);
		temp = temp*10*1.852;
		itemp = temp;
		gpsx->speed = itemp;
	} 

	posx=MID_GPS_CommaPos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=MID_GPS_StrToNum(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	
	}
}

/**
 * @brief 解析GPVTG语句中的地面速度。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseGpvtg(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	if ((gpsx == NULL) || (buf == NULL)) return;
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");							 
	if (p1 == NULL) return;
	posx=MID_GPS_CommaPos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=MID_GPS_StrToNum(p1+posx,&dx);
		if(dx<3)gpsx->speed*=MID_GPS_Pow10(10,3-dx);	 	 		//确保扩大1000倍
	}
}  

/**
 * @brief 解析一组NMEA语句中的常用GPS信息。
 * @param gpsx 解析结果结构体。
 * @param buf NMEA语句缓冲区。
 */
void MID_GPS_ParseAll(nmea_msg *gpsx,uint8_t *buf)
{
	MID_GPS_ParseGpgsv(gpsx,buf);	//GPGSV解析
	MID_GPS_ParseGpgga(gpsx,buf);	//GPGGA解析 	
	MID_GPS_ParseGpgsa(gpsx,buf);	//GPGSA解析
	MID_GPS_ParseRmc(gpsx,buf);	//GPRMC解析
//	MID_GPS_ParseGpvtg(gpsx,buf);	//GPVTG解析
}








