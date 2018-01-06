#include "sys.h"
#include "check_cmd.h"
#include "usart.h"
#include "delay.h"



u8 check_dir_io(void)
{
	u8 res=0;
  u8 check;
	GPIO_CHECK_Init();
	check=DIR_CHECK(1);
	  if(check)
		{
			switch(check)
			{
				case 1:printf("\r\ncheck_0\r\n");break;
				case 2:printf("\r\ncheck_1\r\n");break;
				case 3:printf("\r\ncheck_2\r\n");break;		
				case 4:printf("\r\ncheck_3\r\n");break;
				case 5:printf("\r\ncheck_4\r\n");break;
				case 6:printf("\r\ncheck_5\r\n");break;
				case 7:printf("\r\ncheck_6\r\n");break;
				case 8:printf("\r\ncheck_7\r\n");break;			
			}	
    res=1; 			
		}
    return res;
}

u8 check_motor_cmd(void)
{
	u8 res=0;
	if(check_dir_io())
	{
	 printf("\r\n出问题了，请检查");
	res=1;
	}
  return res;
}




