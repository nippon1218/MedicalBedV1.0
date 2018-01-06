2017.12.19	
将电机4和电机5的pwm口调换了
故：1，将床体中的4号和5号在外面那个头子上对换一下
    2，将列表更新一下，包括插头的列表和cad图中对应的表

自动坐便运行顺序：
1：Uart_Washlet_Auto()				——调用的函数
2：Push_Rod_Start，Motor_1_START	——支背曲腿都起来
3：Uart_Back_Leg()					——支背、下曲腿同时运行函数
4：Uart_Washlet(0);	            	——坐便打开			
5：Uart_Swash_Dry();  				——冲洗烘干			
  5-1：Uart_Push_Rod_Swash_Dry(1,2000);      ————冲洗烘干推杆伸出    
  5-2：Uart_Swash_Auto();  				   ————自动冲洗烘干
    5-2-1：(DIR_SB=1)Uart_Push_Rod_Swash(flag,swash_dry_pulse_lim);————按键执行自动冲洗功能
  5-3：Uart_Swash_Hand();			    ——按键执行冲洗功能
	5-3-1：Uart_Push_Rod_Swash()				——推杆运动	
  5-4:Uart_Dry_Auto();  				——自动烘干2分钟
	5-4-1：Uart_Push_Rod_Dry()					——推杆运动
  5-5：Uart_Dry_Hand();				——按键执行烘干功能
	5-5-1： Uart_Push_Rod_Dry					——推杆运动

6：Uart_Washlet()					——坐便关闭	