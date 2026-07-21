#include "app_irremote.h"

motion_state_t get_CarState = MOTION_RUN;

uint8_t IR_RGB_flag ;
uint8_t IR_BEEP_flag ;
uint8_t gIRdata = CLEAR_DATA;


uint16_t IR_Motor =200;

void Printf_Irremote(void)
{
    gIRdata = get_infrared_command();
    clear_infrared_command();
    	
		
    if(gIRdata !=CLEAR_DATA)
    {
		
        printf("Key data:%x\r\n",gIRdata);
        gIRdata = CLEAR_DATA;
    }

}

void Control_Blue_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{
		 
		Control_RGB_ALL(Blue_RGB);
		delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}


}


void Control_Red_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{
	

    Control_RGB_ALL(Red_RGB);
    delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}


}


void Control_Green_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{

		 Control_RGB_ALL(Green_RGB);
    delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}


}


void Control_Yellow_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{

	Control_RGB_ALL(Yellow_RGB);
    delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}

}

void Control_Purple_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{

	 Control_RGB_ALL(Purple_RGB);
    delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}

}

void Control_Cyan_RGB_IR()
{
	if(gIRdata==IR_LIGHT)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{

	 Control_RGB_ALL(Cyan_RGB);
    delay_ms(100);
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}

}


//void Pid_Control_RGB()
//{	
//	
//	int RGB_flag=0;
//	
//	if(RGB_flag==0){
//	PID_TypeDef RGBpid;
//	PID_param_init(&RGBpid);
//	set_pid_target(&RGBpid, 0);
//	
//	set_p_i_d(&RGBpid, RGB_p, 0, RGB_d);
//	RGB_flag=1;
//	}
//	if(RGB_flag==1)
//	{





//}
//	
//	set_pid_target(&RGBpid, Blue_RGB);
//	
//	Control_RGB_ALL(Red_RGB);
//	
//}
void Control_Breathe_RGB_IR()
{
	
	static int i;
	if(gIRdata==IR_0)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{
for(i=255;i>1;i--){
//	 Control_RGB_ALL(Red_RGB);
//    delay_ms(i);
//		Control_RGB_ALL(OFF);
//		delay_ms(i);
//		Control_RGB_ALL(Blue_RGB);
//		delay_ms(i);
//	Control_RGB_ALL(OFF);
//		delay_ms(i);
Control_RGB_ALL(Red_RGB);
	delay_ms(i);
Control_RGB_ALL(Red_RGB);
	delay_ms(10);
}
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}

}


void Control_Flowing_RGB_IR()
{
	
	static int i;
	if(gIRdata==IR_8)
	{
		IR_RGB_flag+=1;
	}
	if(IR_RGB_flag%2==0)
	{
//for(i=0;i<2;i++){
	 Control_RGB_ALL(Red_RGB);
    delay_ms(10);
		Control_RGB_ALL(OFF);
		delay_ms(10);
		Control_RGB_ALL(Yellow_RGB);
		delay_ms(10);
	Control_RGB_ALL(OFF);
		delay_ms(10);
	Control_RGB_ALL(Blue_RGB);
	delay_ms(10);
	Control_RGB_ALL(OFF);
		delay_ms(10);
		Control_RGB_ALL(Green_RGB);
	delay_ms(10);
	Control_RGB_ALL(OFF);
		delay_ms(10);
		Control_RGB_ALL(Purple_RGB);
	delay_ms(10);
	Control_RGB_ALL(OFF);
		delay_ms(10);
			Control_RGB_ALL(Cyan_RGB);
	delay_ms(10);
	Control_RGB_ALL(OFF);
		delay_ms(10);
	
	
//}
	}
else{

	Control_RGB_ALL(OFF);
	delay_ms(100);
}

}



//    Control_RGB_ALL(Red_RGB);
//    delay_ms(100);
//    Control_RGB_ALL(Green_RGB);
//    delay_ms(100);
//    Control_RGB_ALL(Blue_RGB);
//    delay_ms(100);
//    Control_RGB_ALL(Yellow_RGB);
//    delay_ms(100);
//    Control_RGB_ALL(Purple_RGB);
//    delay_ms(100);
//    Control_RGB_ALL(Cyan_RGB);
//    delay_ms(100);
void Control_BEEP_IR()
{
	if(gIRdata==IR_BEEP)
	{
		IR_BEEP_flag+=1;
	}
	if(IR_BEEP_flag%2==0)
	{
//		  printf("Key data:%x\r\n",IR_RGB_flag);
	Buzzer_open_state();
	}
else{
	
		Buzzer_close_state();

}


}


void Motor_IR_Control()
{
		if (gIRdata == IR_ADD)    //加速，每次加100 Accelerate by adding 100 each time
	{
		IR_Motor += 100;
		if (IR_Motor > 800)
		{
			IR_Motor = 800;
		}
	}
	if (gIRdata == IR_SUB)	//减速，每次减100 Reduce speed by 100 each time
	{
		IR_Motor -= 100;
		if (IR_Motor < 100)
		{
			IR_Motor = 100;
		}
	}

}



void IR_Control_Car(void)
{
    gIRdata = get_infrared_command();
    clear_infrared_command();
    
    switch(gIRdata)
    {
        case IR_POWER:		Motor_Stop(1) ;Buzzer_close_state();	{Control_RGB_ALL(OFF);delay_ms(100);}break;	
        case IR_UP:			 Motor_IR_Control(); Motor_Run(IR_Motor,IR_Motor) ;delay_ms(1500); Motor_Stop(1) ;break;		
        case IR_LIGHT:		Control_Blue_RGB_IR();break;	
        case IR_LEFT:	 Motor_Left(IR_Motor,-IR_Motor);break;			
        case IR_BEEP:		Control_BEEP_IR();break;			
        case IR_RIGHT:	Motor_Right(-IR_Motor,IR_Motor) ;break;		
        case IR_LEFT_SPIN: 	Motor_Left(IR_Motor,IR_Motor) ;break;	 	
        case IR_BACK:		Motor_IR_Control(); Motor_Back(IR_Motor,IR_Motor) ;delay_ms(1500); Motor_Stop(1) ;break;   		
        case IR_RIHGT_SPIN:Motor_Right(IR_Motor,IR_Motor)  ;	break;	
        case IR_ADD:	Motor_IR_Control();break;			
        case IR_0:		Control_Breathe_RGB_IR()	;break;		
        case IR_SUB:	Motor_IR_Control();break;			
        case IR_1:	break;			
        case IR_2:	break;break;			
        case IR_3:	break;			
        case IR_4:	break;			
        case IR_5:	break;			
        case IR_6:	break;			
        case IR_7:	break;			
        case IR_8:	Control_Flowing_RGB_IR();break;
        case IR_9:  break;
        default: break;

    
    }
    gIRdata = CLEAR_DATA;

}


