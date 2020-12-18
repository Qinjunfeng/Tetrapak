/*
*Arduino PID仿真模型
*/
/*外部全局*/
int LED = 13; //LED输出定义为pin13
int waveform; //PWM波形显示变量

//Var///////////////////////////////////////////////////////////////////
float ControlValue;      //存放PID运算后的输出值
float entropy;   		     //熵增变量，用熵来表达受控对象的阻力默认值-0.50
float SetValue; 		     //设定值
float ActualValue;       //当前值 用于表现受控对象的状态数据

char parData ; 			     //接收来自串口的参数
char cmdData ; 			     //接受来自串口的命令 操作

bool Seri_control;        //串口输入分支标志
bool PIDswitch;			      //PID控制开关
bool PWM;         	      //PID脉宽调制对象
bool PWMprint;            //PWM波形打印开关

unsigned long TMR_interval;	  //PID刷新时间间隔




//定义上升沿与下降沿结构体//////////////////////////////////////
struct edgeDef
    {
      bool inputbit;//input
      bool Rtemp;   //rising temp bit
      bool Ftemp;   //falling temp bit
      bool Redge;   //RisingEdge令
      bool Fedge;   //FallingEdge令
    };
//定义上升沿&下降沿函数////////////////////////////////////////
void EDGE(struct edgeDef *p)
{
  	p->Redge = p->inputbit && !p->Rtemp;//上升沿令
  	p->Rtemp = p->inputbit;
  	p->Fedge = !p->inputbit && p->Ftemp;//下降沿令
  	p->Ftemp = p->inputbit;
}
//////////////////////////////////////////
struct Def_TMR_DB//定时器结构体
  {
   	unsigned long timekeeper;	//时间戳
   	unsigned long ET;			    //定时器当前值
   	bool 	  	    TT;		      //定时器计时状态
   	bool 		      DN;		      //定时器计时完成标志
   	bool 		      temp;		    //EN位上升沿临时存储变量
  };//定义定时器结构Def_TMR_DB

void TIMER(bool EN,unsigned long PV,struct Def_TMR_DB *TMR)//定时器函数原型
  {
    
    if(EN)//如果EN == TRUE时 开始定时器的逻辑
    { 
      
      if(!TMR->temp)//在程序运行的第二个扫描周期以及之后 TMR->temp会阻断timekeeper的数据刷新
      TMR->timekeeper = millis();//在检测的EN第一个扫描周期将系统时间存入timekeeper
      TMR->temp = EN;//将EN状态传递给temp变量以此产生一个上升沿
      
      TMR->ET = millis() - TMR->timekeeper;//计算消耗的时间
      if(TMR->ET >= PV)//如果（EN使能后消耗的时间ET）大于等于设定的时间PV那么 DN=TRUE 否则DN = false
        TMR->DN = HIGH;
      else
        TMR->DN = LOW;
        
      if(!TMR->DN)//如果在EN = TRUE 且DN为false（未计时完成的情况下）TT = TRUE
        TMR->TT = HIGH;//在整个计时过程中TT一直为TRUE 直到计时完成
      else    //如果此时计时完成 那么TT = FALSE 所以TT和DN的波形会呈现完全相反的状态
        TMR->TT = LOW;
    }
    else
    {
      //如果EN在定时过程中被中断那么成员数据则被清除
      TMR->timekeeper = 0;
      TMR->ET         = 0;
      TMR->TT         = LOW;
      TMR->DN         = LOW;
      TMR->temp       = LOW;
    }
    //由于使用了结构体指针，所以函数无需返回值
  }
/*PID struct*/
struct PIDdata
    {
      float Kp;                       //比例系数Proportional
      float Ki;                       //积分系数Integral
      float Kd;                       //微分系数Derivative

      float Ek;                       //当前误差
      float Ek1;                      //前一次误差 e(k-1)
      float LocSum;                   //积分累加变量
	    float last[3];				          //用于判断积分
    };

/*PID_func*/
float PID_FUNC(float SetValue,float ActualValue,struct PIDdata *PID)
{
  float PIDloc;

  PID->Ek = SetValue - ActualValue;//求当前误差
  /*积分运算使能条件*/
  do{	
		  if(ActualValue > SetValue * 0.30)//限定积分运算的工作范围在SV %30以上
		  {
		  	if(abs(PID->Ek - (PID->last[0]+PID->last[1]+PID->last[2]) / 3) < 1.0)//判断波动之小于2则开始积分
			  	PID->LocSum += PID->Ek;
		  	else
			  	PID->LocSum += 0.0;
		  }
	  	/*历史数据记录*/
	  	PID->last[0] = PID->Ek1;
	  	PID->last[1] = PID->last[0];
	  	PID->last[2] = PID->last[1];
  	}while(0);//do while(0)在此只为分割语句块，无他意义
		  
  PIDloc = (PID->Kp * PID->Ek) + (PID->Ki * PID->LocSum) + (PID->Kd * (PID->Ek - PID->Ek1));

  if(PIDloc > 1000)//PIDLimit
    {
		PIDloc = 1000;
	}
  else if(PIDloc < -1000)
    {
		PIDloc = -1000;
	}

  	PID->Ek1 = PID->Ek;  

  return PIDloc;
}


/*Help function*/
void HELP(void)
	{
		Serial.println("***PIDsim*** By Qinjunfeng 20200909");
		Serial.println("     ------CMD-----");
    Serial.println("'k' ---> set      Time Interval");
		Serial.println("'S' ---> set      SetValue");
		Serial.println("'e' ---> set      entropy ");
		Serial.println("'p' ---> set      Kp   ");
		Serial.println("'i' ---> set      Ki  ");
		Serial.println("'d' ---> set      Kd  ");
		Serial.println("'s' ---> PID      running......");
    Serial.println("'t' ---> PID      stop!");
		Serial.println("'v' ---> print    Message~");
		Serial.println("'m' ---> print    ActualValue");
		Serial.println("H/h ---> help     menu");
    Serial.println("\n");

	};
struct PIDdata PID;        //PID运算结构体变量
/*串口打印数据*/
void MESSAGE(void)
	{
		
		//PID间隔时间
    Serial.print("PID Cycle = ");
    Serial.print(TMR_interval);
    Serial.println("ms;");
    
    //被控量设定值
		Serial.print("setValue = ");
		Serial.println(SetValue);

		//系统反馈值
		Serial.print("Actual Value = ");
		Serial.println(ActualValue);
		
		
		//PID输出
		Serial.print("PID_ControlValue = ");
		Serial.println(ControlValue);
		
		//PID当前误差
		Serial.print("PID.Ek = ");
		Serial.println(PID.Ek);
		//PID上次误差
		Serial.print("PID.Ek1 = ");
		Serial.println(PID.Ek1);
		
		//PID KP值
		Serial.print("PID.Kp = ");
		Serial.println(PID.Kp); 
		//PID Ki值
		Serial.print("PID.Ki = ");
		Serial.println(PID.Ki);
		//PID Kd值
		Serial.print("PID.Kd = ");
		Serial.println(PID.Kd);

    //熵增
    Serial.print("entropy = ");
    Serial.println(entropy);
		
		
		Serial.print("\n");
	};

void DEBUG(unsigned int number){
    //打印Debug编号
    Serial.print(">>>>>>>>>>>>>>>>>> Debug No: =");
    Serial.println(number);
    //打印命令 以整数形式
    Serial.print("CMDdata = ");
    Serial.println(cmdData);
    //打印串口接收到的参数
    Serial.print("ParData = ");
    Serial.println(parData);
  }


//Struct Var//////////////////////////////////////////////////

struct Def_TMR_DB Pulser;     //用来产生脉冲 用于模拟执行器做功
struct Def_TMR_DB TMR_Cycle;  //用于限定PID计算的周期


struct edgeDef CaseState[6];  //功能 在每一个嵌套case语句中打印提示语的上升沿结构

////////////////////////////////////////////////////////////////////////
void setup() {
  
  Serial.begin(19200);  //串口初始化 波特率19200bps

  pinMode(LED, OUTPUT); //设置ping 13 = LED输出

  /*PID 系数初始化*/
  do{
	   SetValue = 360.0;
	   entropy = -0.5;

	   PID.Kp = 10.00;
	   PID.Ki = 0.30;
	   PID.Kd = 0.50;

     TMR_interval = 1000;


	  
	   PIDswitch = HIGH;
  	}while(0);
  
  
  }
/////////////////////////////////////////////////////////////////////////
void loop() {
  
   if (Serial.available() > 0)//如果检测到从串口进来的字符则执行
    {
       if(!Seri_control)
          cmdData = Serial.read();  //先读取操作命令
       else
          parData = Serial.read();  //再读取调整命令   
    }   

  // 用switch嵌套处理串行端口的用户命令以及参数。
  switch(cmdData)
    {
      case 'S'://SetValue  //case0
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向
		  
          //进入下一层switch
          switch(parData)
              {
                case '+'://参数递增调整
                    SetValue += 36.0;
                    Serial.print("SetValue = ");
                    Serial.println(SetValue);
                    //DEBUG(0);//debug0
         
                break;
                
                case '!'://参数恢复默认值
                    SetValue = 360.0;
                    Serial.print("SetValue = ");
                    Serial.println(SetValue);
         
                break;

                case '-'://参数递减调整
                    SetValue -= 36.0;
                    Serial.print("SetValue = ");
                    Serial.println(SetValue);
                    
                break;    
                }
          break;

      case 'e'://set entropy //case1
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向

          switch(parData)
            {
                case '+'://参数递增调整
                    entropy += 0.20;
                    Serial.print("entropy = ");
                    Serial.println(entropy);
                    //DEBUG(1);//debug1
         
                break;
                
                case '!'://参数恢复默认值
                    entropy = 0.50;
                    Serial.print("entropy = ");
                    Serial.println(entropy);
         
                break;
                

                case '-'://参数递减调整
                    entropy -= 0.20;
                    Serial.print("entropy = ");
                    Serial.println(entropy);
                
                break;    
            }

          break;
      case  'p'://Kp adjust  //case2
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向
		  
          switch(parData)
              {
                case '+'://参数Kp递增调整
                    PID.Kp += 0.1;
                    Serial.print("PID.Kp = ");
                    Serial.println(PID.Kp);
                    //DEBUG(2);//debug2

                break;
                
                case '!'://参数Kp恢复默认值
                    PID.Kp = 10.0; 
                    Serial.print("PID.Kp = ");
                    Serial.println(PID.Kp);               
                break;

                case '-'://参数Kp递减调整
                    PID.Kp -= 0.1;
                    Serial.print("PID.Kp = ");
                    Serial.println(PID.Kp);               
                break;    
                }
          break;

      case  'i'://Ki adjust   //case3
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向

          switch(parData)
              {
                case '+'://参数Ki递增调整
                    PID.Ki += 0.02;
                    Serial.print("PID.Ki = ");
                    Serial.println(PID.Ki); 
                    //DEBUG(3);//debug3
                break;

                case '!'://参数Ki恢复默认值
                    PID.Ki = 0.30;
                    Serial.print("PID.Ki = ");
                    Serial.println(PID.Ki); 
                break;

                case '-'://参数Ki递减调整
                    PID.Ki -= 0.02;
                    Serial.print("PID.Ki = ");
                    Serial.println(PID.Ki); 
                break;
                }

          break;
      case  'd'://Kd adjust  //case4
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向

          switch(parData)
              {
                case '+'://参数Kd递增调整
                    PID.Kd += 0.3;
                    Serial.print("PID.Kd = ");
                    Serial.println(PID.Kd);
                    //DEBUG(4);//debug4
                break;

                case '!'://参数Kd恢复默认值
                    PID.Kd = 0.50;
                    Serial.print("PID.Kd = ");
                    Serial.println(PID.Kd);
                break;

                case '-'://参数Kd递减调整
                    PID.Kd -= 0.3;
                    Serial.print("PID.Kd = ");
                    Serial.println(PID.Kd);
                break;
                
                }

          break;

      case  'k'://Kd adjust  //case5
          Seri_control = HIGH;//如果命令输入正确则锁定外侧switch的指向

          switch(parData)
              {
                case '+'://参数Kd递增调整
                    TMR_interval += 100;
                    Serial.print("TMR_interval = ");
                    Serial.println(TMR_interval);
                    //DEBUG(5);//debug5
                break;

                case '!'://参数Kd恢复默认值
                    TMR_interval = 1000;
                    Serial.print("TMR_interval = ");
                    Serial.println(TMR_interval);
                break;

                case '-'://参数Kd递减调整
                    TMR_interval -= 100;
                    if(TMR_interval < 100) TMR_interval = 100;//对Cycle定时器的PV进行限幅 最小0.1秒
                    Serial.print("TMR_interval = ");
                    Serial.println(TMR_interval);
                break;
                
                }

          break;

        case 'W'://打印PWM波形选项
          Seri_control = HIGH;
          
          switch(parData)
            {

              case '1':// PWM 打印功能开启
                PWMprint = HIGH;
                Serial.println("print PWM = on");
              break;

              case '0'://PWM 打印功能关闭
                PWMprint = LOW;//
                Serial.println("print PWM = off");
              break;
            }
           if(PWMprint && Pulser.DN)//打印PWM波形逻辑
            {
              if(PWM)
              {
                waveform = 100;
              }
              else
              {
                waveform = 50;
              }

              Serial.println(waveform);  
            }

        break;


        case 'H'://帮助菜单
	      case 'h':// H or h都可以激活帮助菜单
	    		HELP();//打印帮助指令
          cmdData = ' ';
	    	break;
	  
    	  case  's'://PID启动命令
            PIDswitch = HIGH;
		    Serial.println("PID Running......");
        cmdData = ' ';
        break;

	      case	't'://PID停止命令
	 	     PIDswitch = LOW;
	    	  Serial.println("PID stop!!!");
         cmdData = ' ';
	    	break;

	     case	'r'://PID 被控量ActualValue重置为零
	  	  ActualValue = 0.00;
		  

		   Serial.print("ActualValue = ");
		    Serial.println(ActualValue);
      
		     cmdData = ' ';
		    break;

        case  'v'://View 
            //print输出
            MESSAGE();
            
            cmdData = ' ';    
       //break;//输出MESSAGE后直接打印Actualvalue      
                
       case  'm'://view PIDsim
            if(TMR_Cycle.DN && !PWMprint)
            { 
              Serial.println(ActualValue);
            }


          break;
      }
      if('q'== cmdData || 'Q' == cmdData || 'q'== parData || 'Q'== parData)
      {
          cmdData = ' ';//每当按下q之后参数的变量才会被清除 才可以选择别的参数
          Seri_control = LOW;//每当按下q之后 串口数据分支才指向 cmdData
      }
      parData = ' ';//每一次的break后都会被清理掉parData便于反复接受用户操作

      /*参数调整提示语*/
      do{
          //打印一次所调整参数的提示语
          CaseState[0].inputbit = (cmdData == 'S');
		      EDGE(&CaseState[0]);
		      if(CaseState[0].Redge) Serial.println("Set  SetValue");

		      //打印一次所调整参数的提示语
          CaseState[1].inputbit = (cmdData == 'e');
	    	  EDGE(&CaseState[1]); 
		      if(CaseState[1].Redge) Serial.println("Set  entropy");

          //打印一次所调整参数的提示语
          CaseState[2].inputbit = (cmdData == 'p');
		      EDGE(&CaseState[2]);
		      if(CaseState[2].Redge) Serial.println("---Set Kp---");

    		  //打印一次所调整参数的提示语
          CaseState[3].inputbit = (cmdData == 'i');
		      EDGE(&CaseState[3]);
		  		if(CaseState[3].Redge) Serial.println("---Set Ki---");

  		    //打印一次所调整参数的提示语
          CaseState[4].inputbit = (cmdData == 'd');
		      EDGE(&CaseState[4]);
  		    if(CaseState[4].Redge) Serial.println("---Set Kd---");

    		  //打印一次所调整参数的提示语
          CaseState[5].inputbit = (cmdData == 'k');
		      EDGE(&CaseState[5]);
		  		if(CaseState[5].Redge) Serial.println("---Set PID Cycletime---");

        }while(0);

      do{ 
          if('S' != cmdData)
          {
              CaseState[0].inputbit = LOW;//当cmdData不等于字符 S 时解除case0的状态
          }
          if('e' != cmdData)
          {
              CaseState[1].inputbit = LOW;//当cmdData不等于字符 e 时解除case1的状态
          }
          if('p' != cmdData)
          {
              CaseState[2].inputbit = LOW;//当cmdData不等于字符 p 时解除case2的状态
          }
          if('i' != cmdData)
          {
              CaseState[3].inputbit = LOW;//当cmdData不等于字符 i 时解除case3的状态
          }
          if('d' != cmdData)
          {
              CaseState[4].inputbit = LOW;//当cmdData不等于字符 d 时解除case4的状态
          }
          if('k' != cmdData)
          {
              CaseState[5].inputbit = LOW;//当cmdData不等于字符 k 时解除case5的状态
          }

        }while(0);

  
  /* N Hz脉冲 */
  TIMER(!Pulser.DN,TMR_interval/10,&Pulser);// 脉冲发生器 
  
  //PID周期定时器 PID Cycle
  TIMER(!TMR_Cycle.DN && PIDswitch,TMR_interval,&TMR_Cycle);

  if(PIDswitch && TMR_Cycle.DN)//PID开关打开的情况下 每一赫兹计算一次PID
    {
       ControlValue = PID_FUNC(SetValue,ActualValue,&PID); 
    }
  /*PWM转换逻辑*/
  if(TMR_Cycle.ET <= ControlValue)// ControlValue越大脉冲宽度越大，调节则更激进
    {
        PWM = HIGH;
    }
  else
    PWM =  LOW;

  /*LED 状态输出*/
  digitalWrite(LED,PWM);//自带LED灯
  
    /*仿真数据的计算逻辑*/
    if(PIDswitch && PWM && Pulser.DN)//PIDswitch为使能条件、PWM则决定输出时间、Edge10Hz.Redge在PWM的时间范围内每秒10Hz执行该语句块。
    {
         ActualValue+= 1.0;//脉宽在最大的情况下每秒个计算周期递增10
    }
    /*数据熵增仿真逻辑*/
    if(TMR_Cycle.DN)
    {
         ActualValue += entropy;//模仿熵增对受控对象的影响（熵值是负数）
    }
    else if( ActualValue < 0)   //若被控物理量在0轴以下进行限幅，防止被控物理量变为负值
         ActualValue = 0;

  
}
