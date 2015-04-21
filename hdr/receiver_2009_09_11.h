#define pi 3.1415926
// AWGN generation using Box-Muller transform
void AWGN(float *data_I, float *data_Q, unsigned long long arraylength, float var){
    float n1,n2;
    float maximum = (float)RAND_MAX;
    
    //srand(time(NULL));
    
    // 依照每次通道能量的大小加noise 開始 //
    float power_RX;
    power_RX=0;
    for(unsigned long long i=0; i<arraylength; i++){
        power_RX+=data_I[i]*data_I[i]+data_Q[i]*data_Q[i];
    } 
    power_RX/=(float)arraylength;  //平均每個sample的能量
    power_RX=power_RX*var/2.0; 
    // 依照每次通道能量的大小加noise 結束 // 

    for(unsigned long long i=0; i<arraylength; i++){
       n1 = (float)rand()/maximum;
       n2 = (float)rand()/maximum;
    ////////////////////////////////////////////////   
       while(n1==0){
         n1 = (float)rand()/maximum;                      
       }             
       while(n2==0){
         n2 = (float)rand()/maximum;                    
       }        
    ////////////////////////////////////////////////      
          
       //data_I[i] += sqrt(-2*log(n1)*var)*cos(2*pi*n2);   //訊號加上AWGN  additive white gaussian noise
       //data_Q[i] += sqrt(-2*log(n1)*var)*sin(2*pi*n2);   //sqrt(var)即標準差
        
       // 依照每次通道能量的大小加noise 開始 //
       data_I[i] += sqrt(-2*log(n1)*power_RX)*cos(2*pi*n2);
       data_Q[i] += sqrt(-2*log(n1)*power_RX)*sin(2*pi*n2);
       // 依照每次通道能量的大小加noise 結束 // 
    }
}

/***********************************************************************************/
// Carrier Frequency Offset
void CFO(float* CFO_I,float* CFO_Q,float FO,float PO,unsigned long long seq_length)    //此處的seq_length代表某一天線之資料總長度
{
	  unsigned long long i;
	  float a,b;
	  FO=FO/pow(10,6);
	  for(i=0;i<seq_length;i++)                        //不懂此處之i及accumulater所代表之意含
	  {
   
		a=CFO_I[i];
		b=CFO_Q[i];
		/* 
	    CFO_I[i] = a*cos(2*pi*FO*i+PO)+b*sin(2*pi*FO*i+PO);   //為何accumulater與i同時增加  可以把accumulater換成FO  ?
		CFO_Q[i] = b*cos(2*pi*FO*i+PO)-a*sin(2*pi*FO*i+PO);
		*/ 
		
		
		//修正CFO公式錯誤 
		
	    CFO_I[i] = a*cos(2*pi*FO*i+PO)-b*sin(2*pi*FO*i+PO);   //為何accumulater與i同時增加  可以把accumulater換成FO  ?
		CFO_Q[i] = b*cos(2*pi*FO*i+PO)+a*sin(2*pi*FO*i+PO);		
		
		//好玩 
		/*
	    CFO_I[i] = a*cos(2*pi*FO/seq_length*i*i+PO)-b*sin(2*pi*FO/seq_length*i*i+PO);   //為何accumulater與i同時增加  可以把accumulater換成FO  ?
		CFO_Q[i] = b*cos(2*pi*FO/seq_length*i*i+PO)+a*sin(2*pi*FO/seq_length*i*i+PO);
        */			
	  }       
}

/***********************************************************************************/
// Phase Noise
void PN(float* nI,float* nQ,float p_noise,unsigned long long seq_length)    //p_noise指variance?  ==>sqrt(p_noise)即standard deviation
{    	     			     
    float n1,n2;
	float phi=0,phq=0;
	float a,b;
    float maximum = (float)RAND_MAX;
    srand(time(NULL));
    for(unsigned long long i=0; i<seq_length; i++){                              //此處的seq_length代表某一天線之資料總長度
       n1 = (float)rand()/maximum;
       n2 = (float)rand()/maximum;
    ////////////////////////////////////////////////   
       while(n1==0){
         n1 = (float)rand()/maximum;                      
       }             
       while(n2==0){
         n2 = (float)rand()/maximum;                    
       }        
    ////////////////////////////////////////////////  
       phi += sqrt(-2*log(n1)*p_noise)*cos(2*pi*n2);      //phase noise是Gaussian random variable之不斷相加
       phq += sqrt(-2*log(n1)*p_noise)*sin(2*pi*n2);      //因此phq是多餘的?
       a=nI[i];
	   b=nQ[i];
       nI[i] = a*cos(phi)-b*sin(phi); 
       nQ[i] = b*cos(phi)+a*sin(phi);                      //應該是b*cos(phi)+a*sin(phi)  ??
    }
}

/***********************************************************************************/
// IQ imbalance
void IQ_Imbalance(float* IQ_I,float* IQ_Q,float alpha,float phase,unsigned long long seq_length)
{
   unsigned long long i;
   float a,b;
   for(i=0;i<seq_length;i++)
   {
    a=IQ_I[i];
    b=IQ_Q[i];	
    IQ_I[i] = (1+alpha)*(a*cos(phase/2)-b*sin(phase/2));           //其中carrier frequency offset之部分因為寫在另外副程式(CFO)故沒有放入
    IQ_Q[i] = (1-alpha)*(b*sin(-phase/2)+b*cos(-phase/2));         //直接代公式即可
   }
}

/***********************************************************************************/
// DC offset
void DCoffset(float* DC_I,float* DC_Q,float I_DC,float Q_DC, unsigned long long seq_length)
{
	unsigned long long i;
	for(i=0;i<seq_length;i++)
	{
      DC_I[i]=DC_I[i]+I_DC;                                          //直接加上offset
      DC_Q[i]=DC_Q[i]+Q_DC;
	}
}

/***********************************************************************************/
// Sampling Clock Offset
void SCO(float *SCO_out,float* SCO_in,float offset, float initial,unsigned long long seq_length)
{
	//Farrow interpolator
	unsigned long long m;
	float mu;
	unsigned long long k;
    offset=offset/pow(10,6);
  for(k=0;k<seq_length;k++)
  {

 	m =(unsigned long long)( k*(1.0+offset)+initial);   //有修改                //1+offset=Tnew/Torig     因為m是整數型式，所以能取floor  我改成(int) 
     


	mu = k*(1.0+offset)+initial-m;       //有修改  

	//initial conditions
	if(k==0)
	{              
	  SCO_out[k] =   SCO_in[m+2]*(-0.5*mu+0.5*mu*mu)
	    	        +SCO_in[m+1]*(1.5*mu-0.5*mu*mu)
                    +SCO_in[m]*(1-0.5*mu-0.5*mu*mu);
                                      //有修改        //為何需要initial   第一筆資料的輸出似乎並不直接等於initial offset
	}
	
	else
	{
	  if( (m+2)<= seq_length-1 )         //判斷m+2,m+1,m,m-1是否超出原數列之index,若超出公式需修改
	  {
		
		SCO_out[k] =   SCO_in[m+2]*(-0.5*mu+0.5*mu*mu)
	    	        +SCO_in[m+1]*(1.5*mu-0.5*mu*mu)
                    +SCO_in[m]*(1-0.5*mu-0.5*mu*mu)
                    +SCO_in[m-1]*(-0.5*mu+0.5*mu*mu);
	  }
	  else
	  {
		  if( (m+1)<= seq_length-1 )
		  {
               SCO_out[k] =  SCO_in[m+1]*(1.5*mu-0.5*mu*mu)
                    +SCO_in[m]*(1-0.5*mu-0.5*mu*mu)
                    +SCO_in[m-1]*(-0.5*mu+0.5*mu*mu);
		  }
		  else
		  {
               if( m<= seq_length-1 )
		      {
               SCO_out[k] =  SCO_in[m]*(1-0.5*mu-0.5*mu*mu)
                    +SCO_in[m-1]*(-0.5*mu+0.5*mu*mu);
		      }
			   else
			   {
				   if( (m-1)<= seq_length-1 )
		          {
                    SCO_out[k] = SCO_in[m-1]*(-0.5*mu+0.5*mu*mu);
		          }
				   else
				   {
                       SCO_out[k]=SCO_out[k-1];
				   }//end if
			   }//end if 
		  }//end if
	  }//end if

	}//end if
  }//end for k
}//end void
