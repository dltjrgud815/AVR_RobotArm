#define F_CPU 16000000UL
#define sbi(PORTX,bitX) PORTX|=(1<<bitX)
#define cbi(PORTX,bitX) PORTX&=~(1<<bitX)
#define tbi(PORTX,bitX) PORTX^=(1<<bitX)

#define FS_SEL 131

#define ADC_VREF_TYPE 0x40

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>

int cnt = 0;
double degree = 0;
int keypad(void);
unsigned int flexsensor1[5], flexsensor2[5];
double flexvalue1, flexvalue2, flexvalue3;
unsigned int motor1, motor2, motor3, motor4;

void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char addressr);
void USART_Transmit(unsigned char tx_data);
void USART_Transmit_init4(int data);
void get_raw_data();
void calibrate();

volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
volatile double bas_a_x,bas_a_y,bas_a_z;
volatile double bas_g_x,bas_g_y,bas_g_z;
volatile double a_x,a_y,a_z;
volatile double g_x,g_y,g_z;
volatile double las_angle_gx,las_angle_gy,las_angle_gz;
volatile double angle_ax,angle_ay,angle_az;
volatile double angle_gx,angle_gy,angle_gz;
volatile double roll,pitch,yaw;
volatile double alpha;
float roll_1, pitch_1;


void INT_init();
void IO_init();
void ADC_init1();
void ADC_init2();
void UART_init();
unsigned int read_adc(unsigned char adc_input);
unsigned int adc_start1(unsigned char ch);
unsigned int adc_start2(unsigned char ch);
void TIMER_init1();
void TIMER_init2(); // 자이로 센서
void TIMER_init3();
void delay_us(unsigned int time);
void delay_ms(unsigned int time);
void mpu6050();

int main()
{
		IO_init(); 
		TIMER_init1();
		ADC_init1();
		ADC_init2();
		UART_init();
		TWCR = (1<<TWEN);
		TWBR = 12;
		twi_write(0x6B, 0x00); 	//sleep 끔
		_delay_ms(1);
		twi_write(0x1A, 0x05); 	//DLPF 10Hz
		calibrate();

		SREG = 0x80;

	while(1)
		{
			for(int i = 0; i < 5; i++)
			{
				flexsensor1[i] = adc_start1(2);
				_delay_us(100);
				flexsensor2[i] = adc_start2(3);
				_delay_us(100);
			}
			for(int j = 0; j < 5; j++)
			{
				flexvalue1 += flexsensor1[j];
				flexvalue2 += flexsensor2[j];
			}
			
			flexvalue1 = (int)(flexvalue1 / 5) / 10;
			flexvalue2 = (int)(flexvalue2 / 5) / 10;
		
		if(flexvalue1 - flexvalue2 < 4)
		{
			motor1 = (int)(625 - ((47 - flexvalue1)*265/17));
			OCR1A = motor1;
			_delay_ms(1);
		
		}
		mpu6050();
		_delay_ms(10);
	} 
}

ISR(TIMER1_OVF_vect)
{
	sei();
	TCNT1 = 0;
}

ISR(TIMER3_OVF_vect)
{
	sei();
	TCNT3 = 0;
}

ISR(TIMER0_OVF_vect)	//0.002s
{
	sei();
	dt += 0.002;

	TCNT0 = 256-125;
}

void mpu6050()
{
	get_raw_data();

		las_angle_gx = roll;	//최근값 누적
		las_angle_gy = pitch;
		las_angle_gz = yaw;

		temp = (a_x_h<<8) | a_x_l;
		a_x = - temp - 16383;
		temp = (a_y_h<<8) | a_y_l;
		a_y = - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z = temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x = temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y = temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z = temp;

		g_x = (g_x - bas_g_x)/FS_SEL;
		g_y = (g_y - bas_g_y)/FS_SEL;
		g_z = (g_z - bas_g_z)/FS_SEL;
		
		angle_ax = atan(-1.000*a_y/sqrt(pow(a_x,2) + pow(a_z,2)))*180/3.141592;
		angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2)))*180/3.141592;

		angle_gx = g_x*dt + las_angle_gx;
		angle_gy = g_y*dt + las_angle_gy;
		angle_gz = g_z*dt + las_angle_gz;

		dt = 0.000;

		alpha = 0.90;
		roll = alpha*angle_gx + (1.000 - alpha)*angle_ax;
		pitch = alpha*angle_gy + (1.000 - alpha)*angle_ay;
		yaw = angle_gz;


		/*
		USART_Transmit_init4(a_x);
		USART_Transmit('\t');
		USART_Transmit_init4(a_y);
		USART_Transmit('\t');
		USART_Transmit_init4(a_z);
		USART_Transmit('\t');
		*/
		/*
		USART_Transmit_init4(g_x);
		USART_Transmit('\t');
		USART_Transmit_init4(g_y);
		USART_Transmit('\t');
		USART_Transmit_init4(g_z);
		USART_Transmit('\t');
		*/

		roll_1 = roll + 100 -48;
		pitch_1 = pitch + 200 - 48;
		if(roll_1 < (58 - 48))
			roll_1 = 58 - 48;
		if(roll_1 > (143 - 48))
			roll_1 = 143 -48;
		if(pitch_1 < (119 -48))
			pitch_1 = 119 -48;
		if(pitch_1 > (184 -48))
			pitch_1 = 184 - 48;
				
		motor2 = (int)(250 + (((184-48) - pitch_1)*300/65));
		/*motor3 = (int)(220 + (((147 -48) - roll_1)*400/85));*/
		motor4 = (int)(620 - (((184 - 48) - pitch_1)*400/65));
			
		OCR1B = motor2;
		_delay_ms(1);
		/*OCR3A = motor3;
		_delay_ms(1);*/
		OCR3B = motor4;
		_delay_ms(1);

		/*USART_Transmit_init4(roll);
		USART_Transmit('\t');
		USART_Transmit_init4(pitch);
		USART_Transmit('\t');
		USART_Transmit_init4(yaw);
		USART_Transmit('\t');
		USART_Transmit_init4(roll_1);
		USART_Transmit('\t');
		USART_Transmit_init4(pitch_1);
		USART_Transmit('\t');
		USART_Transmit_init4(motor3);
		USART_Transmit('\t');
		USART_Transmit_init4(motor4);
		USART_Transmit('\t');
		USART_Transmit('\r');
		
		_delay_ms(100);*/
}

void UART_init()
{
	UCSR0A = 0x00;
	UCSR0B = (1<<TXEN0); 
	UCSR0C = (3<<UCSZ00);  
	UBRR0H = 0;
	UBRR0L = 103;
}

void IO_init()
{
	PORTA = 0xFF;
	DDRA = 0xFF;
	PORTC = 0xFF;
	DDRC = 0xFF;
	DDRD = 0x1C;
	DDRB = 0b01100000;
	DDRE = 0b00011000;
}

void ADC_init1()
{
	ADMUX=0b01000000;
	ADCSRA=0b11000111;
}

void ADC_init2()
{
	ADMUX=0b01000001;
	ADCSRA=0b11000111;	
}

unsigned int read_adc(unsigned char adc_input)
{
	unsigned int ANALOG_VALUE=0;
	ADMUX=adc_input|(ADC_VREF_TYPE);
	ADCSRA |= 0x40;
	while((ADCSRA&0x10)==0);
	ADCSRA&=0b11101111;
	ANALOG_VALUE=ADCL+(ADCH<<8);
	return ANALOG_VALUE;
}

unsigned int adc_start1(unsigned char ch)
{
	unsigned int Value;
	ADC_init1();
	Value = read_adc(ch);
}

unsigned int adc_start2(unsigned char ch)
{
	unsigned int Value;
	ADC_init2();
	Value = read_adc(ch);
}

void TIMER_init1()
{	
	TCCR0 = (1<<CS02)|(1<<CS01);	//256 분주 
	TCCR1A = 0b10100010;
	TCCR1B = 0b00011011;
	TCCR3A = 0b10100010;
	TCCR3B = 0b00011011;
	TIMSK = 0b00000101;
	ETIMSK = 0x04;
	TCNT0 = 256-125;
	ICR1 = 4999;
	ICR3 = 4999;
	asm("sei");
}

void TIMER_init2()
{
	TCCR0 = (1<<CS02)|(1<<CS01);	//256 분주 
	TCNT0 = 256-125;				//125 번 => 0.002s
	TIMSK = (1<<TOIE0);
	asm("sei");
}

void TIMER_init3()
{
	TCCR3A = 0b10100010;
	TCCR3B = 0b00011011;
	ETIMSK = 0x04;
	ICR3 = 4999;
	asm("sei");
}


void delay_us(unsigned int time)
{
	unsigned int ns_i;
	for(ns_i = 0; ns_i<time; ns_i++){
		asm("nop");
	}
}

void delay_ms(unsigned int time)
{
	unsigned int ns_i;
	for(ns_i = 0; ns_i<time; ns_i++){
		delay_us(1000/14);
	}
}

void calibrate()	//초기값 읽기 
{
	int cal = 10;

	for(int i=0; i<cal; i++)	//평균 
	{
		get_raw_data();
	
		temp = (a_x_h<<8) | a_x_l;
		a_x += - temp - 16383;
		temp = (a_y_h<<8) | a_y_l;
		a_y += - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z += temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x += temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y += temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z += temp;

		_delay_ms(100);
	}	
	
	a_x /= cal;
	a_y /= cal;
	a_z /= cal;
	g_x /= cal;
	g_y /= cal;
	g_z /= cal;

	bas_a_x = a_x;	//초기 값으로 저장 
	bas_a_y = a_y;
	bas_a_z = a_z;
	bas_g_x = g_x;
	bas_g_y = g_y;
	bas_g_z = g_z;

}

void get_raw_data()
{
	a_x_h = twi_read(0x3B);		//x축 가속도
	a_x_l = twi_read(0x3C);
	a_y_h = twi_read(0x3D);		//y축 가속도 
	a_y_l = twi_read(0x3E);		
	a_z_h = twi_read(0x3F);		//z축 가속도 
	a_z_l = twi_read(0x40);		
	g_x_h = twi_read(0x43);		//x축 각속도 
	g_x_l = twi_read(0x44);		
	g_y_h = twi_read(0x45);		//y축 각속도 
	g_y_l = twi_read(0x46);		
	g_z_h = twi_read(0x47);		//z축 각속도 
	g_z_l = twi_read(0x48);		
}

void twi_write(unsigned char address,unsigned char data)
{ 
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT))); 
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	TWDR = data; 				 //data 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송  

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP
} 

unsigned char twi_read(char address)
{ 
	unsigned char data;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT))); 
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//Repeat START

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

	TWDR = 0b11010001;			 //AD(1101000)+R(1) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림 

	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림 

	data = TWDR; 

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP

	return data; 
}

void USART_Transmit(unsigned char tx_data)
{ 
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = tx_data; 
}

void USART_Transmit_init4(int data)
{
	if(data < 0)
	{
		data = -data;
		USART_Transmit('-');
	}
	else
		USART_Transmit(' ');

	int temp = 0;
	temp = data/10000;
	USART_Transmit(temp+48);
	temp = (data%10000)/1000;
	USART_Transmit(temp+48);
	temp = (data%1000)/100;
	USART_Transmit(temp+48);
	temp = (data%100)/10;	
	USART_Transmit(temp+48);
	temp = data%10; 
	USART_Transmit(temp+48);
}