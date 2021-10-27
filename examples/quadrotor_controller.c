#define F_CPU 16000000UL // Clock Speed
#define F_OSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR F_OSC/16/BAUD-1
#define MPU6050 (0x68<<1)
#define GYRO_X_ADDR 0x43
#define GYRO_Y_ADDR 0x45
#define GYRO_Z_ADDR 0x47
#define ACC_X_ADDR 0x3B
#define ACC_Y_ADDR 0x3D
#define ACC_Z_ADDR 0x3F
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include <math.h>

uint16_t tg1,tg2,tg3,tg4; uint32_t s;  uint16_t zu,zm,tg, t3,t2,t1,t4;
volatile char son_kanal_4,son_kanal_3,son_kanal_2,son_kanal_1;
uint16_t esc1, esc2, esc3, esc4;
uint16_t pw1, pw2, pw3, pw4;
int USART_Transmit(unsigned char data,FILE *stream);
FILE uart_str = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_RW); //printf ile gönderilecek veri için buffer dosya aç.
void USART_Init( unsigned int ubrr)
{
	/*Baudrateyi aç */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*RX TX Aç */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* 8data, 2stop bit */
	
}

int USART_Transmit( unsigned char data,FILE *stream )
{
	/* Veri almaya hazýr mý?*/
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Veriyi buffera koyup yolla */
	UDR0 = data;
	return 0;
}

void MPU6050_Baslat(void) {
	i2c_start_wait(MPU6050+I2C_WRITE); //MPU6050'nin adresi ile haberleþme baþlat.
	i2c_write(0x6b); //Güç Register'ýna yaz
	i2c_write(0x00); //0 deðerini yaz
	i2c_stop(); //Haberleþmeyi bitir
	
	i2c_start_wait(MPU6050+I2C_WRITE); //MPU6050'nin adresi ile haberleþme baþlat.
	i2c_write(0x1b); //Jiroskop config registerý
	i2c_write(0x08);  //500d/s aralýðýna geç.
	i2c_stop(); //Haberleþmeyi bitir
	
	i2c_start_wait(MPU6050+I2C_WRITE);
	i2c_write(0x1C);
	i2c_write(0x10);  //+-8g aralýðýna geç.
	i2c_stop();
}

float GyroErr(char addr) {
	float GyroXh=0;
	for (int i=0; i<2000; i++)
	{
		i2c_start_wait (MPU6050+I2C_WRITE);	//MPU6050 adresi
		i2c_write (addr); //Okunacak olan register
		i2c_start (MPU6050+I2C_READ ); //Okuma iþlemini baþlat
		GyroXh += ((i2c_readAck() << 8) | i2c_readNak());	//16-biti birleþtir.
		i2c_stop(); } //Haberleþmeyi durdur.
	GyroXh=GyroXh/2000; //Son okunan deðeri 2000'e böl.
	printf("%#8X \rAdresindeki \rHata: %f\n",addr,GyroXh);	//Okunan adres ve deðeri yazdýr.
	return GyroXh;
}


float GyroOku (char addr) {
	float Gyro=0;
	i2c_start_wait (MPU6050+I2C_WRITE);
	i2c_write (addr);
	i2c_start (MPU6050+I2C_READ );
	Gyro= (i2c_readAck() << 8) | i2c_readNak();
	i2c_stop();
	return Gyro;
}

ISR(TIMER0_OVF_vect){
	s++;
	}

ISR(PCINT2_vect) {
	zu=s*127.5;
	tg=zu;
	if (PIND&(1<<PIND7))
	{
		if (son_kanal_3==0 )
{
			son_kanal_3=1;
			tg3=tg;
		//	printf("Interrupt HIGH\n");
}
	}
	else if(son_kanal_3==1)	//Ard D7
	{
		son_kanal_3=0;
		t3=zu-tg3; //us cinsinden zaman
	//	printf("Interrupt LOW\n");
	}
	
	if (PIND&(1<<PIND6))
	{
		if (son_kanal_2==0)
{
			son_kanal_2=1;
			tg2=tg;
			//	printf("Interrupt HIGH\n");
}
	}
	else if(son_kanal_2==1)	//Ard D6
	{
		son_kanal_2=0;
		t2=zu-tg2; //us cinsinden zaman
		//	printf("Interrupt LOW\n");
	}
	
	if (PIND&(1<<PIND5))
	{
		if (son_kanal_1==0 )
{
			son_kanal_1=1;
			tg1=tg;
			//	printf("Interrupt HIGH\n");
}
	}
	else if(son_kanal_1==1)	//Ard D5
	{
		son_kanal_1=0;
		t1=zu-tg1; //us cinsinden zaman
		//	printf("Interrupt LOW\n");
	} 
	
	if (PIND&(1<<PIND4))
	{
		if (son_kanal_4==0 )
{
			son_kanal_4=1;
			tg4=tg;
			//	printf("Interrupt HIGH\n");
}
	}
	else if(son_kanal_4==1)		//Ard D4
	{
		son_kanal_4=0;
		t4=zu-tg4; //us cinsinden zaman
		//	printf("Interrupt LOW\n");
	}
}

void Timer0_Baslat(void) {
	TCNT0=0;	//Timer0 registerýný sýfýrla
	TIFR0=0;	//Timer0 flagýný sýfýrla
	TCCR0A=0x00;	
	TCCR0B|=(1<<CS01); //Timer0 için 8 prescale
	TIMSK0|=(1<<TOIE0); //Timer0 Interrupt Açýk
}


void escYaz2(void) {
//	PORTD |= (1<<PIND3);
	PORTB |= 0b00011110; //           00011110=30
	uint32_t basla=s;
	while (PORTB>0)
	{		
		if ((s-basla)*127.5>=esc1)
		{
		//	printf("t: %f\n", (s-basla)*127.5);
			PORTB &= ~(1<<PINB1);
		}
		if ((s-basla)*127.5>=esc2)
		{
			PORTB &= ~(1<<PINB2);
		} 
		if ((s-basla)*127.5>=esc3)
		{
			PORTB &= ~(1<<PINB3);
		}
		if ((s-basla)*127.5>=esc4)
		{
			PORTB &= ~(1<<PINB4);
		}
	}


}

uint16_t esc_Limit(uint16_t esc, uint16_t min, uint16_t max) {
	uint16_t escout;
	if (esc<min)
	{
		escout=min;
	}
	
	else if (esc>max)
	{
		escout=max;
	}
	
	else {
		escout=esc;
	}
	return escout;
}


void esc_Baslat2(void) {
	uint32_t basla=s;
	while ((s-basla)*0.1275<=5000)
{
	//	printf("Burda: %f\n",(s-basla)*0.1275);
		PORTB |= 0b00011110;
		_delay_ms(0.89);
		PORTB &= 0b11100001;
		_delay_ms(3);
}
}


int main( void )
{
	float err1, err2, err3, err1t, err2t, err3t, derr1, ierr1, derr2, ierr2, derr3, ierr3;
//	float kp=1.2, kpy=0.1, ki=0.05, kd=1.5, kpr=0.03;	//Bu ayarlar güzel
	float kp=1.2, kpy=0.1, ki=0.05, kd=1.5, kpr=0.03;
	float GyroX=0; float GyroY=0; float GyroZ=0;
	float AccX=0; float AccY=0; float AccZ=0; float acctop;
	float pitch_acc=0; float roll_acc=0;
	float pitch_gyro=0; float roll_gyro=0; float yaw_out=0;
	float pitch_out=0; float roll_out=0;
	float pitch_eski=0; float roll_eski=0;
	float GyroXh=0; float GyroYh=0; float GyroZh=0;
	int rolld, pitchd, yawd, thr, imax=120, imin=-120;		//isat=100 güzel
	char gyrohazir=0; 	uint32_t s2=0; float dt=0.005; int test=0;
	float roll_ratec, pitch_ratec, yaw_ratec;
	DDRD &= ~(1 << PIND7); //7 numara input
	DDRD|=(1<<PIND3); //PIND3 output
	DDRB|=(1<<PINB1)|(1<<PINB2)|(1<<PINB3)|(1<<PINB4);	//PINB1, PINB2, PINB3 output
	DDRC|=(1<<PINC0)|(1<<PINC1);
	stdout = &uart_str;
	USART_Init(MYUBRR);
	i2c_init();
	MPU6050_Baslat();
	GyroXh=GyroErr(GYRO_X_ADDR)/65.5; GyroYh=GyroErr(GYRO_Y_ADDR)/65.5; GyroZh=GyroErr(GYRO_Z_ADDR)/65.5;
	sei(); //Global interrupt açýk
	PCMSK2|=(1<<PCINT23)|(1<<PCINT22)|(1<<PCINT21)|(1<<PCINT20);	//23-20 numaralý pin interrupt pinlerini (PD7-..-PD4) aktifleþtir.
	PCICR|=(1<<PCIE2);	//23-16 numaralý pinler  için interrupt'ý aktifleþtir.
	Timer0_Baslat();
	esc_Baslat2();
	
	
	while (1)
	{
		//	PORTC|=(1<<PINC0);
			/////////////////////////////////Açýlarý Hesapla //////////////////////////////////////
			GyroX=GyroOku(GYRO_X_ADDR)/65.5-GyroXh; GyroY=GyroOku(GYRO_Y_ADDR)/65.5-GyroYh; GyroZ=GyroOku(GYRO_Z_ADDR)/65.5-GyroZh;
			pitch_gyro=GyroX*dt; roll_gyro=GyroY*dt; yaw_out+=GyroZ*dt; //Gyrodan hesaplanan anlýk açý deðerleri
		//	printf("\r\nPitch_gyro: %.2f||", GyroX); //printf("\rRoll Gyro: %.2f||", GyroY); printf("\rYaw_gyro: %.2f||", GyroZ);	
			AccX=GyroOku(ACC_X_ADDR); AccY=GyroOku(ACC_Y_ADDR); AccZ=GyroOku(ACC_Z_ADDR);	//Ývmeölçer ivme deðerleri
			acctop=sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);		//Toplam ivme
			pitch_acc=asin(AccY/acctop)*57.324;			//Ývme ölçerden hesaplanan pitch açýsý
			roll_acc=asin(AccX/acctop)*(-57.324);		//Ývme ölçerden hesaplanan roll açýsý
			roll_acc-=-9.92;	pitch_acc-=-1.87;	//Ývmeölçer ile okunan hata deðerleri offsetlemesi
		//	printf("\r\nRoll_acc: %.2f||",roll_acc); printf("\rPitch_acc: %.2f||",pitch_acc);	//Acc açý debug
		//	printf("%.2f\n",pitch_acc);
			if(gyrohazir){		//Gyro açý okumaya hazýr mý?
				pitch_out=(pitch_gyro+pitch_eski)*0.998+pitch_acc*0.002;	//Tümleyen filtre
				roll_out=(roll_gyro+roll_eski)*0.998+roll_acc*0.002;		//Tümleyen filtre
			}
			else {				//Baþlangýçta ivmeölçer açý deðerini doðru kabul et.
				pitch_out=pitch_acc;
				roll_out=roll_acc;
				gyrohazir=1;
			}
			pitch_eski=pitch_out;
			roll_eski=roll_out;
			printf("\r\nPitch: %.2f||", pitch_out); printf("\rRoll: %.2f||", roll_out); printf("\rYaw: %.2f||\n", yaw_out);	//Son çýkýþ açýlarý debug
			///////////////////////////////////////////////////////////////////////////////////////////
			
			////////////////////////////////////Kontrol Bloðu////////////////////////////////////////////
		//	printf("%d\n",t2);
			rolld = 0.06*t1-90; pitchd = 0.06*t2-90; thr = t3; yawd = 0.06*t4-90; //Kumandadan istenen açýlar
		//	printf("rolld:%d||",rolld); printf("pitchd:%d||",pitchd); printf("yawd:%d||\n",yawd);	//Kumanda açý debug
			roll_ratec=kpr*(rolld-roll_out)/dt;	pitch_ratec=kpr*(pitchd-pitch_out)/dt;	yaw_ratec=(yawd-yaw_out)/dt;  //Açý --> rate
		//	printf("rollratec=%d||",roll_ratec); printf("pitchratec=%d||",pitch_ratec);	printf("yawratec=%d||\n",yaw_ratec); //Rate debug
			thr=esc_Limit(thr,1000,1600);
		//	printf("Gaz:%d\n",thr);		//Throttle debug
			err1=(pitch_ratec-GyroX); err2=(roll_ratec-GyroY); err3=(yaw_ratec-GyroZ); //Hatalar
			ierr1 += err1; ierr2 += err2; //Hatanýn integrali
			if (ierr1>imax)
			{
				ierr1=imax;
			}
			
			else if (ierr1<imin)
			{
				ierr1=imin;						//I kontrolcü için doyma bloðu
			}
			
			if (ierr2>imax)
			{
				ierr2=imax;
			}
			
			else if (ierr2<imin)
			{
				ierr2=imin;
			} 
			
			derr1=err1-err1t; derr2=err2-err2t; //Hatanýn türevi
			err1t=err1; err2t=err2; //Eski hata deðerini sakla
			esc1=thr+err1*kp+kd*derr1+ki*ierr1-kpy*err3+65;	esc3=thr-(err1*kp+kd*derr1+ki*ierr1+kpy*err3); //Pitch kontrol +82
			esc2=thr+err2*kp+kd*derr2+ki*ierr2+kpy*err3+45;	esc4=thr-(err2*kp+kd*derr2+ki*ierr2-kpy*err3);		//Roll kontrol +107
		//	printf("err1: %f||derr1: %f||ierr1: %f||Top: %f\n",kp*err1, kd*derr1, ki*ierr1, kp*err1+kd*derr1+ki*ierr1);
			esc1=esc_Limit(esc1,1100,1900); esc2=esc_Limit(esc2,1100,1900); 
			esc3=esc_Limit(esc3,1100,1900); esc4=esc_Limit(esc4,1100,1900);	//Doyma bloðu 
		//	printf("hata:%f\n",(pitch_ratec-GyroX)*kp);			
		//	printf("esc1:%d||",esc1); printf("esc2:%d||",esc2); printf("esc3:%d||",esc3); printf("esc4:%d||\n",esc4);	//ESC debug
		/*	while (t3<1100)			//Gaz aþaðýda iken hiçbir þey yapma
			{
				PORTB |= 0b00011110;
				_delay_ms(0.95);
				PORTB &= 0b11100001;
				_delay_ms(4);
			} */
			escYaz2();
			//////////////////////////////////////////////////////////////////////////////////////////////
			dt=(s-s2)*0.0001275; //Döngü süresi (s)		//printf kullanýrken açýlmasý gerek
			while((s-s2)*0.1275<5){		//Süre 5  ms'den küçükse burada dön.
				
			}
			s2=s;				//Bir sonraki döngünün baþlangýç süresi
		//	PORTC &= ~(1<<PINC0);
		//	_delay_ms(1); 
			
	}
	
}

