/**@mainpage package optimal_path_in_multistage_graphs
 @author Group 15 Gokul Ramaswamy, Arjit Tiwari, Vijaykumar                     Agnihotri
 
 AVR Studio Version 4.17, Build 666

 Date: 8th November 2010
 
 This the core module of the project.

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 11059200
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)


*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include <stdlib.h>
#include "Node.c"



// Global variable
int LeftShaftCount, RightShaftCount;
struct Node;
int curAngle = 0;
int levelcount=1;
int lev1Count,lev2Count,lev3Count;
int i1,i2,i3;
struct Node node;
struct Node* temp[4];
int error=5;
char buff[5];
int final_index[4];

/** 0 for traversing,1 for back traversing*/
int flag[10]={0,0,0,0,0,0,0,0,0,0};

/**flag for start node. to avoid counting again*/
int first_time_flag=0; 
int distanceStack[3];
int distanceStackTop = -1,sum=0;
int finalPathMatrix[4][4];

/** Stores final paths seen so far - 1. To properly adjust with index  */
int finalPathsSeen = -1;	//

// Function declaration for functions in lcd.c
void init_devices();
void lcd_set_4bit();
void lcd_init();
void lcd_print (char row, char coloumn, unsigned int value, int digits);
void lcd_string(char *str);
void lcd_reset();



/**
*Self defined function
**/



void initNode(struct Node*);  // for node initialization
void start_level(); // for start node of graph
void level2(); // another level
void level3(); // another level
void final_level(); //final level




/**
*Port initiialization
**/

/**
* Initialization for Timer 5
*/
void timer5_init()
{
	TCCR5B=0x00;
	TCCR5A=0xA9;
	TCCR5B=0x0B;
	DDRE=0x18;
	PORTE=0x00;
}

/**
* Configuration for ADC Pins
*/
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

/**
* Initialization for ADC Pins
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


/**
* Used to control the Speed of the Left and the Right Wheels of the Bot
*/
void velocity(unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL=left_motor;
	OCR5BL=right_motor;
}


/**
* Port Initialization for Motion of the Bot
*/
void INIT_PORTS()
{
	DDRA=0x0F;
	PORTA=0x00;
	DDRL=0x18;
	PORTL=0x18;
}

/**
* Makes the robot move in the Forward Direction
*/
void FORWARD()
{
	PORTA=0x06;
}

/**
* Makes the robot to Stop
*/
void STOP()
{
	PORTA=0x00;
}

/**
* Makes the robot move in the Reverse dDrection
*/
void REVERSE()
{
	PORTA=0x09;
}

/**
* Makes the robot to rotate left
*/
void LEFT_TURN()
{
	PORTA=0x05;
}

/**
* Makes the robot to rotate Right
*/
void RIGHT_TURN()
{
	PORTA=0x0A;
}

/**
* Enables interupts for the Left wheel 
*/
void left_position_encoder_interrupt_init (void)  //Interrupt 4 enable
{
          cli(); //Clears the global interrupt
          EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
          EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
          sei();   // Enables the global interrupt 
}

/**
* Enables interupts for the Right wheel 
*/
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt 
}


ISR(INT4_vect) 
{
	LeftShaftCount++;  //increment left shaft position count
}

ISR(INT5_vect)
{
	RightShaftCount++;  //increment right shaft position count
}


/**
* Make the Bot move in the forarwd direction by the Distance of DistanceInMM
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
 	float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

 	ReqdShaftCount = DistanceInMM / 5.338; 
	// division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 	RightShaftCount = 0;
	FORWARD();
 	while(1)
 	{
  		if(RightShaftCount > ReqdShaftCountInt)
 		 {
  		break;
 		 }
	 } 
 STOP(); //Stop action
}

/**
* Make the Bot move in the reverse direction by the Distance of DistanceInMM
*/
void reverse_distance_mm(unsigned int DistanceInMM)
{
	
 	float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

 	ReqdShaftCount = DistanceInMM / 5.338; 
	// division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 	RightShaftCount = 0;
	REVERSE();
 	while(1)
 	{
  		if(RightShaftCount > ReqdShaftCountInt)
 		 {
  		break;
 		 }
	 } 
 STOP(); //Stop action
}

/**
* Function used for turning robot by specified degrees
*/
void angle_rotate(unsigned int Degrees, int dir)
{
	velocity(100,100);
    float ReqdShaftCount = 0;
    unsigned long int ReqdShaftCountInt = 0;

	//if(dir == 1)
	//	Degrees = Degrees * (165/180);
	//else if(dir == 2)
	//	Degrees = Degrees * (135/180);

    
    RightShaftCount = 0; //defined globle variable
    LeftShaftCount = 0; 

	if(dir == 1)
	{
		ReqdShaftCount = (float) Degrees * 155 / 4.090 / 180; // division by resolution to get shaft count
    	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 
		LEFT_TURN();	
	}
	else if(dir == 2)
	{
		ReqdShaftCount = (float) Degrees * 145 / 4.090 / 180; // division by resolution to get shaft count
    	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 

		RIGHT_TURN();

	}
	//curAngle += Degrees;
	

 while (1)
  {

   if((RightShaftCount >= ReqdShaftCountInt) || (LeftShaftCount >= ReqdShaftCountInt))
      break;
  }
   STOP(); //Stop action
}


unsigned char ADC_Conversion_(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
		{
		ADCSRB = 0x08;			// select the ch. > 7
		}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		//do not disturb the left adjustment
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; 		//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


/**
* Main function
* The starting point of the project
*/
int main()
{
	INIT_PORTS();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();

	timer5_init();


	cli(); //Clears the global interrupts
	adc_pin_config();
	adc_init();
	sei(); //Enables the global interrupts


	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	//////////start/////////////

	while (levelcount!=5)
	{


		if (levelcount==1)
			start_level();


		if (levelcount==2)
			level2();


		if (levelcount==3)
			level3();


		if (levelcount==4)
			final_level();

	}


}
void final_move()
{
	levelcount=0;
	struct node* final;
	temp[levelcount]=&final;
	lev1Count = edgeDetect(temp[levelcount]);
	rotateSkipping(lev1Count-final_index[0]);
	followWhiteLineTillNode();
	linear_distance_mm(60);

	angle_rotate((temp[levelcount]->angles[final_index[0]])-10,2);
	levelcount++;
	lev2Count = edgeDetect(temp[levelcount]);
	rotateSkipping(lev2Count-final_index[1]);
	followWhiteLineTillNode();
	linear_distance_mm(60);

	angle_rotate((temp[levelcount]->angles[final_index[1]])-10,2);
	levelcount++;
	lev3Count = edgeDetect(temp[levelcount]);
	rotateSkipping(lev3Count-final_index[2]);
	followWhiteLineTillNode();
	exit(0);

}

/**
* Prints the Shortest Path on LCD after traversing the entire Graph
*/
void printMatrix()
{
	int minPath = 10000;	//TODO: Set The minPath greater than the largest path possible
	int minIndex = -1;
	for(int y=0; y<4; y++)
	{
		if(finalPathMatrix[y][3] < minPath)
		{
			minPath = finalPathMatrix[y][3];
			minIndex = y;
		}
	}

	lcd_reset();
	lcd_cursor(1,1);
	lcd_string("Optimal Path:");
	
	lcd_cursor(2,1);
	for(int x =0; x<3; x++)
	{
		itoa(finalPathMatrix[minIndex][x],buff,10);
		final_index[x]=finalPathMatrix[minIndex][x];
		lcd_string(buff);		
		_delay_ms(2000);
	}

	itoa(minPath,buff,10);
	lcd_string(buff);
	_delay_ms(5000);




}



/**
*  Rotates clockwise by skipping n whitelines, Stops at the nth Whiteline
*/
void rotateSkipping(int n)
{
	unsigned char sharp1, sharp2, sharp3;
	int alreadyOnWhite = 0;	// Holds the number of outgoing edges 

	while(n > 0)
	{
		sharp1 = ADC_Conversion_(1);
		sharp2 = ADC_Conversion_(2);
		sharp3 = ADC_Conversion_(3);
		if(sharp2<15)
		{
			if(alreadyOnWhite == 0)
			{
				n--;
				alreadyOnWhite = 1;
			}
		}
		else
		{
			alreadyOnWhite = 0;		
		}
		
		velocity(95,95);
		angle_rotate(10, 2);
	}
	

}



/**
*  Bot rotates 180 degrees at a node and finds all the edges that are going out of that node
*/
int edgeDetect(struct Node* node)
{
	unsigned char sharp1, sharp2, sharp3;
	int count = 0;	// Holds the number of outgoing edges 
	lcd_reset();
	lcd_cursor(1,1);
	lcd_string("Scanning");

	// PRE-REQUISUITE:  the FB towards positive Y-Axis
	int ang, alreadyOnWhite = 0;
	
	// Polling:
	// Rotate it by 180 degrees in increments of 5 degrees
	// at each 5 degrees, check if an edge is present. If so, note the angle.
	for(ang=0; ang<=180+error; ang+=5)
	{

		sharp1 = ADC_Conversion_(1);
		sharp2 = ADC_Conversion_(2);
		sharp3 = ADC_Conversion_(3);

		if(sharp2<15)
		{
			if(alreadyOnWhite == 0)
			{
				((node)->validChildIndex)++;
				node->angles[count] = ang;
				node->children[count] = (struct Node*)malloc(sizeof(struct Node));
				initNode(node->children[count]);
				(node->children[count])->parent = node;

				count++;
				alreadyOnWhite = 1;
			}
		}	
		else
		{
			alreadyOnWhite = 0;
		}

		velocity(100,100);
		angle_rotate(5, 1);
		
	}


	char str[9] = "count: ";
	str[7] = '0' + count;
	str[8] = '\0';
	lcd_reset();
	lcd_cursor(2,1);
	lcd_string("Scanning Done.");
	_delay_ms(1000);
	lcd_cursor(1,1);
	lcd_string(str);
	reverse_distance_mm(20);
	velocity(100,100);
	return count;
	
}

/**
* Node Initialization function
*/
void initNode(struct Node* node)
{
	node->parent = NULL;
	
	node->children[0] = NULL;
	node->children[1] = NULL;
	node->children[2] = NULL;
	node->children[3] = NULL;	
		

	// Set all the angles to 0
	node->angles[0] = node->angles[1] = node->angles[2] = node->angles[3] = 0;			
	
	// Initially none of the children are visited.
	node->visited[0] = node->visited[1] = node->visited[2] = node->visited[3]=node->visited[4]=node->visited[5] = 0;	

	// index of the maximum child which is valid, initially it is -1
	node->validChildIndex = -1;	
}


/*
* Returns the total distance
* Follows the White line till a node is reached
*/
int followWhiteLineTillNode()
{
	unsigned char sharp1, sharp2, sharp3;
	lcd_reset();
	lcd_cursor(1,1);
	lcd_string("Following Edge");
	

	int startShaftCount = RightShaftCount;

	

	_delay_ms(1000);
	while(1)
	{
		sharp1 = ADC_Conversion_(1);
		sharp2 = ADC_Conversion_(2);
		sharp3 = ADC_Conversion_(3);

		if(sharp1>50&&sharp2<15)
		{
				velocity(45,110);		//Left
				FORWARD();
		}
		else if(sharp3>50&&sharp2<15)
		{
				velocity(110,45);		//Right
				FORWARD();
		}
		else if(sharp1<15 && sharp2<15 && sharp3<15)
		{
	
			//_delay_ms(6000);
			STOP();
			break;
		}
		else if(sharp2<15)
		{
				velocity(115,115);
				FORWARD();
		}
	}
	
	velocity(100,100);
	lcd_reset();

	
	lcd_cursor(1,1);
	itoa(((RightShaftCount - startShaftCount) * 5.338 / 10) + 11, buff,10);
	lcd_string("Distance: ");
	lcd_string(buff);


	lcd_cursor(2,1);
	lcd_string("Followed Edge");
	_delay_ms(1000);
	return (((RightShaftCount - startShaftCount) * 5.338 / 10) + 11); 
}



/**
* Function to handle the first level or the root node of the graph
*/
void start_level()
{	
	
	if(flag[levelcount]==0)
	{
		if(first_time_flag==0)
		{
			temp[levelcount]=&node;
			lev1Count = edgeDetect(temp[levelcount]);
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("Count: ");
			lcd_cursor(2,1);
			itoa(lev1Count, buff,10);
			lcd_string(buff);
			first_time_flag=1;
		}
		i1=0;
		while(temp[levelcount]->visited[i1]==1)
		{
			i1++;
			if (i1>=lev1Count)
			{
				lcd_cursor(2,1);
				lcd_string("Exiting");
				reverse_distance_mm(20);
				angle_rotate(190,2);
				free(temp[levelcount]);
				_delay_ms(1000);
				printMatrix();
				final_move();
								
			}
		}
		error=(i1+1)*7;
		temp[levelcount+1]=temp[levelcount]->children[i1];
		rotateSkipping(lev1Count-i1);
		lcd_reset();
		lcd_cursor(1,1);
		lcd_string("new child");
		lcd_cursor(2,1);
		itoa(temp[levelcount]->angles[i1], buff,10);
		lcd_string(buff);

		distanceStackTop++;									// Add the Path distance to stack
		distanceStack[distanceStackTop] = followWhiteLineTillNode();


		_delay_ms(1000);
		levelcount++;
		flag[levelcount]=0;
	}
	else
	{
		linear_distance_mm(45);
		angle_rotate((temp[levelcount]->angles[i1])-10,2);
		temp[levelcount]->visited[i1]=1;
		flag[levelcount]=0;
		_delay_ms(1000);
	}
}

/**
* Second Level (Any Intermediate Level)
* Code for Second Level. This code is applicable for any intermediate level, 
* in this case , the second level.
*/
void level2()
{
	
	if(flag[levelcount]==0)				//If First time coming to the Node.
	{
		
		//adjusting itself for counting
		linear_distance_mm(55);
		angle_rotate(temp[levelcount-1]->angles[i1]-error,2);
		//At Level 2
		lev2Count = edgeDetect(temp[levelcount]);
		lcd_reset();
		lcd_cursor(1,1);
		lcd_string("Count: ");
		lcd_cursor(2,1);
		itoa(lev2Count, buff,10);
		lcd_string(buff);
		i2=0;
		temp[levelcount+1]=temp[levelcount]->children[i2];
		rotateSkipping(lev2Count);
		lcd_reset();
		lcd_cursor(1,1);
		lcd_string("new child");
		lcd_cursor(2,1);
		itoa(temp[levelcount]->angles[i2], buff,10);
		lcd_string(buff);



		distanceStackTop++;									// Add the Path distance to stack
		distanceStack[distanceStackTop] = followWhiteLineTillNode();


		_delay_ms(1000);
		levelcount++;
		flag[levelcount]=0;
		
	}
	else
	{
		temp[levelcount]->visited[i2]=1;
		i2=0;
		while(temp[levelcount]->visited[i2]==1)
			i2++;
		if (i2>=lev2Count)
		{
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("bac gret");
			lcd_cursor(2,1);
			itoa(temp[levelcount]->angles[i2-1], buff,10);
			lcd_string(buff);
			linear_distance_mm(60);
			if((temp[levelcount-1]->angles[i1])>110)// here in place of 110 we should use 90 but to match the error we have chosen 110. it can vary.
			{
				if((temp[levelcount]->angles[lev2Count-1])>=(temp[levelcount-1]->angles[i1]))
					angle_rotate((360-((temp[levelcount]->angles[lev2Count-1])-(temp[levelcount-1]->angles[i1]))+5),2);
				else
					angle_rotate((360-((temp[levelcount-1]->angles[i1])-(temp[levelcount]->angles[lev2Count-1]))+5),2);
			}
			else
			{
				if((temp[levelcount]->angles[lev2Count-1])>=(temp[levelcount-1]->angles[i1]))
					angle_rotate(((temp[levelcount]->angles[lev2Count-1])-(temp[levelcount-1]->angles[i1])-15),2);
				else
					angle_rotate(((temp[levelcount-1]->angles[i1])-(temp[levelcount]->angles[lev2Count-1])-15),2);
			}
			
	 		rotateSkipping(1);	
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("Going back 1");

			followWhiteLineTillNode();
			distanceStack[distanceStackTop] = 0;		// Pop from Distance Stack 
			distanceStackTop--;							// during backtracking

			free(temp[levelcount]);
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("node gone");
			_delay_ms(1000);
			levelcount--;
			flag[levelcount]=1;
		}
		else 
		{
			lcd_reset();
			lcd_cursor(2,1);
			itoa(temp[levelcount]->angles[i2], buff,10);
			lcd_string(buff);
			_delay_ms(1000);
			temp[levelcount+1]=temp[levelcount]->children[i2];
			linear_distance_mm(50);
			angle_rotate(temp[levelcount]->angles[i2-1],2);
			_delay_ms(1000);
			rotateSkipping(lev2Count-i2);
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("next child 2");

			distanceStackTop++;												// Add the Path distance to stack
			distanceStack[distanceStackTop] = followWhiteLineTillNode();


			_delay_ms(1000);
			levelcount++;
			flag[levelcount]=0;
		}
	}
}


/**
* Third Level (Any Intermediate Level)
* Code for Third Level. This code is applicable for any intermediate level, 
* in this case , the third level.
*/
void level3()
{
	
	if(flag[levelcount]==0)
	{
		
		//adjusting itself for counting
		linear_distance_mm(60);
		angle_rotate(temp[levelcount-1]->angles[i2]-error,2);
		//At Level 3
		lev3Count = edgeDetect(temp[levelcount]);
		lcd_reset();
		lcd_cursor(1,1);
		lcd_string("Count: ");
		lcd_cursor(2,1);
		itoa(lev3Count, buff,10);
		lcd_string(buff);
		i3=0;
		temp[levelcount+1]=temp[levelcount]->children[i3];
		rotateSkipping(lev3Count);
		lcd_reset();
		lcd_cursor(1,1);
		lcd_string("new chid 3");


		distanceStackTop++;									// Add the Path distance to stack
		distanceStack[distanceStackTop] = followWhiteLineTillNode();


		_delay_ms(1000);
		levelcount++;
		flag[levelcount]=0;
		
	}
	else
	{
		temp[levelcount]->visited[i3]=1;
		i3=0;
		while(temp[levelcount]->visited[i3]==1)
			i3++;
		if (i3>=lev3Count)
		{
			lcd_reset();
			lcd_cursor(1,1);
			itoa(temp[levelcount]->angles[lev3Count-1], buff,10);
			lcd_string(buff);
			lcd_cursor(2,1);
			itoa(temp[levelcount-1]->angles[i2], buff,10);
			lcd_string(buff);
			linear_distance_mm(50);
			if((temp[levelcount-1]->angles[i2])>90)
			{
				if((temp[levelcount]->angles[lev3Count-1])>=(temp[levelcount-1]->angles[i2]))
					angle_rotate((360-((temp[levelcount]->angles[lev3Count-1])-(temp[levelcount-1]->angles[i2]))+5),2);
				else
					angle_rotate((360-((temp[levelcount-1]->angles[i2])-(temp[levelcount]->angles[lev3Count-1]))+5),2);
			}
			else
			{
				if((temp[levelcount]->angles[lev3Count-1])>=(temp[levelcount-1]->angles[i2]))
					angle_rotate(((temp[levelcount]->angles[lev3Count-1])-(temp[levelcount-1]->angles[i2])-25),2);
				else
					angle_rotate(((temp[levelcount-1]->angles[i2])-(temp[levelcount]->angles[lev3Count-1])-25),2);
			}
			rotateSkipping(1);
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("going back 2");

			followWhiteLineTillNode();
			distanceStack[distanceStackTop] = 0;		// Pop from Distance Stack 
			distanceStackTop--;	

			free(temp[levelcount]);
			levelcount--;
			flag[levelcount]=1;
		}
		else 
		{
			temp[levelcount+1]=temp[levelcount]->children[i3];
			linear_distance_mm(50);
			angle_rotate(temp[levelcount]->angles[i3-1],2);
			rotateSkipping(lev3Count-i3);
			lcd_reset();
			lcd_cursor(1,1);
			lcd_string("next child 3");
			_delay_ms(1000);

			distanceStackTop++;									// Add the Path distance to stack
			distanceStack[distanceStackTop] = followWhiteLineTillNode();

			levelcount++;
			//angle_rotate(temp[levelcount-1]->angles[i3],1);
			flag[levelcount]=0;
		}
	}
}



/**
* Final Level
* Code for Final Level
*/

void final_level()
{

	finalPathsSeen++;

	finalPathMatrix[finalPathsSeen][0] = i1;
	finalPathMatrix[finalPathsSeen][1] = i2;
	finalPathMatrix[finalPathsSeen][2] = i3;
	finalPathMatrix[finalPathsSeen][3] = 0;

	for(int levels=0;levels<3;levels++)
		finalPathMatrix[finalPathsSeen][3] += distanceStack[levels];

	

	lcd_reset();
	lcd_cursor(1,1);
	lcd_string("Rotating for back track");

	lcd_cursor(2,1);
	lcd_string("Ttl Dist:");
	itoa(finalPathMatrix[finalPathsSeen][3],buff,10);
	lcd_string(buff);

	velocity(100,100);
	linear_distance_mm(7);
	angle_rotate(170,2);
	rotateSkipping(1);

	followWhiteLineTillNode();
	distanceStack[distanceStackTop] = 0;		// Pop from Distance Stack 
	distanceStackTop--;	

	_delay_ms(1000);
	levelcount--;
	flag[levelcount]=1;
	lcd_cursor(2,1);
	itoa(levelcount, buff,10);
	lcd_string(buff);
	lcd_cursor(2,4);
	itoa(flag[levelcount], buff,10);
	lcd_string(buff);
}



