/********************************************************************
    Solution File for lab2.c
********************************************************************/

// Global Variables
	long z = 0;


void main(void)
{

	int x = 2;
	int y = 7;

	z = x + y;

	while(1) asm(" NOP");	//trap end of main

} //end of main

// end of Lab2.c
