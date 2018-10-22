#include <iostream>
#include <unistd.h>
#include "PWM.h"

using namespace std;
using namespace exploringBB; 

int main(int argc, char *argv[])
{
	int altitude = 0;
	altitude = 25000;
	if (altitude == 25000)  // set altitude you want to release capsule
	{
		// sets the speed of the motors
		int PWM_choose = 0;  // to set to either A or to B
		PWM pwm("pwm-2:0", PWM_choose, "2");   // this is the file created 
		pwm.setPeriod(10000000);   //set the period in ns
		pwm.setDutyCycle(100.0f);   //can use percentage or time in ns
		pwm.setPolarity(PWM::ACTIVE_LOW);   //ACTIVE_LOW or ACTIVE_HIGH
		pwm.run();   //start the output
		char dir = 'o';
		
		if (dir == 'o')
		{
			// sets direction to open
			PWM_choose = 0;     // set to PWM0A OPEN
			PWM pwm("pwm-0:0", PWM_choose, "0");      // this is the file created 
			pwm.setPeriod(1000000);      //set the period in ns
			pwm.setDutyCycle(95.0f);      //can use percentage or time in ns
			pwm.setPolarity(PWM::ACTIVE_LOW);      //ACTIVE_LOW or ACTIVE_HIGH
			pwm.run();      //start the output
			usleep(10000000);
			pwm.stop();
			dir = 'c';
		}
		
		usleep(1000000); // wait 1 second 
		
		
		if (dir == 'c')
		{
			// sets direction to close
			int PWM_choose = 1;      // set to PWM2B CLOSE
			PWM pwm("pwm-4:1", PWM_choose, "4");       // this is the file created 
			pwm.setPeriod(1000000);       //set the period in ns
			pwm.setDutyCycle(95.0f);       //can use percentage or time in ns
			pwm.setPolarity(PWM::ACTIVE_LOW);       //ACTIVE_LOW or ACTIVE_HIGH
			pwm.run();       //start the output
			usleep(10000000);
			pwm.stop();
		}
	}
}