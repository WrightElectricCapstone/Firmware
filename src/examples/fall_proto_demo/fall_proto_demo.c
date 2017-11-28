

/**
 * @file fall_proto_demo.c
 * application demonstrating fine servo and motor control
 * part of the Wright Electric capston fall prototype demo
 *
 * @author Alexander Wolff <alexander_wolff@umail.ucsb.edu>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <drivers/drv_hrt.h>


#define TICK_INTERVAL 10000
#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000
#define MID_THROTTLE 1500
#define FULL_PITCH_UP 2000
#define FULL_PITCH_DOWN 1000
#define NEUTRAL_PITCH 1500

/*
*Initialize UART
*@param uart_name = the name of the uart which you want to init
*/
int uart_init(char* uart_name);
/*
*Set the baudrate for the uart
*@param fd = the UART number (this is the value returned by uart_init)
*@param baud = the baud rate to set the UART to, this is probably 57600 for any Pixhawk 2 port
*/
int set_uart_baudrate(const int fd, unsigned int baud);
/*
*Sets channel number to tagret on the servo controller connected to UART number
*@param uartNumber = the uart which the servo controller is connected on (must be open)
*@param channelNumber = the channel on the servo controller that the servo is on
*@param target = pulse width in quarter microseconds
*/
bool setTargetCP(int uartNumber, unsigned char channelNumber, unsigned short target );


/*
*Global variables tracking the current servo outputs and the new servo outputs
*/
int CurrPitch;
int TargetPitch;
int CurrThrottle;
int TargetThrottle;


/*
*For Timer Interrupt to update servo updates
*/
struct hrt_call _call;

/*
*Throttle State Machine
*THROTTLE_UP , throttle setting is increasing towards targetthrottle
*THROTTLE_DOWN, throttle setting is decreasing towards target throttle
*THROTTLE_HOLD, throttle is set to target throttle
*THROTTLE_STOP, emergency state, something is wrong
*/
enum ThrottleState{
	THROTTLE_UP,
	THROTTLE_HOLD,
	THROTTLE_DOWN,
	THROTTLE_STOP
}ThrottleState;


/*
*Pitch State Machine
*PITCH_UP , pitch setting is increasing towards target pitch
*PITCH_DOWN, pitch setting is decreasing towards target pitch
*PITCH_HOLD, pitch is set to target pitch
*/
enum PitchState{
	PITCH_UP,
	PITCH_HOLD,
	PITCH_DOWN,
}PitchState;


/*
*Signals which can be dispatched to state machine
*TICK, system time increment, update servo outputs, interval dictated by granularity
*/
enum Signals{
	TICK
}Signals;

/*
*Variables tracking time to target transition
*/
int ElapsedTicks = 0;
int ResumeTick = 0;


/*
*Sends a signal to the state machine, calls the appropriate state handlers and passes Sig to state machine
*/
int Dispatch(int Sig);
/*
*State Handlers
*/
int ThrottleUp(int Sig);
int ThrottleDown(int Sig);
int ThrottleHold(int Sig);
int ThrottleStop(int Sig);
int PitchUp(int Sig);
int PitchDown(int Sig);
int PitchHold(int Sig);
/*
*Timer Interrupt Handler
*/
void TimerHandler(void* arg);


//allow this to be executed from shell
__EXPORT int fall_proto_demo_main(int argc, char *argv[]);

//main
int fall_proto_demo_main(int argc, char *argv[])
{

	printf("%s\n","Welcome to the Wright Electric Fall Prototype Demo!");
	
	//make sure callback is not already active
	hrt_cancel(&_call);
	//Connect Timer Handler and begin executing every TICK_INTERVAL
	hrt_call_every(&_call,1,TICK_INTERVAL,(hrt_callout)&TimerHandler, NULL);

	//initial outputs	
	CurrThrottle = MIN_THROTTLE;
	CurrPitch = NEUTRAL_PITCH; 

	//write initial state to servo controller, this will be uncommented once servo controller is connected to pixhawk
	//int uart = uart_init("/dev/ttyS3");
	//set_uart_baudrate(uart, 57600);
	//setTargetCP(uart, 0, 4000)


	//begin takeoff
	TargetThrottle = MAX_THROTTLE;
	TargetPitch = FULL_PITCH_UP;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//Maintain pitch, not trying to loop out
	TargetPitch = NEUTRAL_PITCH;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//shift to cruise throttle and bring nose level
	TargetThrottle = MID_THROTTLE;
	TargetPitch = FULL_PITCH_DOWN;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//cruise
	TargetPitch = NEUTRAL_PITCH;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//begin descent
	TargetPitch = FULL_PITCH_DOWN;
	TargetThrottle = MIN_THROTTLE;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//reached angle of attack
	TargetPitch = NEUTRAL_PITCH;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//flare point reached
	TargetPitch = FULL_PITCH_UP;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//land
	TargetPitch = NEUTRAL_PITCH;
	ResumeTick = 1000;
	ElapsedTicks = 0;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}


	PX4_INFO("exiting");

	return 0;
}




int Dispatch(int Sig)
{
	switch(ThrottleState)
	{
		case THROTTLE_UP:
		{
			ThrottleUp(Sig);
			break;
		}
		case THROTTLE_DOWN:
		{
			ThrottleDown(Sig);
			break;
		}
		case THROTTLE_HOLD:
		{
			ThrottleHold(Sig);
			break;
		}
		case THROTTLE_STOP:
		{
			ThrottleStop(Sig);
		}

	}
	switch(PitchState)
	{
		case PITCH_UP:
		{
			PitchUp(Sig);
			break;
		}
		case PITCH_DOWN:
		{
			PitchDown(Sig);
			break;
		}
		case PITCH_HOLD:
		{
			PitchHold(Sig);
			break;
		}

	}
	return 0;		
}



void TimerHandler(void* arg)
{
	Dispatch(TICK);
}

int ThrottleUp(int Sig)
{
	if(CurrThrottle > TargetThrottle)
	{
		ThrottleState = THROTTLE_DOWN;
	}
	else if(CurrThrottle == TargetThrottle)
	{
		ThrottleState = THROTTLE_HOLD;
	}
	else if(Sig == TICK)
	{
		CurrThrottle += 1;
		ElapsedTicks += 1;
	}
	//write new outputs to servos
	return 0;
}

int ThrottleDown(int Sig)
{
	if(CurrThrottle < TargetThrottle)
	{
		ThrottleState = THROTTLE_UP;
	}
	else if(CurrThrottle == TargetThrottle)
	{
		ThrottleState = THROTTLE_HOLD;
	}
	else if(Sig == TICK)
	{
		CurrThrottle -= 1;
		ElapsedTicks += 1;
	}
	//write new outputs to servos
	return 0;
}

int ThrottleHold(int Sig)
{
	if(CurrThrottle < TargetThrottle)
	{
		ThrottleState = THROTTLE_UP;
	}
	else if(CurrThrottle > TargetThrottle)
	{
		ThrottleState = THROTTLE_DOWN;
	}
	else if (Sig == TICK)
	{
		ElapsedTicks += 1;
		//do nothing
	}
	//write new outputs to servos
	return 0;

}


int ThrottleStop(int Sig)
{
	CurrThrottle = 1000;
	return 0;
	if(Sig == TICK)
	{
		ElapsedTicks++;
	}
	//write new outputs to servos
}

int PitchUp(int Sig)
{
	if(CurrThrottle > TargetThrottle)
	{
		ThrottleState = THROTTLE_DOWN;
	}
	else if(CurrThrottle == TargetThrottle)
	{
		ThrottleState = THROTTLE_HOLD;
	}
	else if (Sig == TICK)
	{
		ElapsedTicks += 1;
		CurrThrottle += 1;
	}
	//write new outputs to servos
	return 0;
}

int PitchDown(int Sig)
{
	if(CurrThrottle < TargetThrottle)
	{
		ThrottleState = THROTTLE_UP;
	}
	else if(CurrThrottle == TargetThrottle)
	{
		ThrottleState = THROTTLE_HOLD;
	}
	else if (Sig == TICK)
	{
		ElapsedTicks += 1;
		CurrThrottle -= 1;
	}
	//write new outputs to servos
	return 0;
}

int PitchHold(int Sig)
{
	if(CurrThrottle < TargetThrottle)
	{
		ThrottleState = THROTTLE_UP;
	}
	else if(CurrThrottle > TargetThrottle)
	{
		ThrottleState = THROTTLE_DOWN;
	}
	else if(Sig == TICK)
	{
		ElapsedTicks += 1;
		//do nothing
	}
	//write new outputs to servos
	return 0;
}



int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}


int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}




bool setTargetCP(int uartNumber, unsigned char channelNumber, unsigned short target )
{
	unsigned char command[4] = { 0x84, channelNumber, target & 0x7F, (target >> 7) & 0x7F };
	write(uartNumber, command, sizeof(command));

	return true;
}




