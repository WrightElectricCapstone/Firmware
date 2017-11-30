

/**
 * @file fall_proto_demo.c
 * application demonstrating fine servo and motor control
 * part of the Wright Electric capstone fall prototype demo
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
#define NUM_MOTORS 1
#define NUM_SERVOS 1

/*
*Initialize UART
*@param uart_name = the name of the uart which you want to init
*/
int uart_init(char* uart_name);

/*
*Sets channel number to tagret on the servo controller connected to UART number
*@param uartNumber = the uart which the servo controller is connected on (must be open)
*@param channelNumber = the channel on the servo controller that the servo is on
*@param target = pulse width in quarter microseconds
*/
bool setTargetCP(int uartNumber, unsigned char channelNumber, unsigned short target );

//Write All Outputs to Servo Controller
int WriteServos(void);


/*
*Global variables tracking the current servo outputs and the new servo outputs
*/
volatile int CurrPitch;
int TargetPitch;
volatile int CurrThrottle;
int TargetThrottle;

//The Uart number connected to the Servo Controller
int ServoControllerUart;

//The Servo Controller channels with motors on them
int MotorChannels[NUM_MOTORS] = {0};
//The servo controller channels with servos on them
int ServoChannels[NUM_SERVOS] = {1};

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
volatile int ElapsedTicks = 0;
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
	
	uart_init("/dev/ttyS5");
	

	CurrThrottle = MIN_THROTTLE;
	CurrPitch = NEUTRAL_PITCH; 

	WriteServos();


	//make sure callback is not already active
	hrt_cancel(&_call);
	//Connect Timer Handler and begin executing every TICK_INTERVAL
	hrt_call_every(&_call,1,TICK_INTERVAL,(hrt_callout)&TimerHandler, NULL);


	//write initial state to servo controller, this will be uncommented once servo controller is connected to pixhawk
	//int uart = uart_init("/dev/ttyS3");
	//set_uart_baudrate(uart, 57600);
	//setTargetCP(uart, 0, 4000)


	//begin takeoff
	TargetThrottle = MAX_THROTTLE;
	TargetPitch = FULL_PITCH_UP;
	ElapsedTicks = 0;
	ResumeTick = 1000;

	while(ElapsedTicks < ResumeTick)
	{
		/*IMPORTANT:
		*Program hangs here unless we do something in the loop
		*Need to figure out why (maybe compiler optimization?)
		*For now just print to give CPU something to do
		**/
		printf("%d\n",CurrThrottle);
	}

	//Maintain pitch, not trying to loop out
	TargetPitch = NEUTRAL_PITCH;
	ElapsedTicks = 0;
	ResumeTick = 1000;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//shift to cruise throttle and bring nose level
	TargetThrottle = MID_THROTTLE;
	TargetPitch = FULL_PITCH_DOWN;
	ElapsedTicks = 0;
	ResumeTick = 100;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//cruise
	TargetPitch = NEUTRAL_PITCH;
	ElapsedTicks = 0;
	ResumeTick = 1000;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//begin descent
	TargetPitch = FULL_PITCH_DOWN;
	TargetThrottle = MIN_THROTTLE + (MAX_THROTTLE - MIN_THROTTLE) / 5;
	ElapsedTicks = 0;
	ResumeTick = 200;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//reached angle of attack
	TargetPitch = NEUTRAL_PITCH;
	ElapsedTicks = 0;
	ResumeTick = 200;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//flare point reached
	TargetPitch = FULL_PITCH_UP;
	ElapsedTicks = 0;
	ResumeTick = 200;
	while(ElapsedTicks < ResumeTick)
	{
		printf("%d\n",CurrThrottle);
	}

	//land
	TargetPitch = NEUTRAL_PITCH;
	TargetThrottle = MIN_THROTTLE;
	ElapsedTicks = 0;
	ResumeTick = 200;
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
			break;
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
	ElapsedTicks++;
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
	}
	//write new outputs to servos
	WriteServos();
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
	}
	//write new outputs to servos
	WriteServos();
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
		//do nothing
	}
	//write new outputs to servos
	WriteServos();
	return 0;

}


int ThrottleStop(int Sig)
{
	CurrThrottle = 1000;
	return 0;
	if(Sig == TICK)
	{
	}
	//write new outputs to servos
	WriteServos();
}

int PitchUp(int Sig)
{
	if(CurrPitch > TargetPitch)
	{
		PitchState = PITCH_DOWN;
	}
	else if(CurrPitch == TargetPitch)
	{
		PitchState = PITCH_HOLD;
	}
	else if (Sig == TICK)
	{
		CurrPitch += 1;
	}
	//write new outputs to servos
	WriteServos();
	return 0;
}

int PitchDown(int Sig)
{
	if(CurrPitch < TargetPitch)
	{
		PitchState = PITCH_UP;
	}
	else if(CurrPitch == TargetPitch)
	{
		PitchState = PITCH_HOLD;
	}
	else if (Sig == TICK)
	{
		CurrPitch -= 1;
	}
	//write new outputs to servos
	WriteServos();
	return 0;
}

int PitchHold(int Sig)
{
	if(CurrPitch > TargetPitch)
	{
		PitchState = PITCH_DOWN;
	}
	else if(CurrPitch < TargetPitch)
	{
		PitchState = PITCH_UP;
	}
	else if (Sig == TICK)
	{
		//do nothing
	}
	//write new outputs to servos
	WriteServos();
	return 0;
}



int uart_init(char * uart_name)
{
	ServoControllerUart = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (ServoControllerUart < 0) {
        errx(1, "failed to open port: %s", uart_name);
        return 0;
    }

    // setup uart
    warnx("setting up uart");
    struct termios uart_config;
    int ret = tcgetattr(ServoControllerUart, &uart_config);
    if (ret < 0) errx(1, "failed to get attr");
    uart_config.c_oflag &= ~ONLCR; // no CR for every LF
    ret = cfsetispeed(&uart_config, B57600);
    if (ret < 0) errx(1, "failed to set input speed");
    ret = cfsetospeed(&uart_config, B57600);
    if (ret < 0) errx(1, "failed to set output speed");
    ret = tcsetattr(ServoControllerUart, TCSANOW, &uart_config);
    if (ret < 0) errx(1, "failed to set attr");

    // clear old data
    tcflush(ServoControllerUart, TCIOFLUSH);
    return 0;
}



bool setTargetCP(int uartNumber, unsigned char channelNumber, unsigned short target )
{
	unsigned char command[4] = { 0x84, channelNumber, target & 0x7F, (target >> 7) & 0x7F };
	write(uartNumber, command, sizeof(command));

	return true;
}




int WriteServos(void)
{
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		setTargetCP(ServoControllerUart, MotorChannels[i], CurrThrottle * 4);
	}

	for(int i = 0; i < NUM_SERVOS; i++)
	{
		setTargetCP(ServoControllerUart, ServoChannels[i], CurrPitch * 4);
	}
	return 0;
}