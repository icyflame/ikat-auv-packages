/* demo_dynamic.c -- demonstrate dynamic loading and
   use of the MotionTracker shared object library */

// Include standard c libraries
#include <stdlib.h>

// Need dlfcn.h for the routines to dynamically load libraries
#include <dlfcn.h>

// For nodelay key character processing
//#include <vga.h>

// Needed for printf function
#include <stdio.h>

// Needed for sleep function
#include <unistd.h>

// Needed for MotionTracker functionality
#include <mt9_sensor/MotionTracker.h>

// Return values for MT_GetOrientationData function
#define MT_NEWDATA			1
#define MT_NODATA			2
#define MT_NOSENSORID		3
#define MT_INCOMPLETE		4
#define MT_CHECKSUMERROR	5
#define MT_NOPORT			6
#define MT_NOCALIBVALUES	7
#define MT_POWERLOSS		8

// Output possibilites for MotionTracker library
#define MT_LOGQUATERNION	0
#define MT_LOGEULER			1
#define MT_LOGROTMATRIX		2

#define RESET_HEADING		0
#define RESET_GLOBAL		1
#define RESET_OBJECT		2
#define RESET_ALIGN			3

// Global pointer to MotionTracker library
void *module;

MotionTracker* pMT;
destroy_t* destroy_mtobject;

int g_nPort = -1;

short SetupFilter()
{
	// Set MT library options
	short bLogCalibratedData = 0;

	// Set MT library variables
	short nPortNumber = 1;
	float fGain = 1.0f;
	short nCorInterval = 1;
	float fRho = 1.0f;
	short nMode = MT_LOGEULER;

	////////////////////////////////////////////////////////
	// Sample frequency and .XMU file location not needed with MT9-B
	// Sample frequency and .XMU file location needed with MT9-A
	short nSampleFrequency = 100;
	char cXmuLocation[26] = "00002087_08052003_003.xmu";
	////////////////////////////////////////////////////////

	/* Dynamically load MT library */
	printf("Dynamic load of the MotionTracker library...");
	module = dlopen("../lib/libmtobject.so",RTLD_NOW);
	if (!module) {
		printf("Couldn't open MotionTracker library: %s\n", dlerror());
		return 0;
	}
	printf("done\n\n");

	// Load the symbols
	create_t* create_mtobject = (create_t*) dlsym(module,"create");
	destroy_mtobject = (destroy_t*) dlsym(module,"destroy");

	if (!create_mtobject || !destroy_mtobject) {
		printf("cannot load symbols\n");
		return 0;
	}
	// create an instance of the class
	pMT = create_mtobject();

	printf("Setting filter parameters...");

	if (g_nPort != -1)
		nPortNumber = g_nPort;	// Port number was passed as argument
		
	pMT->MT_SetCOMPort(nPortNumber);
	
	// DeviceName can also be used
	// For instance when using USB device (/dev/ttyUSB*)
	// As long as it starts with /dev/tty.
	//pMT->MT_SetCOMPort_DeviceName("/dev/ttyS0");
	
	// Used to retreive the DeviceID of attached sensor. 
	// Set COM port before using this function!
	pMT->MT_SetCalibratedOutput(bLogCalibratedData);

	pMT->MT_SetFilterSettings(fGain,nCorInterval,fRho);
	
	pMT->MT_SetTimeout(3);

	////////////////////////////////////////////////////////
	// Sample frequency and .XMU file location not needed with MT9-B
	// Sample frequency and .XMU file location needed with MT9-A
	pMT->MT_SetSampleFrequency(nSampleFrequency);
	pMT->MT_SetxmuLocation(cXmuLocation);
	////////////////////////////////////////////////////////
	
	pMT->MT_SetOutputMode(nMode);
	
	printf("done\n\n");
	
	return 1;
}

short GetData()
{
	float fOrientationData[10] = {0};
	short nNew = 0;
	short nRetval = 0;

	pMT->MT_GetOrientationData(&nNew,fOrientationData,0);

	nRetval = nNew;

	switch(nNew) {
	case MT_NEWDATA:
		printf("%f %f %f\n", fOrientationData[0],fOrientationData[1],fOrientationData[2]);
 		break;
	case MT_NODATA:
		printf("No Data On COM Port\n\n");
		break;
	case MT_NOSENSORID:
		printf("No Sensor ID Received From Sensor\n\n");
		break;
	case MT_INCOMPLETE:
		printf("Incomplete Data Received (Connection Lost)\n\n");
		break;
	case MT_CHECKSUMERROR:
		printf("Checksum Error\n\n");
		break;
	case MT_NOPORT:
		printf("COM port Could Not Be opened\n\n");
		break;
	case MT_NOCALIBVALUES:
		printf("XMU File With Calibration Data Could Not Be Read or \nMTS Data With Calibration Data Not Set\n\n");
		break;
	case MT_POWERLOSS:
		printf("Power Supply To The Sensor Was Probably Interupted\n\n");
		break;
	}
	
	return nRetval;
}

int main(int argc, char **argv)
{
	short bKeepRunning = MT_NEWDATA;
	int nChar = 0;
        int c;

        // Check input argument for COM port number of attached MT9
	while (1) {
		c = getopt(argc, argv, "p:");

		if (c == -1) {
			/* End of option list */
			break;
		}

		switch (c) {
		case 'p':
			/* port */
			if (optarg) {
				g_nPort = atoi(optarg);
			}

			break;
		default:
			fprintf(stderr, "Usage: %s [-p port]\n\n", argv[0]);
			exit(1);
		}
	}
	
    // Load the MT library and set filter parameters
	if (!SetupFilter())
		exit(1);

	printf("Start processing by the MotionTracker library...");
	pMT->MT_StartProcess();
	printf("done\n\n");

	// try to get data, stop on error or "q" pressed
	while(bKeepRunning == 0 || bKeepRunning == MT_NEWDATA || bKeepRunning == MT_POWERLOSS)
	{
		bKeepRunning = GetData();	// Poll MoTionTracker library for new data

		//usleep(1);

		nChar; //= vga_getkey();		// Check for key pressed by user
		switch(nChar) {
		case 103:			// Key "g" is pressed
			pMT->MT_ResetOrientation(RESET_GLOBAL);	// Global Reset
			break;
		case 104:			// Key "h" is pressed
			pMT->MT_ResetOrientation(RESET_HEADING);	// Heading Reset
			break;
		case 111:			// Key "o" is pressed
			pMT->MT_ResetOrientation(RESET_OBJECT);	// Object Reset
			break;
		case  97:			// Key "a" is pressed
			pMT->MT_ResetOrientation(RESET_ALIGN);	// Align Reset
			break;
		case 113:			// Key "q" is pressed
			printf("\n\nProcessing stopped by user\n\n");
			bKeepRunning = -1;	// Quit polling loop
			break;
		}
	}
	
	printf("Stopping...");
	pMT->MT_StopProcess();
	printf("done\n\n");

	// Wait for a moment to give MotionTracker object time to close internal thread
	// Not essential, but segmentation fault may occur if program ends before object is done
	usleep(1000);	// Sleep 100 msec

	// Destroy the class
	destroy_mtobject(pMT);

	// Close the MotionTracker shared object
	printf("Close MotionTracker shared object...");
	dlclose(module);
	printf("done\n\n");

	return 1;
}
