#include <vector>
#include <iostream>

#ifndef H_TASK_PARAM_H
#define H_TASK_PARAM_H

using namespace std;

#define START 97
#define MARKER 98
#define FOLLOW_MARKER 99
#define GO_DOWN_MARKER 101
#define BUOY 102
#define HIT_BUOY 103
#define GO_BACK_BUOY 104
#define GO_UP_BUOY 105
#define GO_OVER_BUOY 106



#define GO_HORIZONTAL 107
#define GO_DEPTH 108
#define BIN 109
#define VGATE 110
#define SMALL_SLEEP 1

class TASKS_PARAM{
public:
	float INTERVAL_VEHICLE_START;
    float MARKER_ANGLE_THRESHOLD;
    float INTERVAL_MARKER_FOLLOWING;
    float INTERVAL_GO_DOWN_MARKER;
    float BUOY_AREA_THRESHOLD;
    float INTERVAL_HIT_BUOY;
    float INTERVAL_GO_BACK_BUOY;
    float INTERVAL_GO_UP_BUOY;
    float INTERVAL_GO_OVER_BUOY;
    float DEPTH_RED_BUOY;
    float DEPTH_YELLOW_BUOY;
    float DEPTH_GREEN_BUOY;

    std::vector<std::pair <int, int> > task_schedule;
	void readFromFile();
	void printVar();	
};
#endif

