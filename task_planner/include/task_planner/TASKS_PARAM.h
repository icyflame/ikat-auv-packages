#include <vector>
#include <iostream>

#ifndef H_TASK_PARAM_H
#define H_TASK_PARAM_H

using namespace std;

#define START 97
#define MARKER 98
#define GO_UP 99
#define FOLLOW_MARKER 101
#define GO_FORWARD 102
#define BUOY 103
#define GO_BACK 104
#define GO_DOWN 105
#define BIN 106
#define GO_DEEP 107

#define VGATE 107
#define SMALL_SLEEP 1

class TASKS_PARAM{
public:
	float INTERVAL_VEHICLE_START;
    float MARKER_ANGLE_THRESHOLD;
    float INTERVAL_MARKER_FOLLOWING;
	float INTERVAL_GOING_DOWN;
    float BUOY_AREA_THRESHOLD;
    float INTERVAL_BUOY_HIT;
	float INTERVAL_BUOY_RETREATING;
	float INTERVAL_RISE_UP;	
    float DEPTH_RED_BUOY;
    float DEPTH_YELLOW_BUOY;
    float DEPTH_GREEN_BUOY;

    //char TASK_SEQUENCE[50];

//        char _START;
//        char _MARKER;
//        char _GO_DOWN;
//        char _FOLLOW_MARKER;
//        char _GO_FORWARD;
//        char _BUOY;
//        char _GO_BACK;
//        char _GO_UP;
//        char _BIN;

//    std::map<char, char> _map;
    std::vector<std::pair <int, int> > schedule;
//    bool sortpair(const pair<int, int> &, const pair<int, int> &);
	void readFromFile();
	void printVar();	
};
#endif

