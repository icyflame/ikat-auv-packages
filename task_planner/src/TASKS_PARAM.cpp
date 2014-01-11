#include <task_planner/TASKS_PARAM.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

using namespace std;

bool sortpair(const pair<int, int> &a,const pair<int, int> &b){
    return (a.first < b.first);
}

void TASKS_PARAM::readFromFile(){
	float value;
	string buff;
        char filename[] = "/home/ikat/ros_workspace/ikat-auv-packages/task_planner/TASKS_PARAM.txt";
	ifstream stream(filename);
	if(!stream){
		cout<<"Error reading input file ! \n";
	}
	else{
		int i=0;
        while(getline(stream, buff))
        {
            if(buff[0] != '#' && buff.size() > 0)
            {
				value = atof(buff.c_str());
				switch(++i){
					case 1:
                                        INTERVAL_VEHICLE_START = value;break;
					case 2:
                                        MARKER_ANGLE_THRESHOLD = value;break;
					case 3:
                                        INTERVAL_MARKER_FOLLOWING = value;break;
					case 4:
                                        INTERVAL_GOING_DOWN = value;break;
					case 5:
                                        BUOY_AREA_THRESHOLD = value;break;
					case 6:
                                        INTERVAL_BUOY_HIT = value;break;
					case 7:
                                        INTERVAL_BUOY_RETREATING = value;break;
					case 8:
                                        INTERVAL_RISE_UP = value;break;
                    case 9:
                                        DEPTH_RED_BUOY = value;break;
                    case 10:
                                        DEPTH_YELLOW_BUOY = value;break;
                    case 11:
                                        DEPTH_GREEN_BUOY = value;break;

                    case 12:
//                                        _START = value;
//                                        _map[_START] = (char)START;
                                        schedule.push_back(make_pair(value, START));
                                        break;
                    case 13:
                                        //_MARKER = value;
//                                        _map[_MARKER] = (char)MARKER;
                                        schedule.push_back(make_pair(value, MARKER));
                                        break;
                    case 14:
//                                        _GO_DOWN = value;
//                                        _map[_GO_DEEP] = (char)GO_DEEP;
                                        schedule.push_back(make_pair(value, GO_UP));
                                        break;
                    case 15:
//                                        _FOLLOW_MARKER = value;
//                                        _map[_FOLLOW_MARKER] = (char)FOLLOW_MARKER;
                                        schedule.push_back(make_pair(value, FOLLOW_MARKER));
                                        break;
                    case 16:
//                                        _GO_FORWARD = value;
//                                        _map[_GO_FORWARD] = (char)GO_FORWARD;
                                        schedule.push_back(make_pair(value, GO_FORWARD));
                                        break;
                    case 17:
//                                        _BUOY = value;
//                                        _map[_BUOY] = (char)BUOY;
                                        schedule.push_back(make_pair(value, BUOY));
                                        break;
                    case 18:
//                                        _GO_BACK = value;
//                                        _map[_GO_BACK] = (char)GO_BACK;
                                        schedule.push_back(make_pair(value, GO_BACK));
                                        break;

                    case 19:
                                        schedule.push_back(make_pair(value, GO_DOWN));
                                        break;

                    case 20:
//                                        _BIN = value;
//                                        _map[_BIN] = (char)BIN;
                                        schedule.push_back(make_pair(value, BIN));
                                        break;
                    case 21:
                                        schedule.push_back(make_pair(value, GO_DEEP));
                                        break;
				}
			}
		}
        sort(schedule.begin(), schedule.end(), sortpair);
        cout<<"Input values read.\n";
	}

	stream.close();
}

void TASKS_PARAM::printVar(){
	cout<<"INTERVAL_VEHICLE_START - "<<INTERVAL_VEHICLE_START<<endl;
    cout<<"MARKER_ANGLE_THRESHOLD - "<<MARKER_ANGLE_THRESHOLD<<endl;
    cout<<"INTERVAL_MARKER_FOLLOWING - "<<INTERVAL_MARKER_FOLLOWING<<endl;
	cout<<"INTERVAL_GOING_DOWN - "<<INTERVAL_GOING_DOWN<<endl;
    cout<<"BUOY_AREA_THRESHOLD - "<<BUOY_AREA_THRESHOLD<<endl;
    cout<<"INTERVAL_BUOY_HIT- "<<INTERVAL_BUOY_HIT<<endl;
	cout<<"INTERVAL_BUOY_RETREATING - "<<INTERVAL_BUOY_RETREATING<<endl;
	cout<<"INTERVAL_RISE_UP - "<<INTERVAL_RISE_UP<<endl;
    cout<<"DEPTH_RED_BUOY -"<<DEPTH_RED_BUOY<<endl;
    cout<<"DEPTH_YELLOW_BUOY -"<<DEPTH_YELLOW_BUOY<<endl;
    cout<<"DEPTH_GREEN_BUOY -"<<DEPTH_GREEN_BUOY<<endl;
	cout<<endl;
    cout<<"START - "<<START<<endl;
	cout<<"MARKER - "<<MARKER<<endl;
    cout<<"GO_UP - "<<GO_UP<<endl;
	cout<<"FOLLOW_MARKER - "<<FOLLOW_MARKER<<endl;
	cout<<"GO_FORWARD - "<<GO_FORWARD<<endl;
	cout<<"BUOY - "<<BUOY<<endl;
	cout<<"GO_BACK - "<<GO_BACK<<endl;
    cout<<"GO_DOWN - "<<GO_UP<<endl;
    cout<<"BIN - "<<BIN<<endl;
    cout<<"GO_DEEP - "<<GO_DEEP<<endl;
}
