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
        int a=0;
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
                                        INTERVAL_GO_DOWN_MARKER = value;break;
                    case 5:
                                        BUOY_AREA_THRESHOLD = value;break;
                    case 6:
                                        INTERVAL_HIT_BUOY = value;break;
                    case 7:
                                        INTERVAL_GO_BACK_BUOY = value;break;
                    case 8:
                                        INTERVAL_GO_UP_BUOY = value;break;
                    case 9:
                                        INTERVAL_GO_OVER_BUOY = value;break;
                    case 10:
                                        DEPTH_RED_BUOY = value;break;
                    case 11:
                                        DEPTH_YELLOW_BUOY = value;break;
                    case 12:
                                        DEPTH_GREEN_BUOY = value;break;

                    case 13:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, START));
                                        break;
                    case 14:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, MARKER));
                                        break;
                    case 15:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, FOLLOW_MARKER));
                                        break;
                    case 16:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, GO_DOWN_MARKER));
                                        break;
                    case 17:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, BUOY));
                                        break;
                    case 18:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, HIT_BUOY));
                                        break;
                    case 19:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, GO_BACK_BUOY));
                                        break;

                    case 20:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, GO_UP_BUOY));
                                        break;

                    case 21:
                                        if(value>0)
                                            task_schedule.push_back(make_pair(value, GO_OVER_BUOY));
                                        break;
				}
            }
		}
        sort(task_schedule.begin(), task_schedule.end(), sortpair);
        cout<<"Input values read.\n";
	}

	stream.close();
}

void TASKS_PARAM::printVar(){
	cout<<"INTERVAL_VEHICLE_START - "<<INTERVAL_VEHICLE_START<<endl;
    cout<<"MARKER_ANGLE_THRESHOLD - "<<MARKER_ANGLE_THRESHOLD<<endl;
    cout<<"INTERVAL_MARKER_FOLLOWING - "<<INTERVAL_MARKER_FOLLOWING<<endl;
    cout<<"INTERVAL_GOING_DOWN - "<<INTERVAL_GO_DOWN_MARKER<<endl;
    cout<<"BUOY_AREA_THRESHOLD - "<<BUOY_AREA_THRESHOLD<<endl;
    cout<<"INTERVAL_BUOY_HIT- "<<INTERVAL_HIT_BUOY<<endl;
    cout<<"INTERVAL_BUOY_RETREATING - "<<INTERVAL_GO_BACK_BUOY<<endl;
    cout<<"INTERVAL_RISE_UP - "<<INTERVAL_GO_UP_BUOY<<endl;
    cout<<"INTERVAL_RISE_UP - "<<INTERVAL_GO_OVER_BUOY<<endl;
    cout<<"DEPTH_RED_BUOY - "<<DEPTH_RED_BUOY<<endl;
    cout<<"DEPTH_YELLOW_BUOY - "<<DEPTH_YELLOW_BUOY<<endl;
    cout<<"DEPTH_GREEN_BUOY - "<<DEPTH_GREEN_BUOY<<endl;
	cout<<endl;
}
