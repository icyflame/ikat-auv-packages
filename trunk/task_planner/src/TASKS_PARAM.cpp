#include <task_planner/TASKS_PARAM.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std;

void TASKS_PARAM::readFromFile(){
	float value;
	string buff;
	char filename[] = "../TASKS_PARAM.txt";
	ifstream stream(filename);
	if(!stream){
		cout<<"Error reading input file ! \n";
	}
	else{
		int i=0;
		while(getline(stream, buff)){
			if(buff[0] != '#' && buff.size() > 0){
				value = atof(buff.c_str());
				switch(++i){
					case 1:
					INTERVAL_VEHICLE_START = value;break;
					case 2:
					INTERVAL_FOLLOWING_MARKER = value;break;
					case 3:
					INTERVAL_MARKER_LOCKED = value;break;
					case 4:
					INTERVAL_GOING_DOWN = value;break;
					case 5:
					INTERVAL_BUOY_AREA = value;break;
					case 6:
					INTERVAL_BUOY_ANGLE = value;break;
					case 7:
					INTERVAL_BUOY_RETREATING = value;break;
					case 8:
					INTERVAL_RISE_UP = value;break;
					case 9:
					START = value;break;
					case 10:
					MARKER = value;break;
					case 11:
					GO_DOWN = value;break;
					case 12:
					FOLLOW_MARKER = value;break;
					case 13:
					GO_FORWARD = value;break;
					case 14:
					BUOY = value;break;
					case 15:
					GO_BACK = value;break;
					case 16:
					GO_UP = value;break;
					case 17:
					BIN = value;break;
				}
			}
		}

	}
	cout<<"Input values read.\n";
	stream.close();
}

void TASKS_PARAM::printVar(){
	cout<<"INTERVAL_VEHICLE_START - "<<INTERVAL_VEHICLE_START<<endl;
	cout<<"INTERVAL_FOLLOWING_MARKER - "<<INTERVAL_FOLLOWING_MARKER<<endl;
	cout<<"INTERVAL_MARKER_LOCKED - "<<INTERVAL_MARKER_LOCKED<<endl;
	cout<<"INTERVAL_GOING_DOWN - "<<INTERVAL_GOING_DOWN<<endl;
	cout<<"INTERVAL_BUOY_AREA - "<<INTERVAL_BUOY_AREA<<endl;
	cout<<"INTERVAL_BUOY_ANGLE - "<<INTERVAL_BUOY_ANGLE<<endl;
	cout<<"INTERVAL_BUOY_RETREATING - "<<INTERVAL_BUOY_RETREATING<<endl;
	cout<<"INTERVAL_RISE_UP - "<<INTERVAL_RISE_UP<<endl;
	cout<<endl;
	cout<<"START - "<<START<<endl;
	cout<<"MARKER - "<<MARKER<<endl;
	cout<<"GO_DOWN - "<<GO_DOWN<<endl;
	cout<<"FOLLOW_MARKER - "<<FOLLOW_MARKER<<endl;
	cout<<"GO_FORWARD - "<<GO_FORWARD<<endl;
	cout<<"BUOY - "<<BUOY<<endl;
	cout<<"GO_BACK - "<<GO_BACK<<endl;
	cout<<"GO_UP - "<<GO_UP<<endl;
	cout<<"BIN - "<<BIN<<endl;
}