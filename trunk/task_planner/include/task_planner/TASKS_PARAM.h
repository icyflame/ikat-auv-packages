using namespace std;

class TASKS_PARAM{
public:
	float INTERVAL_VEHICLE_START;
	float INTERVAL_FOLLOWING_MARKER;
	float INTERVAL_MARKER_LOCKED;
	float INTERVAL_GOING_DOWN;
	float INTERVAL_BUOY_AREA;
	float INTERVAL_BUOY_ANGLE;
	float INTERVAL_BUOY_RETREATING;
	float INTERVAL_RISE_UP;	

	int START;
	int MARKER;
	int GO_DOWN;
	int FOLLOW_MARKER;
	int GO_FORWARD;
	int BUOY;
	int GO_BACK;
	int GO_UP;
	int BIN;

	void readFromFile();
	void printVar();	
};