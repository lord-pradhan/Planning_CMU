/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <iostream>
#include <math.h>
#include <mex.h>
#include <array>
#include <queue>
#include <vector>
#include <list>
#include <iterator>
#include <limits>
#include <bits/stdc++.h> 
#include <algorithm> 
#include <chrono> 
using namespace std::chrono; 

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9

int l_binary = -100;
int r_binary = -100;

// class OpenSet{

// };

double euclidDist(int x1, int y1, int x2, int y2){
	return (double)sqrt((x1 - x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

class State{

private:
	int x_grid, y_grid;
	double g_val;
	double h_val;
	bool expanded;

public:

	State(): expanded(false){
		// bool optPath = 0;
		g_val = numeric_limits<double>::infinity();
	}

	int getX() const {return x_grid;} 
	int getY() const {return y_grid;} 
	double getH() const {return h_val;} 
	double getG() const {return g_val;} 
	bool getExpanded() const {return expanded;} 

	void setX(int x_grid_) { x_grid = x_grid_; return;}
	void setY(int y_grid_) { y_grid = y_grid_; return;}
	void setG(double g_val_) { 
		g_val = g_val_;
	}

	void setH(int goalposeX, int goalposeY){
		h_val = (double)sqrt(((x_grid - goalposeX)*(x_grid-goalposeX) + (y_grid-goalposeY)*(y_grid-goalposeY)));
	}

	void expand(){ expanded = true; return; }

};

struct CompareF{
    bool operator()(State const & s1, State const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long eps = 1;
        return eps*s1.getH() + s1.getG() > eps*s2.getH() + s2.getG();
    }
};

stack <State> optPath;

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
	auto start = high_resolution_clock::now();
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);

	int newposeX = robotposeX; int newposeY = robotposeY;
	// mexPrintf("Target steps are %d \n", target_steps);


	if (l_binary == -100){l_binary = curr_time;}
	if (r_binary== -100){r_binary = target_steps;}

	// if robot reaches, wait there
	if

	while( duration.count() < 990 && euclidDist(robotposeX, robotposeY, 
	target_traj[curr_time], target_traj[curr_time+target_steps]) > 2){ //&& l_binary <= r_binary){ // optPath.pop();

		int target_point = ceil(l_binary + (r_binary - l_binary)/2);

		// mid_array = l + (r-l)/2;

		// if (del_t == 0) { break; }

		// else if (del_t > 0){ l = mid_array +1; }

		// else if (del_t < 0){ r = mid_array-1 ;}

	    // mexPrintf("Started program");
	    // 8-connected grid
	    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
	    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0}; 

	    // ********** New Planner *********
	    // declare variables
	    int t_ct = 0;
	    int back_ct=0;

	    while(!optPath.empty()){
	    	optPath.pop();
	    }

	    State state_init;
		vector<vector<State> > grid_map(y_size, vector<State>(x_size, state_init));

		// initialize start state
		grid_map[robotposeY-1][ robotposeX - 1 ].setG(0.0);

		// initialize map
		for (int i =0; i<y_size; i++){
			for (int j=0; j<x_size; j++){

				grid_map[i][j].setX(j+1);
				grid_map[i][j].setY(i+1);
				grid_map[i][j].setH(target_traj[target_point], target_traj[target_point + target_steps]);
				// grid_map[i][j].setH(target_traj[curr_time+1], target_traj[curr_time+1+target_steps]);
			}
		}

		// initialize open list
		priority_queue <State, vector<State>, CompareF> open_set;
		open_set.push( grid_map[robotposeY-1][robotposeX-1] );

		// mexPrintf("starting while loop\n");

		// start while loop for A* expansion
		while( !grid_map[ (int) target_traj[target_point+target_steps] - 1 ][ (int) target_traj[target_point] -1 ].getExpanded() && !open_set.empty() ){

			State temp = open_set.top();
			int x_temp = temp.getX();
			int y_temp = temp.getY();
			double g_temp = temp.getG();
			grid_map[y_temp-1][x_temp-1].expand();

			// remove smallest S from open
			open_set.pop();

			// span through successors at next time step
			t_ct++;

			for(int dir = 0; dir < NUMOFDIRS; dir++){

		        int newx = x_temp + dX[dir];
		        int newy = y_temp + dY[dir];

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
		        {
		            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && (!grid_map[newy-1][newx-1].getExpanded()) )  //if free
		            {

		            	if( grid_map[newy-1][newx-1].getG() > g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

							grid_map[newy-1][newx-1].setG(g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
							open_set.push(grid_map[newy-1][newx-1]);
		            	}
		            }
		        }
		    }
		}

		// start backtracking
		// stack <State> optPath;
		optPath.push(grid_map[ target_traj[target_point+target_steps] - 1 ][ target_traj[target_point] - 1]);

		while( (optPath.top().getX() != grid_map[robotposeY-1][robotposeX-1].getX() || optPath.top().getY() 
			!= grid_map[robotposeY-1][robotposeX-1].getY() ) && back_ct < 790000 ){

			double min_G = numeric_limits<double>::infinity(); 
			int finX, finY;

			for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

		        int newx = optPath.top().getX() + dX[dir1];
		        int newy = optPath.top().getY() + dY[dir1];

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && min_G > grid_map[newy-1][newx-1].getG() ){

					min_G = grid_map[newy-1][newx-1].getG();
					finX = newx; finY = newy;
		        }

		    }
		   
		    optPath.push(grid_map[finY-1][finX-1]);
		    back_ct++;
		}

		int del_t = target_point - curr_time - optPath.size(); //time that robot has to wait there

		// optPath.pop();
		optPath.pop();
		// mexPrintf("Plan executed \n");
		newposeX = optPath.top().getX();
		newposeY = optPath.top().getY();

		// begin binary seach for next target point
		if (l_binary <= r_binary){

			if (del_t==0){
				mexPrintf("del_t = 0, Plan executed \n");
				action_ptr[0] = newposeX;
			    action_ptr[1] = newposeY;
				return;
			}

			else if(del_t > 0)
				r_binary = target_point - 1;

			else 
				l_binary = target_point +1;
	
		}

		// mexPrintf("Robot pose  %d  %d  \n" , robotposeX, robotposeY);
		// mexPrintf("Next pose %d  %d  \n", newposeX, newposeY);
	    stop = high_resolution_clock::now();
	    duration = duration_cast<microseconds>(stop - start);
	}

	// if (l_binary>r_binary){

	// 	// mexPrintf("Converged \n");
	// 	// mexPrintf("Plan executed \n");
	// 	newposeX = optPath.top().getX();
	// 	newposeY = optPath.top().getY();
	// 	optPath.pop();
	// }

	if ( euclidDist(robotposeX, robotposeY, target_traj[curr_time], target_traj[curr_time+target_steps]) <= 10 ){

		// mexPrintf("Converged \n");
		// mexPrintf("Plan executed \n");
		newposeX = optPath.top().getX();
		newposeY = optPath.top().getY();
		optPath.pop();
	}

	// if (robotposeX==target_traj[curr_time])

	// mexPrintf("newpose is : %d %d \n", newposeX, newposeY);
	action_ptr[0] = newposeX;
    action_ptr[1] = newposeY;
	return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}