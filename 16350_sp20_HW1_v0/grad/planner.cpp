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

// class OpenSet{

// };

class State{

private:
	int x_grid, y_grid;
	double g_val;
	double h_val;
	double f_val;
	bool closed;
	// int heap_index;
	bool expanded;

public:

	State(): g_val(100000.0), closed(0), expanded(0){
		// bool optPath = 0;
	}

	// ~State(){}

	int getX() const {return x_grid;} 
	int getY() const {return y_grid;} 
	double getH() const {return h_val;} 
	double getG() const {return g_val;} 
	double getF() const {return f_val;} 
	bool getExpanded() const {return expanded;} 
	bool getClosed() const {return closed;} 


	void setX(int x_grid_) { x_grid = x_grid_; return;}
	void setY(int y_grid_) { y_grid = y_grid_; return;}

	void setG(double g_val_) { 
		g_val = g_val_;
		// f_val = g_val + h_val;
		return;
	}

	void setF(void){f_val = g_val + h_val;}

	void setH(int goalposeX, int goalposeY){

		h_val = (double)sqrt(((x_grid - goalposeX)*(x_grid-goalposeX) + (y_grid-goalposeY)*(y_grid-goalposeY)));
		return;
	}

	// returns true if successfully added to closed
	void addToClosed(){ closed = 1; return; }

	void expand(){ expanded = 1; return; }

	// void setHeapInd(int heap_index_){heap_index = heap_index_; return;}

};

struct CompareF{
    bool operator()(State const & s1, State const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return s1.getF() < s2.getF();
    }
};


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
    mexPrintf("Started program");
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0}; 
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    // int goalposeX = (int) target_traj[target_steps-1];
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision

    // ********** Old Planner ********
    // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    // double disttotarget;
    // for(int dir = 0; dir < NUMOFDIRS; dir++)
    // {
    //     int newx = robotposeX + dX[dir];
    //     int newy = robotposeY + dY[dir];

    //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    //     {
    //         if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
    //         {
    //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < olddisttotarget)
    //             {
    //                 olddisttotarget = disttotarget;
    //                 bestX = dX[dir];
    //                 bestY = dY[dir];
    //             }
    //         }
    //     }
    // }


    // ********** New Planner *********
    // declare variables
    int t_ct = curr_time;
    int back_ct=0;
    // State expanded_node;
    // State = state;
	// grid_map = new State[x_size][y_size];

 //    vector<vector<State> > grid_map(y_size);
	// for ( int i1 = 0 ; i1 < y_size ; i1++ )
	//    grid_map[i1].resize(x_size);

    State state_init;
	vector<vector<State> > grid_map(y_size, vector<State>(x_size, state_init));


	mexPrintf("state declared \n");

	// initialize start state
	grid_map[robotposeX-1][robotposeY-1].setG(0.0);

	// initialize map
	for (int i =0; i<x_size; i++){
		for (int j=0; j<y_size; j++){

			grid_map[i][j].setX(i+1);
			grid_map[i][j].setY(j+1);

			grid_map[i][j].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
			grid_map[i][j].setF();
		}
	}


	// initialize open list
	priority_queue <State, vector<State>, CompareF> open_set;
	open_set.push( grid_map[robotposeX-1][robotposeY-1] );

	mexPrintf("starting while loop");

	// start while loop for A* expansion
	while( !grid_map[ (int) target_traj[t_ct+1] -1 ][ (int) target_traj[t_ct + target_steps+1] -1 ].getExpanded() && !open_set.empty() && t_ct <100){

		int x_temp = open_set.top().getX();
		int y_temp = open_set.top().getY();
		double g_temp = open_set.top().getG();
		grid_map[x_temp-1][y_temp-1].expand();
		grid_map[x_temp-1][y_temp-1].addToClosed();

		// remove smallest S from open
		open_set.pop();

		// span through successors at next time step
		t_ct++;

		for(int dir = 0; dir < NUMOFDIRS; dir++){

	        int newx = x_temp + dX[dir];
	        int newy = y_temp + dY[dir];

	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
	        {
	            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && (!grid_map[newx-1][newy-1].getClosed()) )  //if free
	            {

	            	if( grid_map[newx-1][newy-1].getG() > g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

						grid_map[newx-1][newy-1].setG(g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
						grid_map[newx-1][newy-1].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
						grid_map[newx-1][newy-1].setF();
						open_set.push(grid_map[newx-1][newy-1]);
	            	}

	            }
	        }
	    }

	    mexPrintf("Finished all expansions");

	    // if (grid_map[ (int) target_traj[t_ct] ][ (int) target_traj[t_ct + target_steps] ].getExpanded()){int t_end = t_ct;}

	    if (t_ct >= 100){cout << "Unable to find a solution"<<endl;}

	    if ( grid_map[ (int) target_traj[t_ct+1] - 1 ][ (int) target_traj[t_ct + target_steps+1] - 1 ].getExpanded() ){cout << "Expanded the target"<<endl;}

	    // delete grid_map;
	}

	// start backtracking
	// State final_st = grid_map[ (int) target_traj[t_ct+1] ][ (int) target_traj[t_ct + target_steps + 1] ];
	list <State> optPath;
	optPath.push_front(grid_map[ (int) target_traj[t_ct+1] - 1 ][ (int) target_traj[t_ct + target_steps + 1] - 1]);

	while( optPath.front().getX() != grid_map[robotposeX-1][robotposeY-1].getX() && optPath.front().getY() != grid_map[robotposeX-1][robotposeY-1].getY() && back_ct < 100 ){

		for(int dir = 0; dir < NUMOFDIRS; dir++){

			double min_G;
	        int newx = optPath.front().getX() + dX[dir];
	        int newy = optPath.front().getY() + dY[dir];

	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){

	        	if(min_G > grid_map[newx-1][newy-1].getG() + (int)map[GETMAPINDEX(optPath.front().getX() , optPath.front().getY(), x_size,y_size)] ){
		        	
		        	optPath.push_front(grid_map[newx-1][newy-1]);
	        	}
	        }
	    }

	    back_ct++;
	}

	optPath.pop_front();
	robotposeX = optPath.front().getX();
	robotposeY = optPath.front().getY();
    // robotposeX = robotposeX + bestX;
    // robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
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