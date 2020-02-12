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

#define NUMOFDIRS 8

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

	State(): closed(false), expanded(false){
		// bool optPath = 0;
		g_val = numeric_limits<double>::infinity();
	}

	// ~State(){ numeric_limits<double>::infinity()

	// }

	int getX() const {return x_grid;} 
	int getY() const {return y_grid;} 
	double getH() const {return h_val;} 
	double getG()  {return g_val;} 
	double getF() const {return f_val;} 
	bool getExpanded() const {return expanded;} 
	bool getClosed() const {return closed;} 


	void setX(int x_grid_) { x_grid = x_grid_; return;}
	void setY(int y_grid_) { y_grid = y_grid_; return;}

	void setG(double g_val_) { 
		g_val = g_val_;
		// f_val = g_val + h_val;
	}

	void setF(){f_val = g_val + 1.0*h_val;}

	void setH(int goalposeX, int goalposeY){

		h_val = (double)sqrt(((x_grid - goalposeX)*(x_grid-goalposeX) + (y_grid-goalposeY)*(y_grid-goalposeY)));
		return;
	}

	// returns true if successfully added to closed
	void addToClosed(){ closed = true; return; }

	void expand(){ expanded = true; return; }

	// void setHeapInd(int heap_index_){heap_index = heap_index_; return;}

};

struct CompareF{
    bool operator()(State const & s1, State const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return s1.getF() > s2.getF();
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
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1}; 
    
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
    int t_ct = 0;
    int back_ct=0;
    // State expanded_node;
    // State = state;
	// grid_map = new State[x_size][y_size];

 //    vector<vector<State> > grid_map(y_size);
	// for ( int i1 = 0 ; i1 < y_size ; i1++ )
	//    grid_map[i1].resize(x_size);

    State state_init;
    mexPrintf("testing the state %f \n", state_init.getG());
	vector<vector<State> > grid_map(y_size, vector<State>(x_size, state_init));


	mexPrintf("state declared \n");

	// initialize start state
	grid_map[robotposeY-1][ robotposeX - 1 ].setG(0.0);
	mexPrintf("G_init is %f \n", grid_map[robotposeY-1][ robotposeX - 1 ].getG());
	mexPrintf("G_1_1 is %f \n", grid_map[1-1][ 1 - 1 ].getG());

	// initialize map
	for (int i =0; i<y_size; i++){
		for (int j=0; j<x_size; j++){

			grid_map[i][j].setX(j+1);
			grid_map[i][j].setY(i+1);

			// grid_map[i][j].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
			// grid_map[i][j].setH(targetposeX, targetposeY);
			grid_map[i][j].setH(target_traj[curr_time+1], target_traj[curr_time+1+target_steps]);
			grid_map[i][j].setF();
		}
	}


	// initialize open list
	priority_queue <State, vector<State>, CompareF> open_set;
	open_set.push( grid_map[robotposeY-1][robotposeX-1] );

	mexPrintf("starting while loop\n");

	// start while loop for A* expansion
	// while( !grid_map[ (int) target_traj[t_ct+ 1 + target_steps]-1 ][ (int) target_traj[t_ct +1] -1 ].getExpanded() && !open_set.empty() && t_ct <79000){
	while( !grid_map[ (int) target_traj[curr_time+1+target_steps] - 1 ][ (int) target_traj[curr_time+1] -1 ].getExpanded() && !open_set.empty() && t_ct <790000){

		State temp = open_set.top();
		int x_temp = temp.getX();
		int y_temp = temp.getY();
		double g_temp = temp.getG();
		// mexPrintf("g_temp value is %f \n", g_temp);				
		grid_map[y_temp-1][x_temp-1].expand();
		// grid_map[y_temp-1][x_temp-1].addToClosed();

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
						// mexPrintf("G (s') is %f \n", grid_map[newy-1][newx-1].getG());
						// grid_map[newy-1][newx-1].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
						// grid_map[newy-1][newx-1].setH(targetposeX, targetposeY);
						grid_map[newy-1][newx-1].setF();
						open_set.push(grid_map[newy-1][newx-1]);
	            	}
	            }
	        }
	    }

	    // mexPrintf("Finished one expansion\n");

	    // if (grid_map[ (int) target_traj[t_ct] ][ (int) target_traj[t_ct + target_steps] ].getExpanded()){int t_end = t_ct;}

	    if (t_ct >= 790000){cout << "Unable to find a solution\n"<<endl;}

	    // if ( grid_map[ (int) target_traj[t_ct+1+target_steps] - 1  ][ (int) target_traj[t_ct +1] - 1 ].getExpanded() ){cout << "Expanded the target"<<endl; break;}
	    if ( grid_map[ (int) target_traj[curr_time+1+target_steps] - 1  ][ (int) target_traj[curr_time+1] - 1 ].getExpanded() ){cout << "Expanded the target"<<endl; break;}

	}

	mexPrintf("Target g-value = %f \n", grid_map[  target_traj[curr_time+1+target_steps] - 1 ][ target_traj[curr_time+1] - 1].getG());
	// mexPrintf("Target robot-value  = %f \n", grid_map[  240 - 1 ][ 150 - 1].getG());

	// for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

 //        int newx = grid_map[  targetposeY +1-1 - 1 ][ targetposeX +2 - 1].getX() + dX[dir1];
 //        int newy = grid_map[  targetposeY +1-1 - 1 ][ targetposeX +2 - 1].getY() + dY[dir1];
 //        mexPrintf("G around target is %f, \n", grid_map[newy-1][newx-1].getG());}

	// grid_map[newy-1][newx-1].setG(g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);

	// start backtracking
	// State final_st = grid_map[ (int) target_traj[t_ct+1] ][ (int) target_traj[t_ct + target_steps + 1] ];
	stack <State> optPath;
	// optPath.push(grid_map[ (int) target_traj[t_ct+1+target_steps] - 1 ][ (int) target_traj[t_ct + 1] - 1]);
	optPath.push(grid_map[  target_traj[curr_time+1+target_steps] - 1 ][ target_traj[curr_time+1] - 1]);

	while( (optPath.top().getX() != grid_map[robotposeY-1][robotposeX-1].getX() || optPath.top().getY() != grid_map[robotposeY-1][robotposeX-1].getY() ) && back_ct < 790000 ){

		double min_G = numeric_limits<double>::infinity(); 
		int finX, finY;

		for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

	        int newx = optPath.top().getX() + dX[dir1];
	        int newy = optPath.top().getY() + dY[dir1];
	        // mexPrintf("newx is %d, \n",newx);

	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && min_G > grid_map[newy-1][newx-1].getG() ){

				min_G = grid_map[newy-1][newx-1].getG();
				finX = newx; finY = newy;
	        }

	        // vector<double> v;

	    }
	    // grid_map[finY-1][finX-1].setG(min_G);
	    // mexPrintf("Min G is = %f \n",min_G);
	    optPath.push(grid_map[finY-1][finX-1]);

	    back_ct++;
	}

	mexPrintf("Back count %d \n", back_ct);

	optPath.pop();
	int newposeX = optPath.top().getX();
	int newposeY = optPath.top().getY();
	mexPrintf("Robot pose  %d  %d  \n" , robotposeX, robotposeY);
	mexPrintf("Next pose %d  %d  \n", newposeX, newposeY);
    // robotposeX = robotposeX + bestX;
    // robotposeY = robotposeY + bestY;
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