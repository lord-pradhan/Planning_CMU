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


// define useful functions
int GetIndex(int x_abs, int y_abs, int t_abs, int x_r, int y_r, int t_r){

	int x = x_abs - x_r; int y = y_abs - y_r; int t = t_abs - t_r;
	return t*(2*t-1)*(2*t+1)/3 + (y+t)*(2*t+1) + x + t;
}


// void GetXYT(int index, int &x, int &y, int &t, int x_r, int y_r, int t_r){

// 	// while(floor(index/) )
// 	// t = floor(index/)
// 	// return index - floor(index / )
// 	x = 1; y = 1; t = 1;
// 	return;
// }


class Node{

private:
	int x_grid, y_grid, t_rob;
	// int 
	double g_val;
	double h_val;
	// double f_val;
	// bool expanded;

public:

	Node(){
		// g_val = numeric_limits<double>::infinity();
	}

	int getX() const {return x_grid;} 
	int getY() const {return y_grid;} 
	int getT() const {return t_rob;} 
	// int getID() const {return id;} 

	double getH() const {return h_val;} 
	double getG()  const {return g_val;} 
	// double getF() const {return f_val;} 
	// bool getExpanded() const {return expanded;} 

	void setX(int x_grid_) { x_grid = x_grid_; return;}
	void setY(int y_grid_) { y_grid = y_grid_; return;}
	void setT(int t_rob_) { t_rob = t_rob_; return;}
	// void setID(int id_){
	// 	id = id_;
	// }

	void setG(double g_val_) { 
		g_val = g_val_;
	}

	// void setF(){f_val = g_val + 1.0*h_val;}

	void setH(int robotposeX, int robotposeY, int goalposeX, int goalposeY){

		h_val = (double)sqrt(((robotposeX - goalposeX)*(robotposeX - goalposeX)
		 + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
		return;
	}

	// void expand(){ expanded = true; return; }
};


struct CompareF{
    bool operator()(Node const & n1, Node const & n2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return n1.getG() + n1.getH() > n2.getG() + n2.getH();
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
    mexPrintf("Started program \n");
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

    // State state_init;
    // mexPrintf("testing the state %f \n", state_init.getG());

    // Declare 3D grid
	// vector< vector<vector<State> > > grid_map( target_steps - curr_time, 
	// 	vector< vector<State> >(y_size, vector<State>(x_size, state_init) ) );

	// vector< vector<vector<State> > > grid_map(target_steps - curr_time);
	// for(int i =0; i<target_steps ; i++){

	// 	grid_map[i].resize(2*i+1 , vector<State>(2*i+1, state_init) );
	// }

	// vector<vector<State> > grid_map(y_size, vector<State>(x_size, state_init));
	// mexPrintf("3D vector of states declared \n");



    // set up hash table
    unordered_map<int, double> lookUpG;

    //robot start state
    lookUpG[GetIndex(robotposeX, robotposeY, curr_time, robotposeX, robotposeY, curr_time)] = 0.0;
    Node rob_start;
    rob_start.setX(robotposeX); rob_start.setY(robotposeY); rob_start.setT(curr_time);
    rob_start.setG(0.0);
    rob_start.setH( robotposeX, robotposeY, target_traj[curr_time], target_traj[curr_time+target_steps] );

	// initialize start state
	// grid_map[robotposeY-1][ robotposeX - 1 ][0].setG(0.0);
	// mexPrintf("Start initialized \n");

	// mexPrintf("G_init is %f \n", grid_map[robotposeY-1][ robotposeX - 1 ].getG());
	// mexPrintf("G_1_1 is %f \n", grid_map[1-1][ 1 - 1 ].getG());

	// // initialize map
	// for (int i =0; i<y_size; i++){
	// 	for (int j=0; j<x_size; j++){
	// 		for (int k = 0; k<target_steps - curr_time; k++){

	// 			grid_map[i][j][k].setX(j+1);
	// 			grid_map[i][j][k].setY(i+1);
	// 			grid_map[i][j][k].setT(curr_time+k);

	// 			// grid_map[i][j].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
	// 			// grid_map[i][j].setH(targetposeX, targetposeY);
	// 			grid_map[i][j][k].setH(target_traj[curr_time+k], target_traj[curr_time+k+target_steps]);
	// 			grid_map[i][j][k].setF();
	// 		}
	// 	}
	// }

	// mexPrintf("3D map fully initialized \n");


	// initialize open list
	// priority_queue <State, vector<State>, CompareF> open_set;.

	priority_queue <Node, vector<Node>, CompareF> open_set;
	//also add heuristic
	open_set.push( rob_start );
	
	// initialize closed list
	unordered_set <int> closed_set;

	// mexPrintf("starting while loop\n");

	int index_temp = 0; double g_temp =0; 
	int x_temp = robotposeX; int y_temp = robotposeY; int t_temp = curr_time; 
	int x_catch = -1; int y_catch = -1; int t_catch = -1; 

	// start while loop for A* expansion
	while( !open_set.empty() && t_ct <790000 ){

		Node temp = open_set.top();
		// void GetXYT(temp.getID(), int &x_temp, int &y_temp, int &t_temp, 
		// 	int robotposeX, int robotposeY, int curr_time);

		int x_temp = temp.getX();
		int y_temp = temp.getY();
		int t_temp = temp.getT();
		double g_temp = temp.getG();

		// mexPrintf("g_temp value is %f \n", g_temp);				
		if ( x_temp == target_traj[t_temp] && y_temp ==  target_traj[t_temp+target_steps] ){
			
			closed_set.insert(GetIndex(x_temp, y_temp, t_temp, robotposeX, robotposeY, curr_time));
			cout << "Expanded the target"<<endl;
			int t_catch = t_temp;
			int x_catch = x_temp; int y_catch = y_temp;
			int index_catch = index_temp;
			double g_catch = g_temp;
			break;
		}

		else
			closed_set.insert( GetIndex(x_temp, y_temp, t_temp, robotposeX, robotposeY, curr_time) );
			// grid_map[y_temp-1][x_temp-1][t_temp - curr_time].expand();

		// remove smallest S from open
		open_set.pop();

		// span through successors at next time step
		t_ct++;

		for(int dir = 0; dir < NUMOFDIRS; dir++){

	        int newx = x_temp + dX[dir];
	        int newy = y_temp + dY[dir];
	        int newt = t_temp + 1;

	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
	        {	
	        	int index_temp = GetIndex(newx, newy, t_temp+1, robotposeX, robotposeY, curr_time);
	            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] 
	            	< collision_thresh) && (closed_set.find(index_temp)==closed_set.end()) )
	            {

	            	if( lookUpG.find(index_temp)==lookUpG.end() || lookUpG[index_temp] >
	            	 g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

						lookUpG[index_temp] = g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
						// mexPrintf("G (s') is %f \n", grid_map[newy-1][newx-1].getG());
						// grid_map[newy-1][newx-1].setH(target_traj[t_ct+1], target_traj[t_ct+1+target_steps]);
						// grid_map[newy-1][newx-1].setH(targetposeX, targetposeY);
						// grid_map[newy-1][newx-1][t_temp-curr_time+1].setF();						
						Node toPushOpen;
						toPushOpen.setX(newx); toPushOpen.setY(newy); toPushOpen.setT(newt);
						toPushOpen.setG(lookUpG[index_temp]);
						toPushOpen.setH( newx, newy, target_traj[newt], target_traj[newt+target_steps] );
						open_set.push(toPushOpen);
	            	}
	            }
	        }
	    }

	    // mexPrintf("Finished one expansion\n");

	    if (t_ct >= 790000){cout << "Unable to find a solution\n"<<endl;}

	}


	// start backtracking
	stack <Node> optPath; int t_back = t_catch;// int index_back = index_catch;
	Node temp_back;
	temp_back.setX(x_catch); temp_back.setY(y_catch); temp_back.setT(t_catch);
	// optPath.setG(lookUpG[GetIndex(x_catch, y_catch, t_catch, robotposeX, robotposeY, curr_time)]);
	optPath.push( temp_back );
	// grid_map[  target_traj[t_catch+target_steps]- 1 ][ target_traj[t_catch] - 1][t_catch-curr_time]);

	while( optPath.top().getT() > 0 ){

		double min_G = numeric_limits<double>::infinity(); 
		int finX, finY, fin_index, finT;
		Node temp1 = optPath.top();
		int x_back = temp1.getX(); int y_back = temp1.getY(); int t_back = temp1.getT();

		// void GetXYT(int temp1.getID(), int &x_back, int &y_back, int &t_back, 
		// 	int robotposeX, int robotposeY, int curr_time);

		for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

	        int newx = x_back + dX[dir1];
	        int newy = y_back + dY[dir1];
	        int newt = t_back - 1; 
	        int new_index = GetIndex(newx, newy, newt, robotposeX, robotposeY, curr_time);
	        // mexPrintf("newx is %d, \n",newx);

	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && min_G > lookUpG[new_index]){

	        	min_G = lookUpG[new_index];
				// min_G = grid_map[newy-1][newx-1][t_back - curr_time-1].getG();
				finX = newx; finY = newy; fin_index = new_index;
	        }

	        finT = newt;
	    }
	    // grid_map[finY-1][finX-1].setG(min_G);
	    // mexPrintf("Min G is = %f \n",min_G);
	    Node toPushClosed;
	    toPushClosed.setX(finX); toPushClosed.setY(finY); toPushClosed.setT(finT);
	    // toPushClosed.setG()
	    // ( fin_index , min_G );
	    optPath.push(toPushClosed);
	    // t_back --;
	}

	mexPrintf("Back count %d \n", back_ct);
	// optPath.pop();
	// void GetXYT(int optPath.top().getID(), int &newposeX, int &newposeY, int &t_fin, 
	// 		int robotposeX, int robotposeY, int curr_time);	

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