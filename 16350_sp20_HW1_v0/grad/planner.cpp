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


#define NUMOFDIRS 9

// define useful functions
unsigned long long GetIndex(int x_abs, int y_abs, int t_abs, int x_r, int y_r, int t_r){

	int x = x_abs - x_r; int y = y_abs - y_r; int t = t_abs - t_r;

	// return t*(2*t-1)*(2*t+1)/3 + (y+t)*(2*t+1) + x + t

	// return ( (double) ((x + y) * (x + y + 1.0)/2.0 + y + t)*((x + y) 
	// 	* (double)(x + y + 1.0)/2.0 + y + t + 1.0) )/2.0 + t;

	return floor( 1000000* (sqrt(2)*x + sqrt(5)*y + sqrt(11)*t) );
}


class Node{

private:
	int x_grid, y_grid, t_rob;

	double g_val;
	double h_val;

public:

	Node(){
		// g_val = numeric_limits<double>::infinity();
	}

	int getX() const {return x_grid;} 
	int getY() const {return y_grid;} 
	int getT() const {return t_rob;} 

	double getH() const {return h_val;} 
	double getG()  const {return g_val;} 

	void setX(int x_grid_) { x_grid = x_grid_; return;}
	void setY(int y_grid_) { y_grid = y_grid_; return;}
	void setT(int t_rob_) { t_rob = t_rob_; return;}

	void setG(double g_val_) { 
		g_val = g_val_;
	}

	void setH(int robotposeX, int robotposeY, int goalposeX, int goalposeY){

		h_val = (double)sqrt(((robotposeX - goalposeX)*(robotposeX - goalposeX)
		 + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
		return;
	}

};


struct CompareF{
    bool operator()(Node const & n1, Node const & n2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long long int eps = 10000000;
        return n1.getG() + eps*n1.getH() > n2.getG() + eps*n2.getH();
    }
};


int function_call = 0;
stack <Node> optPath;

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
    // mexPrintf("Started program \n");


    if (function_call==0){

	    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
	    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0}; 
	    
	    // ********** New Planner *********
	    // declare variables
	    unsigned int t_ct = 0;
	    unsigned int back_ct=0;

	    while(!optPath.empty()){
	    	optPath.pop();
	    }

	    // set up hash table
	    unordered_map<unsigned long long, double> lookUpG;

	    //robot start state
	    lookUpG[GetIndex(robotposeX, robotposeY, curr_time, robotposeX, robotposeY, curr_time)] = 0.0;
	    Node rob_start;
	    rob_start.setX(robotposeX); rob_start.setY(robotposeY); rob_start.setT(curr_time);
	    rob_start.setG(0.0);
	    rob_start.setH( robotposeX, robotposeY, target_traj[curr_time], target_traj[curr_time+target_steps] );


		priority_queue <Node, vector<Node>, CompareF> open_set;

		open_set.push( rob_start );
		
		// initialize closed list
		unordered_set <unsigned long> closed_set;

		unsigned long long index_temp = 0; double g_temp =0; 
		int x_temp = robotposeX; int y_temp = robotposeY; int t_temp = curr_time; 
		int x_catch = -1; int y_catch = -1; int t_catch = -1; 

		// start while loop for A* expansion
		while( !open_set.empty() ){

			Node temp = open_set.top();			

			int x_temp = temp.getX();
			int y_temp = temp.getY();
			int t_temp = temp.getT();
			double g_temp = temp.getG();

			if ( x_temp == target_traj[t_temp] && y_temp ==  target_traj[t_temp+target_steps] ){
				
				closed_set.insert(GetIndex(x_temp, y_temp, t_temp, robotposeX, robotposeY, curr_time));
				cout << "Expanded the target"<<endl;
				t_catch = t_temp;
				x_catch = x_temp; y_catch = y_temp;

				break;
			}

			else{
				closed_set.insert( GetIndex(x_temp, y_temp, t_temp, robotposeX, robotposeY, curr_time) );

			}

			// remove smallest S from open
			open_set.pop();

			// span through successors at next time step
			t_ct++;

			for(int dir = 0; dir < NUMOFDIRS; dir++){

		        int newx = x_temp + dX[dir];
		        int newy = y_temp + dY[dir];
		        int newt = t_temp + 1;
		        unsigned long long index_temp = GetIndex(newx, newy, newt, robotposeX, robotposeY, curr_time);

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && 
		        	((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
		        	((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && 
		        	(closed_set.find(index_temp)==closed_set.end()) &&

		        	(lookUpG.find(index_temp)==lookUpG.end() || 
		        	lookUpG[index_temp] > g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]) ){

					lookUpG[index_temp] = g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];

					Node toPushOpen;
					toPushOpen.setX(newx); toPushOpen.setY(newy); toPushOpen.setT(newt);
					toPushOpen.setG(lookUpG[index_temp]);
					toPushOpen.setH( newx, newy, target_traj[newt], target_traj[newt+target_steps] );
					open_set.push(toPushOpen);
		        }
		    }

		    // if (t_ct >= 790000){cout << "Unable to find a solution\n"<<endl;}
		}


		// start backtracking
		int t_back = t_catch;// int index_back = index_catch;
		Node temp_back;
		temp_back.setX(x_catch); temp_back.setY(y_catch); temp_back.setT(t_catch);
		// mexPrintf("X catch is %d \n Y catch is %d \n T catch is %d \n", x_catch, y_catch, t_catch);

		optPath.push( temp_back );

		while( (optPath.top().getX() != robotposeX || optPath.top().getY() != robotposeY) && 
			optPath.top().getT() > curr_time+1 ) {

			double min_G = 1000000; 
			int finX, finY, finT; unsigned long fin_index;
			Node temp1 = optPath.top();
			int x_back = temp1.getX(); int y_back = temp1.getY(); int t_back = temp1.getT();
			// mexPrintf("state while backtracking is %d %d %d \n", x_back, y_back, t_back);
		
			for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

		        int newx = x_back + dX[dir1];
		        int newy = y_back + dY[dir1];
		        int newt = t_back - 1; 
		        unsigned long long new_index = GetIndex(newx, newy, newt, robotposeX, robotposeY, curr_time);

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && 
		        	min_G >	lookUpG[new_index] && lookUpG[new_index]!= 0.0 &&
		        	!(closed_set.find(new_index)==closed_set.end()) ){

		        	min_G = lookUpG[new_index];
					finX = newx; finY = newy; fin_index = new_index; finT = newt;
		        }
		    }

		    Node toPushClosed;
		    toPushClosed.setX(finX); toPushClosed.setY(finY); toPushClosed.setT(finT);
		    
		    optPath.push(toPushClosed);
		    
		    back_ct++;		    
		} 		
	}

	else{
		optPath.pop();
		// mexPrintf("entered else \n");
	}

	int newposeX = optPath.top().getX();
	int newposeY = optPath.top().getY();
	int newtime = optPath.top().getT();

	// mexPrintf("Robot pose  %d  %d  %d \n" , robotposeX, robotposeY, curr_time);
	// // mexPrintf("G-value at robot pose is %f \n", 
	// // 	lookUpG[GetIndex(robotposeX, robotposeY, curr_time, robotposeX, robotposeY, curr_time) ]);
	// mexPrintf("Next pose %d  %d  %d \n", newposeX, newposeY, newtime);
	// mexPrintf("G-value at final pose is %f \n", 
		// lookUpG[GetIndex(newposeX, newposeY, curr_time, robotposeX, robotposeY, curr_time) ] );

    action_ptr[0] = newposeX;
    action_ptr[1] = newposeY;
       
    function_call++;
    // mexPrintf("One run done \n");
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