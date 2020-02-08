/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

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

int temp = 0;

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           int robotposeX,
            int robotposeY,
            int goalposeX,
            int goalposeY,
            char *p_actionX,
            char *p_actionY
		   )
{

    //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};    
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    printf("call=%d\n", temp);
    temp = temp+1;
    
    //printf("robot: %d %d; ", robotposeX, robotposeY);
    //printf("goal: %d %d;", goalposeX, goalposeY);
    
	//for now greedily move towards the target, 
	//but this is where you can put your planner 
	double mindisttotarget = 1000000;
	for(int dir = 0; dir < NUMOFDIRS; dir++)
	{
    	   int newx = robotposeX + dX[dir];
     	   int newy = robotposeY + dY[dir];
    
    	   if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
               
              if ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] == 0){ //if free
                double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < mindisttotarget){
                  mindisttotarget = disttotarget;
                  *p_actionX = dX[dir];
                  *p_actionY = dY[dir];
                }
              }
    	   }
	}
    //printf("action: %d %d; \n", *p_actionX, *p_actionY);

    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector <x,y> for the robot pose
//3rd is a row vector <x,y> for the target pose
//plhs should contain output parameters (1): 
//1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
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
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]); 
    return;
    
}





