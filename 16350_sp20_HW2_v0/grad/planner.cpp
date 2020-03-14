/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
// #include "kdtree.h"
#include <time.h> 
#include <random>
#include <stdio.h> 
#include <stdlib.h> 
#include <algorithm>
#include <vector>
#include <stack>          // std::stack
#include <functional>
#include <queue>
#include <limits>
#include <bits/stdc++.h> 

#include "utilheader.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]


/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
  double cellsize = 1.0;
  //take the nearest cell
  *pX = (int)(x/(double)(cellsize));
  if( x < 0) *pX = 0;
  if( *pX >= x_size) *pX = x_size-1;

  *pY = (int)(y/(double)(cellsize));
  if( y < 0) *pY = 0;
  if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}


struct CompareF{
    bool operator()(NodePRM const & n1, NodePRM const & n2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long eps = 1;
        return n1.getG() >  n2.getG();
    }
};



// All planners reside here
static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
  //for now just do straight interpolation between start and goal checking for the validity of samples
  double distance = 0;
  int i,j;
  for (j = 0; j < numofDOFs; j++){
      if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
          distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
  }
  int numofsamples = (int)(distance/(PI/20));
  if(numofsamples < 2){
      printf("the arm is already at the goal\n");
      return;
  }
  *plan = (double**) malloc(numofsamples*sizeof(double*));
  int firstinvalidconf = 1;
  for (i = 0; i < numofsamples; i++){
      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
      for(j = 0; j < numofDOFs; j++){
          (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
      }
      if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
      {
          firstinvalidconf = 1;
          printf("ERROR: Invalid arm configuration!!!\n");
      }
  }    
  *planlength = numofsamples;

  return;
}


static void plannerPRM( double*	map, int x_size, int y_size, double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad, int numofDOFs, double*** plan,  int* planlength)
{
  // parameters
  int N_samples = 200;
  double angleUB = PI, angleLB = -PI;
  double nbd = 1;

  //initialize
  int ct=0, elemCt = 0;
  std::vector <NodePRM> listOfNodes;
  // srand(time(0));

  while( elemCt < N_samples){

    std::vector <double> currSamplePt;

    // sample a point
    for (int i = 0; i<numofDOFs; i++){

      currSamplePt.push_back( randomDouble(angleLB, angleUB) );
    }

    // make an array of doubles for collision checking
    double anglesArr[ numofDOFs ];
    std::copy( currSamplePt.begin(), currSamplePt.end(), anglesArr );

    if ( !IsValidArmConfiguration( anglesArr, numofDOFs, map, x_size, y_size) ){

      NodePRM pushNode;
      pushNode.setElemID(elemCt);
      pushNode.setCoord( currSamplePt );

      std::vector<NodePRM> neighbours;
      
      // find neighbors
      for (auto i_node : listOfNodes){

        if( distanceFn(pushNode, i_node) < nbd){

          neighbours.push_back( i_node );
        }
      }

      // check connected or not
      for (auto i1_node : neighbours){

        if( !same_component( pushNode, i1_node, listOfNodes ) && can_connect( pushNode, i1_node, map, x_size, y_size ) ){

          pushNode.insertAdj( i1_node.getID() );
          i1_node.insertAdj( pushNode.getID() );
        }
      }

      listOfNodes.push_back( pushNode );
      elemCt++;
    }
  }

  // post-processing done, now find nearest neighbours for start and goal (Query phase)
  std::stack<NodePRM> start_stack, end_stack;
  NodePRM startNode, endNode;

  std::vector<double> startCoord(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> endCoord(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

  startNode.setCoord( startCoord );
  startNode.setElemID(elemCt);
  elemCt++;
  listOfNodes.push_back(startNode);

  endNode.setCoord( endCoord );
  endNode.setElemID(elemCt);
  elemCt++;
  listOfNodes.push_back(endNode);

  int minStart=0, minEnd=0;

  for( auto i_node : listOfNodes ){

  	double startDist = distanceFn( i_node, startNode );
  	double endDist = distanceFn( i_node, endNode );

  	if ( startDist < distanceFn( listOfNodes[minStart], startNode ) ){
  		
  		minStart = i_node.getID();
  		start_stack.push( i_node );
  	}

		if ( endDist < distanceFn( listOfNodes[minEnd], endNode ) ){
  		
  		minEnd = i_node.getID();
  		end_stack.push( i_node );
  	}  	
  }


  while(!start_stack.empty()){

  	if( can_connect( startNode, start_stack.top(), map, x_size, y_size ) ){
  		
  		startNode.insertAdj( start_stack.top().getID() );
      start_stack.top().insertAdj( startNode.getID() );
  		break;
  	}
  	else
  		start_stack.pop();

  }

  while(!end_stack.empty()){

  	if( can_connect( endNode, end_stack.top(), map, x_size, y_size ) ){
  		
  		endNode.insertAdj( end_stack.top().getID() );
      end_stack.top().insertAdj( endNode.getID() );
  		break;
  	}
  	else
  		end_stack.pop();

  }

  // Query done, now search graph using Dijkstraa's
  std::priority_queue< NodePRM , std::vector<NodePRM>, CompareF > open_set;
  startNode.setG(0.0);
  open_set.push( startNode );

  while( !endNode.isExpanded() && !open_set.empty()){

  	NodePRM temp = open_set.top();
  	open_set.pop();

  	for(auto succesor : temp.getAdjIDs() ){

  		if( listOfNodes[succesor].getG() > temp.getG() + distanceFn( temp, listOfNodes[succesor] ) ){

  			listOfNodes[succesor].setG( temp.getG() + distanceFn( temp, listOfNodes[succesor] ) );
  			open_set.push(listOfNodes[succesor]);
  		}
  	}
  }

  // start backtracking
  std::stack<NodePRM> optPath;
  optPath.push( endNode );

  while(optPath.top().getID() != startNode.getID() ){

  	double min_G = std::numeric_limits<double>::infinity(); 
  	int finID;
  	NodePRM temp1 = optPath.top();

  	for( auto succesor : temp1.getAdjIDs() ){

  		if(min_G > temp1.getG() + distanceFn( listOfNodes[succesor], temp1 ) ){

  			min_G = temp1.getG() + distanceFn( listOfNodes[succesor], temp1 );
  			finID = listOfNodes[succesor].getID();
  		}
  	}

  	optPath.push( listOfNodes[finID] );
  }

  optPath.pop();
  *planlength = optPath.size();

  *plan = (double**) malloc(optPath.size() *sizeof(double*));
  for(int i=0; i < optPath.size(); i++ ){

  	(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 

  	for(int j=0; j< numofDOFs; j++){

  		(*plan)[i][j] = optPath.top().getNthCoord(j);
  	}

  	optPath.pop();
  }





// *plan = (double**) malloc(numofsamples *sizeof(double*));
//    int firstinvalidconf = 1;
//   for (i = 0; i < numofsamples; i++){
//       (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
//       for(j = 0; j < numofDOFs; j++){
//           (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
//       }
//       if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
//       {
//           firstinvalidconf = 1;
//           printf("ERROR: Invalid arm configuration!!!\n");
//       }
//   }    
//   *planlength = numofsamples;

  return;
}


static void plannerRRT(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
 
	

    return;
}


static void plannerRRT_star(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    

    
    return;
}


static void plannerRRT_connect(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
        
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;

    
    // you can may be call the corresponding planner function here
    if (planner_id == 3)
    {
       plannerPRM(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }

    if (planner_id == 0)
    {
       plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
 
    if (planner_id == 2)
    {
       plannerRRT_star(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }

    if (planner_id == 1)
    {
       plannerRRT_connect(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }

    //dummy planner which only computes interpolated path
 //    if (planner_id == 0){
	//     planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
	// }
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





