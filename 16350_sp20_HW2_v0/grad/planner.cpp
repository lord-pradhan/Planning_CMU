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
#include <iostream>

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
        // long eps = 1;
        return n1.getG() > n2.getG();
    }
};



// *******All planners reside here*********

// ****DEFAULT********
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




///// Begin planners ////////
// ********PLANNER PRM ******* ///////

static void plannerPRM( double*	map, int x_size, int y_size, double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad, int numofDOFs, double*** plan,  int* planlength)
{
  //no plan by default
  *plan = NULL;
  *planlength = 0;
  // mexPrintf("Planner called \n");
  // mexEvalString("drawnow");

  // parameters
  int N_samples = 10000;
  double angleUB = 2*3.14, angleLB = 0.0;
  double nbd = sqrt(numofDOFs)*10.0*PI/180.0;

  //initialize
  int ct=0, elemCt = 0;
  std::vector <NodePRM> listOfNodes;
  srand(time(nullptr));
  // srand(time(0));

  std::vector<double> startCoord(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> endCoord(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

  if(IsValidArmConfiguration( startCoord.data(), numofDOFs, map, x_size, y_size )==1 
      && IsValidArmConfiguration( endCoord.data(), numofDOFs, map, x_size, y_size )==1 ){

    mexPrintf("Before first while loop \n");
    mexEvalString("drawnow");

    int ct1=0;
    while( elemCt < N_samples ){

      std::vector <double> currSamplePt;

      // sample a point
      for (int i = 0; i<numofDOFs; i++){

        double temp = randomDouble(angleLB, angleUB);
        currSamplePt.push_back( temp );
      }

      if ( IsValidArmConfiguration( currSamplePt.data(), numofDOFs, map, x_size, y_size)==1 ){

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
        if(!neighbours.empty()){

          for (auto i1_node : neighbours){

            if( !same_component( pushNode, i1_node, listOfNodes ) && 
              can_connect( pushNode, i1_node, map, x_size, y_size ) ){

              pushNode.insertAdj( i1_node.getID() );
              listOfNodes[i1_node.getID()].insertAdj( pushNode.getID() );
            }
          }
        }

        listOfNodes.push_back( pushNode );
        elemCt++;
      }

    //   mexPrintf("Didn't find valid arm config \n");
      // mexEvalString("drawnow");
      ct1++;
    }

    mexPrintf("Exited preprocessing - %d \n", ct1);
    mexEvalString("drawnow");


    /////// Query phase /////
    // post-processing done, now find nearest neighbours for start and goal ////////
    std::stack<NodePRM> start_stack, end_stack;
    NodePRM startNode, endNode;

    // mexPrintf("size of start and end vectors - %d and %d \n", startCoord.size(), endCoord.size());
    // mexEvalString("drawnow");

    startNode.setCoord( startCoord );
    startNode.setElemID(elemCt);
    elemCt++;

    endNode.setCoord( endCoord );
    endNode.setElemID(elemCt);
    elemCt++;


    int minStart=0, minEnd=0;


    for( NodePRM i_node : listOfNodes ){

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


    listOfNodes.push_back(startNode);
    listOfNodes.push_back(endNode);

    int ct2 = 0;
    while( !start_stack.empty() ){//&& ct2<1000 ){

      if( can_connect( startNode, start_stack.top(), map, x_size, y_size ) ){
        
        listOfNodes[N_samples].insertAdj( start_stack.top().getID() );
        listOfNodes[start_stack.top().getID()].insertAdj( listOfNodes[N_samples].getID() );
        ct2++;
        break;
      }
      else
        start_stack.pop();

      ct2++;
    }

    int ct3=0;
    while(!end_stack.empty() ){// && ct3<1000){

      if( can_connect( endNode, end_stack.top(), map, x_size, y_size ) ){
        
        listOfNodes[N_samples+1].insertAdj( end_stack.top().getID() );
        listOfNodes[end_stack.top().getID()].insertAdj( listOfNodes[N_samples+1].getID() );
        ct3++;
        break;
      }
      else
        end_stack.pop();

      ct3++;
    }

    mexPrintf("Ended while loops for finding closest points - %d and %d \n", ct2, ct3);
    mexEvalString("drawnow");


    //// ****** Query done, now Dijsktraa's *********//////

    std::priority_queue< NodePRM , std::vector<NodePRM>, CompareF > open_set;
    listOfNodes[N_samples].setG(0.0);
    open_set.push( listOfNodes[N_samples] );

    int ct4=0;
    while( !listOfNodes[N_samples+1].isExpanded() && !open_set.empty() && ct4<100000){

      // NodePRM temp = open_set.top();
      // open_set.pop();
      int tempID = open_set.top().getID();
      listOfNodes[tempID].expand();
      open_set.pop();

      printf("listOfNodes[tempID].getG() is %lf \n tempID is %d \n ", listOfNodes[tempID].getG(), tempID);

      if(listOfNodes[tempID].getAdjIDs().empty()){
        printf("someone doesnt have any connections :( \n");
        break;
      }

      for(auto succesor : listOfNodes[tempID].getAdjIDs() ){

        printf("listOfNodes[succesor].getG() is %lf \n succesor is %d \n", 
          listOfNodes[succesor].getG(), succesor);
        printf("distanceFn( listOfNodes[tempID], listOfNodes[succesor] ) is %lf\n",
         distanceFn( listOfNodes[tempID], listOfNodes[succesor] ));

        if( listOfNodes[succesor].getG() > listOfNodes[tempID].getG() + 
          distanceFn( listOfNodes[tempID], listOfNodes[succesor] ) ){

          listOfNodes[succesor].setG( listOfNodes[tempID].getG() + distanceFn( listOfNodes[tempID], 
                                      listOfNodes[succesor] ) );
          open_set.push(listOfNodes[succesor]);
        }
      }

      if(listOfNodes[N_samples+1].isExpanded()){

        printf("target expanded %d \n", ct4);
        break;

      } else if(open_set.empty()){

        printf("open set is empty\n");
        printf("path can't be found\n");
        return;
      }

      ct4++;
    }

    mexPrintf("end while loop 4 - %d \n", ct4);
    mexEvalString("drawnow");

    printf("listOfNodes.back().getG() is %lf \n listOfNodes.back().getAdjIDs()[0] is %d \n", 
      listOfNodes[N_samples+1].getG(), listOfNodes[N_samples+1].getAdjIDs()[0]);


    ///////// start backtracking ////////////
    std::stack<NodePRM> optPath;
    optPath.push( listOfNodes[N_samples+1] );

    int ct5=0;
    while( optPath.top().getID() != listOfNodes[N_samples].getID() && ct5<100000 ){

      double min_G = std::numeric_limits<double>::infinity(); 
      int finID;
      int tempID = optPath.top().getID();

      for( auto succesor : listOfNodes[tempID].getAdjIDs() ){

        if(min_G > listOfNodes[succesor].getG() + distanceFn(listOfNodes[succesor], listOfNodes[tempID])){

          min_G = listOfNodes[succesor].getG() + distanceFn( listOfNodes[succesor], listOfNodes[tempID] );
          finID = listOfNodes[succesor].getID();
        }
      }

      optPath.push( listOfNodes[finID] );
      ct5++;
    }

    mexPrintf("end backtracking while loop - %d \n Start populating plan", ct5);
    mexEvalString("drawnow");
    // optPath.pop();
    int sizePathFin = optPath.size();
    *planlength = sizePathFin;

    *plan = (double**) malloc(sizePathFin *sizeof(double*));
    for(int i=0; i < sizePathFin; i++ ){

      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 

      for(int j=0; j< numofDOFs; j++){

        (*plan)[i][j] = optPath.top().getNthCoord(j);
      }

      optPath.pop();
    }
  }
  else{

    mexPrintf("either goal or start configs are invalid \n");
    mexEvalString("drawnow");
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






// *************PLANNER RRT ******************** //
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

  //  parameters
  double eps = sqrt(numofDOFs)*5.0*PI/180.0;
  double tol = 2.0*PI/180;
  double goalProb = 0.2;
  bool backwards = 1;

  // initialize
  bool goalRegion = false;
  srand(time(nullptr));
  double angleUB = 2.0*3.14, angleLB = 0.0;
  int goalSampling;

  // store start and end vectors
  std::vector<double> startCoordActual(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> endCoordActual(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

  std::vector<double> startCoord, endCoord;

  if (backwards){

    startCoord = endCoordActual;
    endCoord = startCoordActual;
  }
  else
  {
    startCoord = startCoordActual;
    endCoord = endCoordActual;
  }
 
  if(IsValidArmConfiguration( startCoordActual.data(), numofDOFs, map, x_size, y_size )==1 
    && IsValidArmConfiguration( endCoordActual.data(), numofDOFs, map, x_size, y_size )==1 ){

    // initialize tree with start / end
    // mexPrintf("before initializing trees \n");
    // mexEvalString("drawnow");
    NodeRRT* root = new NodeRRT;
    root->setParent(nullptr);
    root->setCoord( startCoord );

    NodeRRT* tail = new NodeRRT;
    tail->setParent(nullptr);

    int ct1=0;
    // begin while loop
    while( !goalRegion && ct1<10000000 ){

      ct1++;
      // mexPrintf("entered main while loop \n");
      // mexEvalString("drawnow");
      std::vector<double> currSamplePt;

      // srand(time(nullptr));
      double probTemp = randomDouble(0.0, 1.0);

      if (probTemp < goalProb)
          goalSampling =1;
      else
          goalSampling = 0;


      if(goalSampling==0){

        for (int i = 0; i<numofDOFs; i++){

          double temp = randomDouble(angleLB, angleUB);
          currSamplePt.push_back( temp );
          // mexPrintf("random point is %f \n", temp);
          // mexEvalString("drawnow");
        }
        // goalSampling=1;
      }
      else{

        for (int i = 0; i<numofDOFs; i++){

          double temp = randomDouble(endCoord[i] - tol, endCoord[i]+tol);
          currSamplePt.push_back( temp );
          // mexPrintf("goal point is %f \n", temp);
          // mexEvalString("drawnow");
        }
        // goalSampling=0;
      }

      if ( IsValidArmConfiguration( currSamplePt.data(), numofDOFs, map, x_size, y_size )==1 ){

        int marker = extend( root, tail, currSamplePt, eps, map, x_size, y_size,  endCoord , tol);
        // '0' = reached, '1' = advanced, '2' = trapped, '3' = reached goal
        // mexPrintf("extend returns %d \n", marker);
        // mexEvalString("drawnow");

        if(marker==3)
          goalRegion=true;
      }

      // mexPrintf("extend returns %d \n", marker);
      // mexEvalString("drawnow");
    }

    // tree traversal from tail to root

    if(!backwards){ // if forwards expansion

      std::stack< std::vector<double> > finPath;
      if ( tail->getParent()!=nullptr ){

        finPath.push(tail->getCoords());
        NodeRRT* traverse = tail;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push( traverse->getCoords() );
        }
      }

      // mexPrintf("traverse while loop done \n");
      // mexEvalString("drawnow");
      // finPath.pop();
      int sizePath =finPath.size();
      *planlength = sizePath;

      // printf("finPath size is %d \n size of vector is %d \n", sizePath, finPath.top().size());
      // mexEvalString("drawnow");

      *plan = (double**) malloc(sizePath *sizeof(double*));
      int firstinvalidconf = 1;

      for(int i=0; i < sizePath; i++ ){

        if(finPath.empty()){

          // printf("broke \n");
          // mexEvalString("drawnow");
          break;
        }

        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 

        for(int j=0; j< numofDOFs; j++){

          (*plan)[i][j] = finPath.top()[j];
          // printf( "print finPath for i: %d, j: %d = %lf \n", i, j, finPath.top()[j]);
        }

        if( IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)==0 && firstinvalidconf)
        {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }

        finPath.pop();
      }
    }
    else{ // if backwards expansion

      std::queue< std::vector<double> > finPath;
      if ( tail->getParent()!=nullptr ){

        finPath.push(tail->getCoords());
        NodeRRT* traverse = tail;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push( traverse->getCoords() );
        }
      }

      int sizePath =finPath.size();
      *planlength = sizePath;

      *plan = (double**) malloc(sizePath *sizeof(double*));
      int firstinvalidconf = 1;

      for(int i=0; i < sizePath; i++ ){

        if(finPath.empty()){

          // printf("broke \n");
          // mexEvalString("drawnow");
          break;
        }

        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 

        for(int j=0; j< numofDOFs; j++){

          (*plan)[i][j] = finPath.front()[j];
          // printf( "print finPath for i: %d, j: %d = %lf \n", i, j, finPath.top()[j]);
        }

        if( IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)==0 && firstinvalidconf)
        {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }

        finPath.pop();
      }
    }

    // deallocate pointers
    if(root!=nullptr)
      delete root;
    
    if(tail!=nullptr)
      delete tail;
  }

  else{

    mexPrintf("either goal or start configs are invalid \n");
    mexEvalString("drawnow");
  }

  return;
}




// ********PLANNER RRT_STAR ******* ///////
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

// ********PLANNER RRT CONNECT ******* ///////
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

  //parameters
  double eps = sqrt(numofDOFs)*5.0*PI/180.0;

  // initialize
  bool treeConnected = false;
  double angleUB = 2.0*3.14, angleLB = 0.0;

  // store start and end vectors
  std::vector<double> startCoord(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> endCoord(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

  if(IsValidArmConfiguration( startCoord.data(), numofDOFs, map, x_size, y_size )==1 
    && IsValidArmConfiguration( endCoord.data(), numofDOFs, map, x_size, y_size )==1 ){
    
    // initialize both trees
    NodeRRT* tree1 = new NodeRRT;
    tree1->setParent(nullptr);
    tree1->setCoord( startCoord );

    NodeRRT* tree2 = new NodeRRT;
    tree2->setParent(nullptr);    
    tree2->setCoord( endCoord );

    NodeRRT* tail1 = new NodeRRT;
    NodeRRT* tail2 = new NodeRRT;
    // printf("just before declaring new nodes \n");

    while(!treeConnected){

      // printf("entered while loop\n");
      std::vector<double> currSamplePt;

      for (int i = 0; i<numofDOFs; i++){

        double temp = randomDouble(angleLB, angleUB);
        currSamplePt.push_back( temp );
        // mexPrintf("random point is %f \n", temp);
        // mexEvalString("drawnow");
      }

      if ( IsValidArmConfiguration( currSamplePt.data(), numofDOFs, map, x_size, y_size )==1 ){

        // newNode1 = nullptr;
        NodeRRT* newNode1 = new NodeRRT;
        // follow this around to debug
        // printf("valid arm config\n");

        if(extend_connect(tree1, currSamplePt, newNode1, eps, map, x_size, y_size)!=2){

          // newNode2 = nullptr;
          // printf("extend star if condition entered\n");

          if (connect(tree2, newNode1, tail2, eps, map, x_size, y_size)==0 ){

            treeConnected=true;
            // tail1->setCoord(newNode1->getCoords());
            // tail1->setParent( newNode1->getParent() );
            tail1 = newNode1;
            // tail2 = newNode2;         
            // printf("tree connected \n");
            break;
          }
        }
        // printf("just before swap trees\n");
        swapTrees(tree1, tree2);
        // swapTrees( tree1, tree2 );
      }
    }

    // check which tree is which
    // printf("start backtracking\n");
    bool forward;
    std::list< std::vector<double> > finPath;
    // printf("startCoord [0] is %lf, [1] is %lf \n", startCoord[0], startCoord[1]);
    // printf("tree1 get coords [0] is %lf, [1] is %lf \n", tree1->getCoords()[0], tree1->getCoords()[1]);
    // printf("tree2 get coords [0] is %lf, [1] is %lf \n", tree2->getCoords()[0], tree2->getCoords()[1]);
    if(tree1->getCoords() == startCoord)
      forward=true;
    else
      forward=false;

    if(forward){ // if forwards expansion

      printf("direction is forward  \n");
      if( tail1->getParent()!=nullptr ){

        finPath.push_front(tail1->getCoords());
        NodeRRT* traverse = tail1;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push_front( traverse->getCoords() );
        }
      }

      if(tail2->getParent()!=nullptr){

        finPath.push_back( tail2->getCoords() );
        NodeRRT* traverse = tail2;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push_back( traverse->getCoords() );
        }
      }
      finPath.pop_front(); finPath.pop_back();
      finPath.push_front(startCoord);
      finPath.push_back(endCoord);
    }
    else{

      printf("direction is backwards\n");
      if( tail2->getParent()!=nullptr ){

        finPath.push_front(tail2->getCoords());
        NodeRRT* traverse = tail2;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push_front( traverse->getCoords() );
        }
      }

      if(tail1->getParent()!=nullptr){

        finPath.push_back( tail1->getCoords() );
        NodeRRT* traverse = tail1;

        while(traverse->getParent()!=nullptr){

          traverse = traverse->getParent();
          finPath.push_back( traverse->getCoords() );
        }
      }
      finPath.pop_front(); finPath.pop_back();
      finPath.push_front(startCoord);
      finPath.push_back(endCoord);
    }

    
    int sizePath =finPath.size();
    *planlength = sizePath;

    // printf("finPath size is %d \n size of vector is %d \n", sizePath, finPath.front().size());
    // mexEvalString("drawnow");

    *plan = (double**) malloc(sizePath *sizeof(double*));
    int firstinvalidconf = 1;

    for(int i=0; i < sizePath; i++ ){

      if(finPath.empty()){

        // printf("broke \n");
        // mexEvalString("drawnow");
        break;
      }

      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 

      for(int j=0; j< numofDOFs; j++){

        (*plan)[i][j] = finPath.front()[j];
        // printf( "print finPath for i: %d, j: %d = %lf \n", i, j, finPath.front()[j]);
      }

      if( IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)==0 && firstinvalidconf)
      {
          firstinvalidconf = 1;
          printf("ERROR: Invalid arm configuration!!!\n");
      }

      finPath.pop_front();
    }

    // deallocate pointers
    if(tree1!=nullptr)
      delete tree1;
    
    if(tree2!=nullptr)
      delete tree2;

    if(tail1!=nullptr)
      delete tail1;

    if(tail2!=nullptr)
      delete tail2;

    // if(newNode1!=nullptr)
    //   delete newNode1;

    // if(newNode2!=nullptr)
    //   delete newNode2;

  }

  else{

    mexPrintf("either goal or start configs are invalid \n");
    mexEvalString("drawnow");
  }

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
                // printf("before calling plan i: %d, j: %d \n ", i, j); 
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

    // printf("mex function done \n"); 
    
    return;
    
}





