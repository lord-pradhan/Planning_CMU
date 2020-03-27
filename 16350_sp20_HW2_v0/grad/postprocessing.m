%% this script executes post-processing for Hw2
%Author - Roshan Pradhan
clear all
close all
clc

mex planner.cpp utilheader.cpp
%% initialize
rng('shuffle');

for i=1:200
    numDof = 2 + randi(5);
    armstart = 2*pi*rand(1, numDof);
    armgoal = 2*pi*rand(1, numDof);
    
    disp(['Start config is ', num2str(armstart)])
    disp(['Goal config is ', num2str(armgoal)])
    
    planner_id = 0;
    runtest('map2.txt', armstart, armgoal, planner_id);
    
    planner_id = 1;
    runtest('map2.txt', armstart, armgoal, planner_id);

    planner_id = 2;
    runtest('map2.txt', armstart, armgoal, planner_id);

    planner_id = 3;
    runtest('map2.txt', armstart, armgoal, planner_id);
    
    disp(['Run number ', num2str(i)])

end
