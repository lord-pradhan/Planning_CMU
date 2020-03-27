%% 
clear all
clc

DOF = [2 2 2 2 2 3 4 5 2 2 3 3 5 3 4 4 3 4];
dof_av = mean(DOF)

%%

RRT_cost = [ 7.808961  6.91 30.657415 17.820411 35.97 12.6 36.9 10.12 45.54 27.5 85.9 30.25 26.4 52.95 40.38 19.47 65 33.6];
RRT_avcost = mean(RRT_cost)
    
RRT_connect_cost = [7.288493 7.3 51.328207 18.073033 54.19 11.76 36.19 12.33 52.5 43.45 73.99 59 27.17 83.424 68.66 19.34 33 25.67];
RRT_c_avcost = mean(RRT_connect_cost)

RRT_star_cost = [ 7.162090 6.4 46.845197 16.836310 53.76 13.25 35.02 9.7 28.37 45.14 58.7 25.75 26.52 40.35 44.1 20.12 44 28.5];
RRT_s_avcost = mean(RRT_star_cost)

PRM_cost = [ 8.278051 7.03 33.531431 15.794583 25.76 12.43 30.65 8.6 37.29 17.02 24.8 15.97 22.65 56.48 20.58 21.6 21.3 22.4];
PRM_avcost= mean(PRM_cost)
%%

RRT_size = [10 10 132 21 146 85 33 9 213 200 486 451 21 382 1124 27 574 672];
RRT_avsize = mean(RRT_size)

RRT_connect_size = [2 2 240 2 296 6 8 2 427 291 1818 644 2 862 1912 2 249 18];
RRT_c_avsize = mean(RRT_connect_size)

RRT_star_size = [ 200 200 200 200 200 200 200 200 200 228 602 250 200 672 327 200 200 1304] ;
RRT_s_avsize = mean(RRT_star_size)

PRM_size = [ 138 138 138 138 138 318 1398 1026 138 138 318 318 1026 318 1398 1398 318 1398];
PRM_avsize = mean(PRM_size)

%%

RRT_time = [ 125 108 6581 229 8955 3242 829 287 18727 17541 173065 69497 658 106884 577996 676 160656 157277];
RRT_time_avtime=mean(RRT_time)/1e3

RRT_connect_time = [133 72 10613 196 14268 777 830 297 32258 14495 436673 65983 529 125008 479346 516 12721 953];
RRT_c_avtime = mean(RRT_connect_time)/1e3

RRT_star_time_tot = [219637 158120 116608 280347 96261 331822 492719 691100 138207 187041 2692379 276655 702170 2972639 514725 598032 232987 6875139];
RRT_s_avtime = mean(RRT_star_time_tot)/1e3

RRT_star_time_improve = [219564 158053 90 280186 78 327721 490791 690721 1967 86 245 145 701468 250 186 597258 73069 393];
RRT_s_avimprove = mean(RRT_star_time_improve)/1e3

PRM_time_tot = [46439 37406 42577 43298 52301 192502 2807408 1440856 44664 59641 181089 178832 1608326 208164 3035267 2940196 196939 2619424];
PRM_avtime = mean(PRM_time_tot)/1e3

PRM_time_road = [23054 21509 21494 22801 23181 113614 2321568 1002313 25556 27414 116526 115368 1072618 117955 2515469 2369852 115244 2254571];
PRM_roadratio = PRM_time_road ./ PRM_time_tot
road_av = mean(PRM_roadratio)

PRM_time_query = PRM_time_tot - PRM_time_road
query_av = 1 - road_av

RRT_short = RRT_time/5e6
RRT_c_short = RRT_connect_time/5e6
RRT_star_short = RRT_star_time_tot/5e6
PRM_short = PRM_time_tot/5e6

%%

RRT_long = 1 - 2/22

RRT_connect_long = 1

RRT_star_long = 1 - 3/22

PRM_long = 1 - 1/22

