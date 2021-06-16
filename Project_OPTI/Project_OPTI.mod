/*********************************************
 * OPL 12.10.0.0 Model
 * Author: Gaelle AbiYounes
 * Creation Date: Jun 14, 2021 at 2:25:14 PM
 *********************************************/

/* Define sets and parameters */
int N = ...;
int V= ...;
int S=...;
int Q=...;
int q_unit=...;
int u=...;
int mMax =...;
int T=...;

range store =1..N;
range node = 1..N+1;
range destination = 2..N+1;
range vehicle = 1..V;
range segment = 1..S; 
range multiple = 1..mMax;
range num_segment = 1..S;
int l[num_segment]=...;
float c[node][node]=...;
int t[node][node]=...;
int d[store][segment]=...;
int ser[store]=...;
int k[vehicle];


/* Define Decision Variables */
dvar int x[node][node][vehicle];    /* is equal to 1 if a vehicle v travels from node i to j, 0 otherwise */
dvar int del[store][segment][vehicle];   /* is equal to 1 if a segment is delivered by vehicle v to store i, 0 otherwise */
dvar int y[segment][vehicle][multiple];  /* is equal to 1 if size m is selected for segment s in vehicle v, 0 otherwise*/


/* Objective Function */
minimize sum (i in node, j in node, v in vehicle) (x[i][j][v]*c[i][j]) + sum (i in node, j in destination, v in vehicle) x[i][j][v]*u + sum (v in vehicle) (l[k[v]]);
/* minimize travel, unloading and loading costs */

/* Constraints */
subject to {
  forall (j in node, v in vehicle) {
    sum (i in node) x[i][j][v] <=1;    /* every vehicle can enter a node j at most one time */
  }
  forall (i in node, v in vehicle) {
    sum (j in node) x[i][j][v] <=1;   /* every vehicle can leave a node i at most one time */
  }
  sum (j in destination, v in vehicle) x[1][j][v] == V;  /* 4 vehicles should depart from the warehouse */
  
  forall (v in vehicle, h in node) {
    sum (i in node) x[i][h][v]- sum (j in node) x[h][j][v] == 0 ;   /* at every node, inflow = outflow. all vehicles that enter a node have to leave it */
  }
  forall (v in vehicle){
    (sum (i in node, j in node) x[i][j][v]*t[i][j] + sum (i in node, j in destination, s in store) x[i][j][v]*ser[s]) <= T; 
    /* traveling and service time for all vehicles should be less than T */
  }
  forall (v in vehicle){
    sum (s in segment, m in multiple) y[s][v][m]*m*q_unit <= Q; 
    /* for all vehicles, what is transported should not exceed capacity Q */
  }
  forall (v in vehicle){
    sum (m in multiple, s in segment) y[s][v][m]>=1; /*for all vehicles,  at least one segment should be transported*/
  }
  
  forall (v in vehicle, s in segment) {
    sum (i in store) del[i][s][v] * d[i][s] == sum (m in multiple) y[v][s][m]*m*q_unit;  
    /* sum of the demand that is delivered at all stores, for a specific segment, is equal to what is transported */
  }
  
  forall (s in segment, i in store){
  	sum (v in vehicle) del[i][s][v] <= 1;    /* a specific segment is only delivered by 1 vehicle for every store to ensure no demand split */
  }  
  forall (v in vehicle, s in segment){
    sum (m in multiple) y[s][v][m]<=1;      /* for all vehicles, only one size/multiple should be selected for each segment */
  }
  forall (v in vehicle){
  	k[v] == sum (s in segment) z[s][v];
  }
  
  forall (i in node, j in node, v in vehicle, m in multiple, s in segment, p in store){
    x[i][j][v]>=0;    /* all variables should be positive */
	del[p][s][v]>=0;
	y[s][v][m]>=0;
	z[s][v]>=0;
	stop[v][s]>=0;
  }
}






