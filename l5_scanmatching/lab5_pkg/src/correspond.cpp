#include "scan_matching_skeleton/correspond.h"
#include "cmath"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();
  int min_index = 0;
  int second_min_index = 0;

  //Do for each point
  for (int ind_trans = 0; ind_trans < n; ++ind_trans)
  {
    float min_dist = 100000.00;
    for (int ind_old = 0; ind_old < m; ++ind_old)
    {
      float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = ind_old;
        if (ind_old == 0)
        {
          second_min_index = ind_old + 1;
        }
        else
        {
          second_min_index = ind_old - 1;
        }
      }
    }
    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
  }
}

void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int trans_size = trans_points.size();
  const int old_size = old_points.size();

  //Do for each point
  for (int ind_trans = 0; ind_trans < min(old_size, trans_size); ++ind_trans)
  {
    /// TODO: Implement Fast Correspondence Search
    int best = -1;
    int second_best = -1;

    double best_dist = __DBL_MAX__;

    // approximated index in scan y(t-1) corresponding to pi_w 
    int start_index = (trans_points[ind_trans].theta - old_points[0].theta) / (old_points.size() / 2 * M_PI);

    // If last match was succesful, then start at that index + 1
    int we_start_at = (last_best != -1) ? (last_best + 1) : start_index;

    // Search is conducted in two directions: up and down
    int up = we_start_at + 1, down = we_start_at;
 
    // True if search is finished in the up (down) direction.
    double last_dist_up = __DBL_MAX__, last_dist_down = __DBL_MAX__;
    
    // True if search is finished in the up (down) direction.
    bool up_stopped = false, down_stopped = false;

    // Until the search is stopped in both directions...
    while ( !(up_stopped && down_stopped) ) 
    { 
      // printf("up_stopped %d | down_stopped %d \r\n", up_stopped, down_stopped);
      // Should we try to explore up or down?
      bool now_up = !up_stopped & (last_dist_up <= last_dist_down); 
      
      // Now two symmetric chunks of code, the now_up and the !now_up
      if(now_up) 
      {
        // If we have finished the points to search, we stop.
        if(up >= old_points.size()) { up_stopped = true; continue; } 
        
        // This is the distance from pwi to the up point.
        last_dist_up = trans_points[ind_trans].distToPoint2(&trans_points[up]);

        // If it is less than the best point, up is our best guess so far.
        if(last_dist_up < best_dist) 
          best = up, best_dist = last_dist_up;

        if(up > start_index) {
          // If we are moving away from start_cell we can compute a
          // bound for early stopping. Currently our best point has 
          // distance best_dist; we can compute the minimum distance
          // to pwi for points j > up (see figure 4(c)).
          double delta_theta = trans_points[up].theta - trans_points[ind_trans].theta;
          double min_dist_up = sin(delta_theta) * trans_points[ind_trans].r;

          if( pow(min_dist_up,2) > best_dist) {
            // If going up we can’t make better than best_dist,
            // then we stop searching in the "up" direction
            up_stopped = true; 
            continue;
          }
            // If we are moving away, then we can implement the jump tables optimization.
            up = (trans_points[up].r < trans_points[ind_trans].r) ? jump_table[up][UP_BIG] : jump_table[up][UP_SMALL];
        } 
        else {
          // If we are moving towards "start_cell", we can’t do any ot the
          // previous optimizations and we just move to the next point.
          up++;
        } 
      }
      // Now down
      else
      {
        // If we have finished the points to search, we stop.
        if(down < 0) { down_stopped = true; continue; } 
        
        // This is the distance from pwi to the down point.
        last_dist_down = trans_points[ind_trans].distToPoint2(&trans_points[down]);

        // If it is less than the best point, down is our best guess so far.
        if(last_dist_down < best_dist) 
          best = down, best_dist = last_dist_down;

        if(down < start_index) {
          // If we are moving away from start_cell we can compute a
          // bound for early stopping. Currently our best point has 
          // distance best_dist; we can compute the minimum distance
          // to pwi for points j < down (see figure 4(c)).
          double delta_theta = trans_points[down].theta - trans_points[ind_trans].theta;
          double min_dist_down = sin(delta_theta) * trans_points[ind_trans].r;

          if( pow(min_dist_down,2) > best_dist) {
            // If going down we can’t make better than best_dist,
            // then we stop searching in the "down" direction
            down_stopped = true; 
            continue;
          }
            // If we are moving away, then we can implement the jump tables optimization.
            down = (trans_points[down].r < trans_points[ind_trans].r) ? jump_table[down][DOWN_BIG] : jump_table[down][DOWN_SMALL];
        } 
        else {
          // If we are moving towards "start_cell", we can’t do any of the
          // previous optimizations and we just move to the next point.
          down--;
        } 
        
      }
    }
    
    // For the next point, we will start at best
    last_best = best;

    if(best != -1)
      c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[best], &old_points[second_best]));
  }
}

void computeJump(vector<vector<int>> &table, vector<Point> &points)
{
  table.clear();
  int n = points.size();
  for (int i = 0; i < n; ++i)
  {
    vector<int> v = {n, n, -1, -1};
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r < points[i].r)
      {
        v[UP_SMALL] = j;
        break;
      }
    }
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r > points[i].r)
      {
        v[UP_BIG] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r < points[i].r)
      {
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r > points[i].r)
      {
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
