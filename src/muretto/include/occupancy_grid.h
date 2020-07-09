#ifndef SRC_OCCUPANCY_GRID_H
#define SRC_OCCUPANCY_GRID_H

#endif //SRC_OCCUPANCY_GRID_H

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>



using namespace std;

namespace occupancy_grid {
    int xy_ind2ind(const nav_msgs::OccupancyGrid &grid, int x_ind, int y_ind);

    int xy2ind(const nav_msgs::OccupancyGrid &grid, float x, float y);

    struct Pair {
        int x_ind;
        int y_ind;
    };

    Pair ind2xy_ind(const nav_msgs::OccupancyGrid &grid, int ind);
    float ind2x(const nav_msgs::OccupancyGrid &grid, int ind);

    float ind2y(const nav_msgs::OccupancyGrid &grid, int ind) ;

    bool is_xy_occupied(nav_msgs::OccupancyGrid &grid, float x, float y);

    void set_xy_occupied(nav_msgs::OccupancyGrid &grid, float x, float y);

    void inflate_cell(nav_msgs::OccupancyGrid &grid, int i, float margin, int val);

    void inflate_map(nav_msgs::OccupancyGrid &grid, float margin);

    double calculate_dist2(double x1, double x2, double y1, double y2);
}
