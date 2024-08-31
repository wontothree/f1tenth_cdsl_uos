// Anthony Garcia

#pragma once

#include <grid_map_core/GridMap.hpp>

class MPCBase
{
public:
    MPCBase();
    ~MPCBase() {};

    void set_local_costmap(const grid_map::GridMap& local_costmap);

    void set_global_costmap(const grid_map::GridMap& global_costmap);

private:
    grid_map::GridMap local_costmap_;
    grid_map::GridMap global_costmap_;
}