#include "local_planner/mpc_base.hpp"

MPCBase::MPCBase()
{
    // ...
}

MPCBase::set_local_costmap(const grid_map::GridMap& local_costmap) { local_costmap_ = local_costmap; }

MPCBase::set_global_costmap(const grid_map::GridMap& global_costmap) {global_costmap_ = global_costmap; }