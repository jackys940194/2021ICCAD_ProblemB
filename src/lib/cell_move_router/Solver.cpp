#include "cell_move_router/Solver.hpp"
#include "cell_move_router/Mover/Mover.hpp"
#include "cell_move_router/RegionCalculator/FinalRegion.hpp"
#include "cell_move_router/Router/GraphApproxRouter.hpp"
#include <iostream>
#include "GlobalTimer.hpp"

namespace cell_move_router {
void Solver::solve() {
  cell_move_router::Router::GraphApproxRouter Router(&GridManager);

  long long OriginalCost = GridManager.getCurrentCost();
  std::cerr << "Original:\n";
  std::cerr << "  CurrentCost: " << OriginalCost * 0.0001 << '\n';

  Router.rerouteAll();
  long long FirstRerouteCost = GridManager.getCurrentCost();
  std::cerr << "After First re-route:\n";
  std::cerr << "  CurrentCost: " << FirstRerouteCost * 0.0001 << '\n';
  std::cerr << "  ReducedCost: " << (OriginalCost - FirstRerouteCost) * 0.0001
            << '\n';

  Mover::Mover Mover(GridManager);
  // TODO: fix FinalRegion infinite loop bug
  // RegionCalculator::FinalRegion FR(&GridManager);
  RegionCalculator::OptimalRegion OR(&GridManager);
  long long nowCost = GridManager.getCurrentCost();
  long long newCost;
  int round = 0;
  bool improved;
  //Grid::GridManager BestGridManager(GridManager);
  auto Timer = GlobalTimer::getInstance();
  do {
    improved = false;
    Mover.move(OR, round++);
    newCost = GridManager.getCurrentCost();
    std::cerr << "Move: "<<round-1<<" : "<<newCost<<std::endl;
    if (newCost < nowCost) {
      improved = true;
      nowCost = newCost;
      //BestGridManager = GridManager.copy();
    }
  } while(improved && !Timer->overTime());
  //BestGridManager = GridManager.copy();
  long long MoveCost = GridManager.getCurrentCost();
  std::cerr << "After move:\n";
  std::cerr << "  CurrentCost: " << MoveCost * 0.0001 << '\n';
  std::cerr << "  ReducedCost: " << (FirstRerouteCost - MoveCost) * 0.0001
            << '\n';
}
} // namespace cell_move_router