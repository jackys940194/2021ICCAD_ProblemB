#pragma once
#include "cell_move_router/Grid/GridManager.hpp"
#include "cell_move_router/Input/Processed/Input.hpp"
#include "cell_move_router/RegionCalculator/RegionCalculator.hpp"
#include <random>
#include <unordered_map>

namespace cell_move_router {
namespace Mover {

class Mover {
  Grid::GridManager &GridManager;
  const Input::Processed::Input *InputPtr;
  std::default_random_engine Random;
  std::unordered_map<const Input::Processed::CellInst *, int> FreqMovedCell;

  void initalFreqMovedCell();

public:
  Mover(Grid::GridManager &GridManager)
      : GridManager(GridManager), InputPtr(GridManager.getInputPtr()) {
    Random.seed(7122);
    initalFreqMovedCell();
  }

  long long calGain(const std::pair<int, int> pos, const Input::Processed::CellInst *CellPtr);

  void sortCandidatePos(std::vector<std::pair<int, int>> &CandidatePos, const Input::Processed::CellInst *CellPtr);

  bool add_and_route(const Input::Processed::CellInst *CellPtr, const int Row,
                     const int Col, long long OgCost);
  void move(RegionCalculator::RegionCalculator &RC, int Round);
};

} // namespace Mover
} // namespace cell_move_router