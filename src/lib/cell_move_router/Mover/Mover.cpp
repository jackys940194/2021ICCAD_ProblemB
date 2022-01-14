#include "cell_move_router/Mover/Mover.hpp"
#include "cell_move_router/Router/GraphApproxRouter.hpp"
#include <unordered_map>

namespace cell_move_router {
namespace Mover {

long long Mover::calGain(const std::pair<int, int> pos, const Input::Processed::CellInst *CellPtr) {
  long long Gain = 0;
  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
      //Gain += removeSegmentGain(GridManager->getNetGraphs()[NetPtr], CellPtr) - lookUp(GridManager->getNetGraphs()[NetPtr], CellPtr, pos);
  }
  return Gain;
}

void Mover::sortCandidatePos(std::vector<std::pair<int, int>> &CandidatePos, const Input::Processed::CellInst *CellPtr){
  std::unordered_map<size_t, long long> posGain;

  for (const auto &pos : CandidatePos) {
    posGain[GridManager.coordinateTrans(pos.first, pos.second, 1)] = calGain(pos, CellPtr);
  }

  auto CandidatePosCmp = [&](const std::pair<int, int> a, const std::pair<int, int> b) {
    
    return posGain.at(GridManager.coordinateTrans(a.first, a.second, 1)) > posGain.at(GridManager.coordinateTrans(b.first, b.second, 1)); 
  };
  std::sort(CandidatePos.begin(), CandidatePos.end(), CandidatePosCmp);
}

void Mover::initalFreqMovedCell() {
  for (auto &Cell : InputPtr->getCellInsts()) {
    if (Cell.isMovable()) {
      //if (GridManager.getCellVoltageArea(&Cell).size())
        //continue; // TODO: handle Cell in VoltageArea
      FreqMovedCell.emplace(&Cell, 0);
    }
  }
}
bool Mover::add_and_route(const Input::Processed::CellInst *CellPtr,
                          const int Row, const int Col, long long OgCost) {
  GridManager.addCell(CellPtr, Row, Col);
  if (GridManager.isOverflow()) {
    GridManager.removeCell(CellPtr);
    return false;
  }
  Router::GraphApproxRouter GraphApproxRouter(&GridManager);
  std::vector<std::pair<
      const Input::Processed::Net *,
      std::pair<std::vector<cell_move_router::Input::Processed::Route>,
                long long>>>
      OriginRoutes;
  bool Accept = true, flag = true;
  auto total = 0;
  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
    auto &OriginRoute = GridManager.getNetRoutes()[NetPtr];
    auto Pair = GraphApproxRouter.singleNetRoute(NetPtr, OriginRoute.first);
    OriginRoutes.emplace_back(NetPtr, std::move(OriginRoute));
    if (Pair.second == false) {
      Accept = false;
      break;
    }
    auto Cost = GridManager.getRouteCost(NetPtr, Pair.first);
    total += Cost;
    Input::Processed::Route::reduceRouteSegments(Pair.first);
    OriginRoute = {std::move(Pair.first), Cost};
    bool Overflow = GridManager.isOverflow();
    GridManager.addNet(NetPtr);
    assert(!GridManager.isOverflow());
  }
  if(Accept && total > OgCost){
    Accept = false;
    flag = false;
  }
  if (Accept) {
    return true;
  }
  if(flag){
    GridManager.getNetRoutes()[OriginRoutes.back().first] =
        std::move(OriginRoutes.back().second);
    OriginRoutes.pop_back();
  }
  while (OriginRoutes.size()) {
    GridManager.removeNet(OriginRoutes.back().first);
    GridManager.getNetRoutes()[OriginRoutes.back().first] =
        std::move(OriginRoutes.back().second);
    OriginRoutes.pop_back();
  }
  GridManager.removeCell(CellPtr);
  return false;
}
void Mover::move(RegionCalculator::RegionCalculator &RC, int Round) {
  std::vector<std::pair<long long, const Input::Processed::CellInst *>>
      CellNetLength;
  for (auto &P : FreqMovedCell) {
    auto CellPtr = P.first;
    long long NetLength = 0;
    for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
      NetLength += GridManager.getNetRoutes()[NetPtr].second;
    }
    CellNetLength.emplace_back(NetLength, CellPtr);
  }
  std::sort(
      CellNetLength.begin(), CellNetLength.end(),
      std::greater<std::pair<long long, const Input::Processed::CellInst *>>());
  unsigned MoveCnt = 0;
  for (auto &P : CellNetLength) {
    auto CellPtr = P.second;
    auto OgCost = P.first; 
    int RowBeginIdx = 0, RowEndIdx = 0, ColBeginIdx = 0, ColEndIdx = 0;
    std::vector<std::pair<int, int>> CandidatePos;
    if(GridManager.getCellVoltageArea(CellPtr).size()){
      for(auto coord : GridManager.getCellVoltageArea(CellPtr)){
        int R = 0, C = 0, L = 0;
        std::tie(R,C,L) = GridManager.coordinateInv(coord);
        CandidatePos.emplace_back(R, C);
      }
    }
    else{
      std::tie(RowBeginIdx, RowEndIdx, ColBeginIdx, ColEndIdx) =
          RC.getRegion(CellPtr);
      for (int R = RowBeginIdx; R <= RowEndIdx; ++R) {
        for (int C = ColBeginIdx; C <= ColEndIdx; ++C) {
          CandidatePos.emplace_back(R, C);
        }
      }
    }
    sortCandidatePos(CandidatePos, CellPtr);
    //std::shuffle(CandidatePos.begin(), CandidatePos.end(), Random);
    auto OldCoord = GridManager.getCellCoordinate(CellPtr);
    {
      for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
        GridManager.removeNet(NetPtr);
      }
      GridManager.removeCell(CellPtr);
    }
    bool Success = false;
    for (auto P : CandidatePos) {
      if (add_and_route(CellPtr, P.first, P.second, OgCost)) {
        Success = true;
        break;
      }
    }
    if (Success)
      ++MoveCnt;
    else {
      GridManager.addCell(CellPtr, OldCoord.first, OldCoord.second);
      for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
        GridManager.addNet(NetPtr);
      }
    }
    if (MoveCnt == InputPtr->getMaxCellMove())
      break;
  }
}

} // namespace Mover
} // namespace cell_move_router