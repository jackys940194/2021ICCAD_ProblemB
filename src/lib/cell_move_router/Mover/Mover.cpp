#include "cell_move_router/Mover/Mover.hpp"
#include "cell_move_router/Router/GraphApproxRouter.hpp"
#include <unordered_map>
#include <iostream>
#include "GlobalTimer.hpp"
#include <algorithm>

namespace cell_move_router {
namespace Mover {

long long Mover::CalculateCost(long long R1, long long C1, long long L1, long long R2, long long C2, long long L2, long long MRL, long long L){
    long long cost = 0;
    long long del_R, del_C, zero_R, zero_C;
    del_R = abs(R1-R2);
    del_C = abs(C1-C2);
    cost+=del_R;
    cost+=del_C;
    if(del_R>0){
        zero_R = 0;
    }else{
        zero_R = 1;
    }
    if(del_C>0){
        zero_C = 0;
    }else{
        zero_C = 1;
    }
    cost+=GridManager.getVLUT()[zero_R*2*L*L*L + zero_C*L*L*L + (L1-1)*L*L + (L2-1)*L + (MRL-1)];
    return cost;
}

long long Mover::lookUp(NetGraph::NetGraph& netGraph, const std::pair<int, int> pos, const Input::Processed::CellInst *CellPtr) {
  int R2 = pos.first;
  int C2 = pos.second;
  int L2 = 0;
  if(!netGraph.getNet()) return 0;
  for (const auto &pin : netGraph.getNet()->getPins()) {
      if (pin.getInst() == CellPtr) {
          L2 = pin.getMasterPin()->getPinLayer()->getIdx();
          break;
      }
  }
  long long minCost = std::numeric_limits<long long>::max();
  size_t R1, C1, L1;
  for (const auto &v : netGraph.getVertices()) {
    if (!v.isDeleted) {
      R1 = v.R;
      C1 = v.C;
      L1 = v.L;
      minCost = std::min(minCost, CalculateCost(R1, C1, L1, R2, C2, L2, netGraph.getNet()->getMinRoutingLayConstraint()->getIdx(), GridManager.getInputPtr()->getLayers().size()));
    }
  }
  return minCost;
}

long long Mover::removeSegmentGain(NetGraph::NetGraph& netGraph, const Input::Processed::CellInst *CellPtr) {
  return netGraph.route(CellPtr);
}

long long Mover::calCost(const std::pair<int, int> pos, const Input::Processed::CellInst *CellPtr) {
  long long Cost = 0;
  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
      Cost += NetPtr->getWeight() * lookUp(GridManager.getNetGraphs()[NetPtr], pos, CellPtr);
  }
  //std::cout<<pos.first<<" "<<pos.second<<" "<<Gain<<std::endl;
  return Cost;
}

void Mover::sortCandidatePos(std::vector<std::pair<int, int>> &CandidatePos, const Input::Processed::CellInst *CellPtr){
  std::unordered_map<size_t, long long> posGain;
  long long Gain = 0;

  // auto Timer = GlobalTimer::getInstance();
  // auto removeSegmetGainStart = Timer->getDuration<>().count();
  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
      Gain +=  NetPtr->getWeight() * removeSegmentGain(GridManager.getNetGraphs()[NetPtr], CellPtr);// - lookUp(GridManager.getNetGraphs()[NetPtr], CellPtr, pos);
  }
  // auto removeSegmetGainEnd = Timer->getDuration<>().count();
  // std::cerr << "removeSegmetGain Time: " << (removeSegmetGainEnd - removeSegmetGainStart)/1e9 << " seconds\n";

  // auto calCostStart = Timer->getDuration<>().count();
  for (const auto &pos : CandidatePos) {
    posGain[GridManager.coordinateTrans(pos.first, pos.second, 1)] = Gain - calCost(pos, CellPtr);
    //std::cout<<CellPtr->getInstName()<<" "<<pos.first<<" "<<pos.second<<" "<<posGain[GridManager.coordinateTrans(pos.first, pos.second, 1)]<<std::endl;
  }
  // auto calCostEnd = Timer->getDuration<>().count();
  // std::cerr << "calCost Time: " << (calCostEnd - calCostStart)/1e9 << " seconds\n";

  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
    GridManager.getNetGraphs()[NetPtr].reset();
  }
  
  CandidatePos.erase(remove_if(CandidatePos.begin(), CandidatePos.end(), [&](const std::pair<int, int> a) {
    return posGain.at(GridManager.coordinateTrans(a.first, a.second, 1)) <= 0;
  }), CandidatePos.end());
  
  auto CandidatePosCmp = [&](const std::pair<int, int> a, const std::pair<int, int> b) {
    return posGain.at(GridManager.coordinateTrans(a.first, a.second, 1)) > posGain.at(GridManager.coordinateTrans(b.first, b.second, 1)); 
  };
  std::sort(CandidatePos.begin(), CandidatePos.end(), CandidatePosCmp);
}

void Mover::initalFreqMovedCell() {
  for (auto &Cell : InputPtr->getCellInsts()) {
    if (Cell.isMovable()) {
      /*if (GridManager.getCellVoltageArea(&Cell).size())
        continue;*/ // TODO: handle Cell in VoltageArea
      FreqMovedCell.emplace(&Cell, 0);
    }
  }
}
bool Mover::add_and_route(const Input::Processed::CellInst *CellPtr,
                          const int Row, const int Col) {
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
  std::vector<std::pair<const Input::Processed::Net *, NetGraph::NetGraph>> OriginNetGraphs;
  bool Accept = true;
  for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
    auto &OriginRoute = GridManager.getNetRoutes()[NetPtr];
    auto &OriginNetGraph = GridManager.getNetGraphs()[NetPtr];
    OriginNetGraphs.emplace_back(NetPtr, std::move(OriginNetGraph));
    auto Pair = GraphApproxRouter.singleNetRoute(NetPtr, OriginRoute.first);
    OriginRoutes.emplace_back(NetPtr, std::move(OriginRoute));
    if (Pair.second == false) {
      Accept = false;
      break;
    }
    auto Cost = GridManager.getRouteCost(NetPtr, Pair.first);
    Input::Processed::Route::reduceRouteSegments(Pair.first);
    OriginRoute = {std::move(Pair.first), Cost};
    bool Overflow = GridManager.isOverflow();
    GridManager.addNet(NetPtr);
    assert(!GridManager.isOverflow());
  }
  if (Accept) {
    return true;
  }
  GridManager.getNetGraphs()[OriginNetGraphs.back().first] = std::move(OriginNetGraphs.back().second);
  GridManager.getNetRoutes()[OriginRoutes.back().first] =
      std::move(OriginRoutes.back().second);
  OriginRoutes.pop_back();
  OriginNetGraphs.pop_back();
  while (OriginRoutes.size()) {
    GridManager.removeNet(OriginRoutes.back().first);
    GridManager.getNetRoutes()[OriginRoutes.back().first] =
        std::move(OriginRoutes.back().second);
    OriginRoutes.pop_back();
  }
  while (OriginNetGraphs.size()) {
    GridManager.getNetGraphs()[OriginNetGraphs.back().first] = std::move(OriginNetGraphs.back().second);
    OriginNetGraphs.pop_back();
  }
  GridManager.removeCell(CellPtr);
  return false;
}
void Mover::move(RegionCalculator::RegionCalculator &RC, int Round) {
  auto Timer = GlobalTimer::getInstance();
  unsigned MaxMoveCnt = InputPtr->getMaxCellMove();
  for (auto &P : FreqMovedCell) {
    if (P.second != 0)
      MaxMoveCnt -= P.second;
  }

  if(Round >= 1) assert(MaxMoveCnt == 0);

  std::vector<std::pair<long long, const Input::Processed::CellInst *>>
      CellNetLength;
    assert(CellNetLength.size() == 0);
  for (auto &P : FreqMovedCell) {
    auto CellPtr = P.first;
    long long NetLength = 0;

    if (MaxMoveCnt == 0 && FreqMovedCell.at(CellPtr) == 0) continue;

    for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
      NetLength += GridManager.getNetRoutes()[NetPtr].second;
    }
    CellNetLength.emplace_back(NetLength, CellPtr);
  }
  std::sort(
      CellNetLength.begin(), CellNetLength.end(),
      std::greater<std::pair<long long, const Input::Processed::CellInst *>>());
  unsigned MoveCnt = (Round == 0) ? 0 : -1;
  for (auto &P : CellNetLength) {
    auto CellPtr = P.second;
    int RowBeginIdx = 0, RowEndIdx = 0, ColBeginIdx = 0, ColEndIdx = 0;
    std::vector<std::pair<int, int>> CandidatePos;
    if(GridManager.getCellVoltageArea(CellPtr).size()){
      std::tie(RowBeginIdx, RowEndIdx, ColBeginIdx, ColEndIdx) =
          RC.getRegion(CellPtr);
      for(auto coord : GridManager.getCellVoltageArea(CellPtr)){
        int R = 0, C = 0, L = 0;
        std::tie(R,C,L) = GridManager.coordinateInv(coord);
        if(R >= RowBeginIdx && R <= RowEndIdx && C >= ColBeginIdx && C <= ColEndIdx)
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
    //auto Timer = GlobalTimer::getInstance();
    //auto sortStart = Timer->getDuration<>().count();
    sortCandidatePos(CandidatePos, CellPtr);
    //auto sortEnd = Timer->getDuration<>().count();
    //std::cerr << "Sort CandidatePos Time: " << (sortEnd - sortStart)/1e9 << " seconds\n";
    //std::shuffle(CandidatePos.begin(), CandidatePos.end(), Random);
    auto OldCoord = GridManager.getCellCoordinate(CellPtr);
    {
      for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
        GridManager.removeNet(NetPtr);
      }
      GridManager.removeCell(CellPtr);
    }
    bool Success = false;
    int candidateCnt = 0;
    for (auto P : CandidatePos) {
      if (Timer->overTime()) break;
      if (add_and_route(CellPtr, P.first, P.second)) {
        Success = true;
        break;
      }
    }
    if (Success){
      if (FreqMovedCell.at(CellPtr) == 0) {
          ++MoveCnt;
          FreqMovedCell[CellPtr]++;
      }
      //std::cout<<"MoveCnt : "<<MoveCnt<<std::endl;
    }else {
      GridManager.addCell(CellPtr, OldCoord.first, OldCoord.second);
      for (auto NetPtr : InputPtr->getRelativeNetsMap().at(CellPtr)) {
        GridManager.addNet(NetPtr);
      }
    }
    if (Timer->overTime()) break;
    if (MoveCnt == MaxMoveCnt)
      break;
  }
}

} // namespace Mover
} // namespace cell_move_router