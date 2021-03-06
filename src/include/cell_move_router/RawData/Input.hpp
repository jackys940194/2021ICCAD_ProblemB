#pragma once
#include "Util/BaseType.hpp"
#include "cell_move_router/RawData/Datas.hpp"
#include <ostream>
#include <vector>

namespace cell_move_router {
namespace RawData {
class Input : Util::Outputable {
  const unsigned MaxCellMove;
  const int RowBeginIdx, ColBeginIdx, RowEndIdx, ColEndIdx;
  const std::vector<Layer> Layers;
  const std::vector<NonDefaultSupplyGGrid> NonDefaultSupplyGGrids;
  const std::vector<MasterCell> MasterCells;
  const std::vector<CellInst> CellInsts;
  const std::vector<Net> Nets;
  const std::vector<VoltageArea> VoltageAreas;
  const std::vector<Route> Routes;

public:
  Input(unsigned MaxCellMove, int RowBeginIdx, int ColBeginIdx, int RowEndIdx,
        int ColEndIdx, std::vector<RawData::Layer> &&Layers,
        std::vector<NonDefaultSupplyGGrid> &&NonDefaultSupplyGGrids,
        std::vector<MasterCell> &&MasterCells,
        std::vector<RawData::CellInst> &&CellInsts, std::vector<Net> &&Nets,
        std::vector<VoltageArea> &&VoltageAreas, std::vector<Route> &&Routes);
  void to_ostream(std::ostream &out) const override;
};
} // namespace RawData
} // namespace cell_move_router