#include "cell_move_router/Grid/GridManager.hpp"
#include "Util/BaseType.hpp"
#include "cell_move_router/CoordinateCodec.hpp"
#include "cell_move_router/Grid/CellGrid.hpp"
#include "cell_move_router/Grid/Grid.hpp"
#include "cell_move_router/Input/Processed/Input.hpp"
#include <limits>
#include <ostream>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>

namespace cell_move_router{
namespace NetGraph{

void NetGraph::setIsReal() {
    for (auto& v : Vertices) {
        if (adj_list[v.id].size() >= 3)
            v.isReal = true;
    }
    for (auto &Pin : Net->getPins()) {
        auto CellPtr = Pin.getInst();
        int R = 0, C = 0;
        std::tie(R, C) = GridManager->getCellCoordinate(CellPtr);
        int L = Pin.getMasterPin()->getPinLayer()->getIdx();
        Vertices[RCL_to_id[{R, C, L}]].isReal = true;
    }
}

}
}