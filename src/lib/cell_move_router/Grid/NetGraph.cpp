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
#include <iostream>
#include <queue>
namespace cell_move_router{
namespace NetGraph{

void NetGraph::reset() {
    for (auto& v : Vertices)
        v.isDeleted = false;
}

long long NetGraph::route(const Input::Processed::CellInst *CellPtr) {
    if (Net == nullptr) return 0;
    int R = 0, C = 0, L = 0;
    std::tie(R, C) = GridManager->getCellCoordinate(CellPtr);
    for (const auto &pin : Net->getPins()) {
        if (pin.getInst() == CellPtr) {
            L = pin.getMasterPin()->getPinLayer()->getIdx();
            break;
        }
    }
    assert(RCL_to_id.count({R, C, L}));
    size_t source = RCL_to_id.at({R,C,L});
    if (adj_list[source].size() != 1) return 0;
    int parent[N];
    for (int i=0; i<N; ++i)
        parent[i] = -1;
    std::queue<std::pair<long long, size_t>> q;
    q.push({0, source});
    long long ans = 0;
    size_t end = 0;
    while(q.size()) {
        auto p = q.front(); q.pop();
        if (Vertices[p.second].isReal && p.second != source) {
            ans = p.first;
            end = p.second;
            break;
        }
        for (auto& edge : adj_list[p.second]) {
            if (int(edge.v) == parent[p.second]) continue;
            q.push({p.first+edge.weight, edge.v});
            parent[edge.v] = p.second;
        }
        assert(q.size() == 1);
    }
    while (parent[end] != -1) {
        Vertices.at(end).isDeleted = true;
        end = parent[end];
    }
    return ans;
}

void NetGraph::setIsReal() {
    for (auto& v : Vertices) {
        if (adj_list[v.id].size() >= 3)
            v.isReal = true;
    }
    for (auto &Pin : Net->getPins()) {
        auto CellPtr = Pin.getInst();
        int R = 0, C = 0;
        std::tie(R, C) = GridManager->getCellCoordinate(CellPtr);
        int oldL = Pin.getMasterPin()->getPinLayer()->getIdx();
        int L = oldL;
        int i=1, j=-1;
        while (!RCL_to_id.count({R, C, L})) {
            L=L+j*i;
            j*=-1;
            i++;
        }
        RCL_to_id[{R, C, oldL}] = RCL_to_id.at({R, C, L});
        Vertices.at(RCL_to_id.at({R, C, L})).isReal = true;
    }
}

}
}