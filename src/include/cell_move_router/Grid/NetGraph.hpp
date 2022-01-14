#pragma once
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

namespace cell_move_router {
namespace Grid{
    class GridManager;
}

namespace NetGraph {
    struct Edge;

    struct Vertex {
        Vertex(size_t R = 0, size_t C = 0, size_t L = 0, size_t id = 0): R(R), C(C), L(L), id(id) {}
        size_t R, C, L;
        size_t id;
        bool isReal;
        bool isDeleted;
    };
    
    struct Edge {

        size_t u, v;
        long long weight;
        
        Edge(size_t u, size_t v, long long weight): u(u), v(v), weight(weight) {}
    };

    class NetGraph {
        const Input::Processed::Net *Net; //may be nullptr ---> Eids == 0
        Grid::GridManager *GridManager;
        std::vector<Vertex> Vertices;
        std::vector<std::vector<Edge>> adj_list;
        std::map<std::tuple<size_t, size_t, size_t>, size_t> RCL_to_id;
      public:
        NetGraph() {}
        NetGraph(const Input::Processed::Net *Net, Grid::GridManager *GridManager): Net(Net), GridManager(GridManager) {}  
        void setVertexNum(size_t num) { adj_list.resize(num); Vertices.resize(num); }
        void addEdge(size_t u, size_t v, long long weight) {
            adj_list[u].emplace_back(u, v, weight);
            adj_list[v].emplace_back(v, u, weight);
        }
        void addVertex(size_t R, size_t C, size_t L, size_t id) {
            Vertices[id] = Vertex(R, C, L, id);
            RCL_to_id[{R, C, L}] = id;
        }
        void setIsReal();
    };
}
}