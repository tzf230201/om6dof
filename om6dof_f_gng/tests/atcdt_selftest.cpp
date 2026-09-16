#include "atcdt_core.hpp"

#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

std::vector<atcdt::Point> grid(float z) {
    std::vector<atcdt::Point> result;
    for (int y=-3;y<=3;++y) for (int x=-3;x<=3;++x)
        result.push_back({.01F*x,.01F*y,z});
    return result;
}
}

int main() try {
    atcdt::Config fine; fine.vigilance=.015F; fine.maxNodes=256;
    atcdt::Config coarse=fine; coarse.vigilance=.05F;
    atcdt::Mapper fineMap(fine),coarseMap(coarse);
    for (int frame=0;frame<5;++frame) { fineMap.learn(grid(0)); coarseMap.learn(grid(0)); }
    require(fineMap.nodeCount()>coarseMap.nodeCount(),"vigilance does not control metric node granularity");
    const auto graph=fineMap.positionGraph();
    require(graph.nodes.size()==graph.ids.size() && !graph.edges.empty(),"position topology was not learned");
    bool planarNormal=false;
    for (const auto& normal:fineMap.normals()) planarNormal|=normal.z>.9F;
    require(planarNormal,"PCA normal and traversability attribute were not learned from position topology");
    for (const auto& [a,b]:fineMap.traversabilityEdges())
        require(a<graph.nodes.size() && b<graph.nodes.size(),"invalid traversability topology index");
    std::cout << "ATC-DT: vigilance, position topology, PCA normal and traversability topology passed\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr<<error.what()<<'\n'; return 1;
}
