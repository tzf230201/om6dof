#include "bio_mapper.hpp"
#include <iostream>
#include <set>

namespace {
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
bio_fgng::DepthFrame plane(double time=0) {
    bio_fgng::DepthFrame f;
    f.intrinsics={24,18,22,22,11.5F,8.5F}; f.timestamp=time;
    f.depth.assign(24*18,2.0F); return f;
}
void checkGraph(const world_fgng::Graph& g,int cap) {
    require(g.nodes.size()<=static_cast<size_t>(cap),"node budget");
    require(g.nodes.size()==g.ids.size(),"ID/position size mismatch");
    std::set<int64_t> ids(g.ids.begin(),g.ids.end());
    require(ids.size()==g.ids.size(),"duplicate live IDs");
    for(const auto& p:g.nodes) require(world_fgng::finite(p),"nonfinite node");
    for(const auto& [a,b]:g.edges) require(a<g.nodes.size() && b<g.nodes.size() && a!=b,"invalid edge");
}

void surfaceConnectivity() {
    // A segment must remain connected on a measured plane, but disconnect when
    // its middle crosses an unobserved gap.
    std::vector<world_fgng::Point> continuous,gap,parallel;
    for(int x=-10;x<=10;++x) for(int y=-3;y<=3;++y) {
        const world_fgng::Point p{x*.01F,y*.01F,1};
        continuous.push_back(p);
        if (std::abs(x)>=5) gap.push_back(p);
        parallel.push_back(p); parallel.push_back({p.x,p.y,1.12F});
    }
    const bio_fgng::SurfaceSupport planeSupport(continuous,.02F),gapSupport(gap,.02F);
    require(planeSupport.supportsSegment({-.07F,0,1},{.07F,0,1}),"continuous surface disconnected");
    require(!gapSupport.supportsSegment({-.07F,0,1},{.07F,0,1}),"short edge bridged unsupported gap");
    const bio_fgng::SurfaceSupport parallelSupport(parallel,.02F);
    require(!parallelSupport.supportsSegment({0,0,1},{0,0,1.12F}),"parallel surfaces bridged");
    require(!planeSupport.supportsSegment({0,0,1.1F},{.01F,0,1.1F}),"unsupported nodes accepted");
    require(!planeSupport.supportsSegment({0,0,1},{1e30F,0,1}),"unbounded segment sampling");

    bio_fgng::MapperConfig c;
    c.graph.maxNodes=64; c.memoryCapacity=1200; c.replayBudget=1200;
    c.inputBudget=1200; c.voxelSize=.01F; c.edgeSupportRadius=.025F;
    bio_fgng::Mapper planeMap(c),twoPlanes(c);
    bio_fgng::FocusControl control; control.mode=bio_fgng::FocusMode::Manual; control.focusDistance=1;
    bio_fgng::MapperStats stats;
    for(int i=0;i<45;++i) {
        bio_fgng::DepthFrame f;
        f.intrinsics={32,24,160,160,15.5F,11.5F}; f.timestamp=i*.1;
        f.depth.assign(32*24,1);
        planeMap.update(f,control);
        for(size_t j=0;j<f.depth.size();++j) if(j%32>=16) f.depth[j]=1.4F;
        stats=twoPlanes.update(f,control);
    }
    require(!planeMap.graph().edges.empty(),"learned continuous plane lost all connectivity");
    const auto g=twoPlanes.graph();
    require(!g.edges.empty(),"disconnected surfaces lost within-surface connectivity");
    require(stats.edges+stats.rejectedEdges==stats.rawEdges &&
            stats.rejectedEdges==stats.rejectedUnsupportedEdges,
            "edge rejection accounting");
    const bio_fgng::SurfaceSupport measured(twoPlanes.memory().representatives(),c.edgeSupportRadius);
    for(const auto& [ia,ib]:g.edges) {
        const auto& a=g.nodes[ia]; const auto& b=g.nodes[ib];
        require(std::abs(a.z-b.z)<.1F,"learned edge connected separated planes");
        require(measured.supportsSegment(a,b),"learned edge has no observed support");
    }
    for(const auto& p:g.nodes)
        require(std::min(std::abs(p.z-1),std::abs(p.z-1.4F))<.02F,"neighborhood learning pulled node into surface gap");
}
}
int main() try {
    surfaceConnectivity();
    bio_fgng::MapperConfig c;
    c.graph.maxNodes=32; c.memoryCapacity=160; c.replayBudget=120; c.inputBudget=100;
    c.epochsPerFrame=2;
    bio_fgng::Mapper a(c),b(c);
    bio_fgng::FocusControl control; control.mode=bio_fgng::FocusMode::Manual; control.focusDistance=2;
    for(int i=0;i<18;++i) {
        auto f=plane(i*.04);
        // Translate sensor rigidly while expressing the same world plane.
        f.pose.m[11]=i*.01; for(auto& z:f.depth) z-=static_cast<float>(f.pose.m[11]);
        const auto s=a.update(f,control); b.update(f,control);
        require(s.insertedPoints<=c.inputBudget && s.memoryPoints<=c.memoryCapacity &&
                s.replayPoints<=c.replayBudget,"support budget");
        checkGraph(a.graph(),c.graph.maxNodes);
        for(const auto& p:a.memory().representatives()) require(std::abs(p.z-2)<1e-5,"double/world transform");
    }
    const auto ga=a.graph(),gb=b.graph();
    require(!ga.nodes.empty(),"empty learned graph");
    require(ga.ids==gb.ids && ga.edges==gb.edges,"nondeterministic graph");
    for(size_t i=0;i<ga.nodes.size();++i)
        require(ga.nodes[i].x==gb.nodes[i].x && ga.nodes[i].y==gb.nodes[i].y && ga.nodes[i].z==gb.nodes[i].z,"nondeterministic geometry");
    // A sensor dropout may stop visible attention, but must not churn stored map.
    auto missing=plane(1); std::fill(missing.depth.begin(),missing.depth.end(),0);
    const auto s=a.update(missing,control); const auto after=a.graph();
    require(s.replayPoints==0 && after.ids==ga.ids && after.edges==ga.edges,"dropout graph changed");
    for(size_t i=0;i<ga.nodes.size();++i) {
        require(after.nodes[i].x==ga.nodes[i].x && after.nodes[i].y==ga.nodes[i].y &&
                after.nodes[i].z==ga.nodes[i].z,"dropout moved node");
        require(a.focus().evaluate(after.nodes[i])==c.focus.peripheralFloor,"dropout retained visible bonus");
    }
    // Invalid frames rejected before changing support; next valid frame remains usable.
    auto invalid=plane(2); invalid.pose.m[0]=2;
    const size_t before=a.memory().size(); bool rejected=false;
    try { a.update(invalid,control); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected && a.memory().size()==before,"invalid pose corrupted support");
    a.update(plane(2),control); checkGraph(a.graph(),c.graph.maxNodes);
    c.graph.maxNodes=-1; rejected=false;
    try { bio_fgng::Mapper bad(c); } catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"negative node capacity accepted");
    c.graph.maxNodes=32; c.edgeSupportRadius=std::numeric_limits<float>::quiet_NaN(); rejected=false;
    try { bio_fgng::Mapper bad(c); } catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"nonfinite support radius accepted");
    std::cout<<"bio mapper: world geometry, budgets, IDs, dropout, surface gaps and validation passed\n";
    return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
