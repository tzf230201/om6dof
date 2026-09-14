#include "comparison_mapper.hpp"

#include <iostream>
#include <set>

namespace {
void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bio_fgng::DepthFrame plane(double time=0) {
    bio_fgng::DepthFrame frame;
    frame.intrinsics={24,18,50,50,11.5F,8.5F};
    frame.timestamp=time;
    frame.depth.assign(24*18,2.0F);
    // Proper +90-degree Y rotation plus translation. Camera z=2 becomes
    // world x=3.25. This catches learning in camera coordinates or double TF.
    frame.pose.m={0,0,1,1.25, 0,1,0,-.4, -1,0,0,.7, 0,0,0,1};
    return frame;
}

std::vector<world_fgng::Point> observedPlane() {
    std::vector<world_fgng::Point> points;
    for (int v=0;v<18;++v) for (int u=0;u<24;++u) {
        const float cameraX=(float(u)-11.5F)*2.0F/50.0F;
        const float cameraY=(float(v)-8.5F)*2.0F/50.0F;
        points.push_back({3.25F,static_cast<float>(double(cameraY)-.4),
                         static_cast<float>(.7-double(cameraX))});
    }
    return points;
}

void equal(const world_fgng::Graph& a,const world_fgng::Graph& b,const char* message) {
    require(a.nodes.size()==b.nodes.size() && a.ids==b.ids && a.edges==b.edges,message);
    for (size_t i=0;i<a.nodes.size();++i)
        require(a.nodes[i].x==b.nodes[i].x && a.nodes[i].y==b.nodes[i].y &&
                a.nodes[i].z==b.nodes[i].z,message);
}

void check(const world_fgng::Graph& graph,size_t cap) {
    require(!graph.nodes.empty() && graph.nodes.size()<=cap,"node capacity violated or empty graph");
    require(graph.ids.size()==graph.nodes.size(),"missing graph IDs");
    require(std::set<int64_t>(graph.ids.begin(),graph.ids.end()).size()==graph.ids.size(),
            "duplicate snapshot IDs");
    for (const auto& point:graph.nodes) require(world_fgng::finite(point),"nonfinite learned point");
    for (const auto& [a,b]:graph.edges)
        require(a<graph.nodes.size() && b<graph.nodes.size() && a!=b,"invalid graph edge");
}

comparison::Config config(const std::string& algorithm) {
    comparison::Config result;
    result.algorithm=algorithm;
    result.memoryMode="current";
    result.mapping.graph.maxNodes=32;
    result.mapping.graph.initialRegions=3;
    result.mapping.memoryCapacity=1200;
    result.mapping.inputBudget=1000;
    result.mapping.replayBudget=1000;
    result.mapping.edgeSupportRadius=0;
    result.mapping.epochsPerFrame=2;
    result.dblEdgeCutPeriodFrames=10000;
    return result;
}

world_fgng::Graph copy(const DynamicGrowingNeuralGas& native) {
    std::vector<GngPoint3f> points;
    std::vector<uint32_t> ids;
    world_fgng::Graph graph;
    native.copyGraph(points,ids,graph.edges);
    for (const auto& point:points) graph.nodes.push_back({point.x,point.y,point.z});
    graph.ids.assign(ids.begin(),ids.end());
    return graph;
}

world_fgng::Graph copy(const dblgng::DistributedBatchLearningGNG& native) {
    std::vector<dblgng::Point3f> points;
    world_fgng::Graph graph;
    native.copyGraph(points,graph.edges);
    for (const auto& point:points) graph.nodes.push_back({point.x,point.y,point.z});
    graph.ids.resize(points.size());
    std::iota(graph.ids.begin(),graph.ids.end(),int64_t{0});
    return graph;
}

void originalCoreAgreement(const std::string& algorithm) {
    auto cfg=config(algorithm);
    comparison::Mapper wrapper(cfg);
    DynamicGrowingNeuralGas dd(cfg.mapping.graph.maxNodes);
    // Deliberately configure the original DBL core without using dblConfig().
    dblgng::Config dc;
    dc.maxNodes=32;
    dc.initialRegions=3;
    dblgng::DistributedBatchLearningGNG dbl(dc);
    std::vector<GngPoint3f> ddPoints;
    std::vector<dblgng::Point3f> dblPoints;
    for (const auto& point:observedPlane()) {
        ddPoints.push_back({point.x,point.y,point.z});
        dblPoints.push_back({point.x,point.y,point.z});
    }
    bio_fgng::FocusControl control;
    for (int frame=0;frame<20;++frame) {
        const auto stats=wrapper.update(plane(frame*.05),control);
        for (int epoch=0;epoch<cfg.mapping.epochsPerFrame;++epoch) {
            if (algorithm=="ddgng") dd.partialFit(ddPoints,500);
            else dbl.learnBatch(dblPoints);
        }
        const auto expected=algorithm=="ddgng" ? copy(dd) : copy(dbl);
        equal(wrapper.graph(),expected,"baseline diverged from unchanged native core");
        equal(wrapper.graph(),wrapper.rawGraph(),"disabled edge filters changed graph");
        require(!wrapper.attentionEnabled(),"baseline enabled attention");
        require(stats.validPoints==432 && stats.inputPixels==432 && stats.memoryPoints==0 &&
                stats.replayPoints==432,"current point accounting mismatch");
        check(wrapper.graph(),32);
        for (const auto& point:wrapper.graph().nodes)
            require(std::abs(point.x-3.25F)<1e-6F,"rotation or translation not applied exactly once");
    }
    const auto before=wrapper.graph();
    auto missing=plane(2);
    std::fill(missing.depth.begin(),missing.depth.end(),0);
    const auto stats=wrapper.update(missing,control);
    equal(wrapper.graph(),before,"empty depth changed baseline graph");
    require(stats.validPoints==0 && stats.replayPoints==0,"empty depth learned a batch");
    // Reset semantics use reconstruction: the original DBL reset() does not
    // reseed its RNG, so the facade owner must not call it as a fresh-session reset.
    comparison::Mapper reset(cfg),fresh(cfg);
    reset.update(plane(),control);
    fresh.update(plane(),control);
    equal(reset.graph(),fresh.graph(),"fresh-session reconstruction nondeterministic");
}

void baselineAttentionIndependence(const std::string& algorithm) {
    auto cfg=config(algorithm);
    cfg.memoryMode="world";
    comparison::Mapper a(cfg),b(cfg);
    bio_fgng::FocusControl left,right;
    left.mode=right.mode=bio_fgng::FocusMode::Manual;
    left.gazeU=.05F; left.gazeV=.1F; left.focusDistance=.5F;
    right.gazeU=.95F; right.gazeV=.9F; right.focusDistance=4;
    for (int i=0;i<10;++i) {
        a.update(plane(i*.1),left);
        b.update(plane(i*.1),right);
        equal(a.graph(),b.graph(),"focus controls influenced baseline learning");
    }
}

void memoryAndFiltering(const std::string& algorithm) {
    auto cfg=config(algorithm);
    cfg.memoryMode="world";
    cfg.mapping.inputBudget=120;
    cfg.mapping.replayBudget=70;
    cfg.mapping.memoryCapacity=200;
    cfg.mapping.voxelSize=.01F;
    comparison::Mapper world(cfg);
    bio_fgng::FocusControl control;
    const auto first=world.update(plane(),control);
    require(first.memoryPoints==120 && first.replayPoints==70 && first.validPoints==432,
            "world support/replay budgets incorrect");
    auto second=plane(.1);
    second.pose.m[3]+=2; // Disjoint view with no overlap in world coordinates.
    const auto next=world.update(second,control);
    require(next.memoryPoints==200 && next.evictions>0,"history not retained or memory cap ignored");
    // Verify history contains observations from both views by independently
    // feeding the same bounded support to the same deterministic voxel memory.
    world_fgng::VoxelMemory expected(.01F,200,7);
    const auto observation=observedPlane();
    for (int view=0;view<2;++view) for (size_t i=0;i<120;++i) {
        auto p=observation[i*observation.size()/120];
        p.x+=view*2;
        expected.insert(p);
    }
    bool oldSeen=false,newSeen=false;
    for (const auto& p:expected.representatives()) {
        oldSeen|=p.x<4;
        newSeen|=p.x>4;
    }
    require(oldSeen && newSeen,"memory fixture did not exercise retained history");
    // Independent direct-core replay verifies that retained observations really
    // reach the learner, rather than merely increasing a memory counter.
    world_fgng::VoxelMemory directMemory(.01F,200,7);
    DynamicGrowingNeuralGas dd(32);
    dblgng::Config dc; dc.maxNodes=32; dc.initialRegions=3;
    dblgng::DistributedBatchLearningGNG dbl(dc);
    uint64_t replayEpoch=0;
    for (int view=0;view<2;++view) {
        for (size_t i=0;i<120;++i) {
            auto p=observation[i*observation.size()/120];
            p.x+=view*2;
            directMemory.insert(p);
        }
        for (int epoch=0;epoch<2;++epoch) {
            const auto batch=directMemory.replay(70,replayEpoch++);
            std::vector<GngPoint3f> d;
            std::vector<dblgng::Point3f> b;
            for (const auto& p:batch) { d.push_back({p.x,p.y,p.z}); b.push_back({p.x,p.y,p.z}); }
            if (algorithm=="ddgng") dd.partialFit(d,500); else dbl.learnBatch(b);
        }
    }
    equal(world.graph(),algorithm=="ddgng" ? copy(dd) : copy(dbl),
          "world memory did not replay retained world observations into native core");
    const auto before=world.graph();
    auto invalid=plane(.2);
    invalid.pose.m[0]=2;
    bool rejected=false;
    try { world.update(invalid,control); } catch (const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid pose accepted");
    equal(world.graph(),before,"invalid pose mutated graph");
    auto missing=plane(.3);
    std::fill(missing.depth.begin(),missing.depth.end(),0);
    const auto dropout=world.update(missing,control);
    equal(world.graph(),before,"dropout learned world history without observations");
    require(dropout.memoryPoints==200 && dropout.replayPoints==0,"dropout discarded observation history");

    auto rawCfg=config(algorithm),filteredCfg=rawCfg;
    filteredCfg.mapping.edgeSupportRadius=.00001F;
    comparison::Mapper supportFiltered(filteredCfg),supportRaw(rawCfg);
    for (int i=0;i<4;++i) {
        supportRaw.update(plane(i*.1),control);
        const auto stats=supportFiltered.update(plane(i*.1),control);
        equal(supportRaw.graph(),supportFiltered.rawGraph(),"support filtering changed native core");
        require(stats.rejectedUnsupportedEdges>0 &&
                stats.edges+stats.rejectedUnsupportedEdges==stats.rawEdges,
                "observation support filtering not honored or counted");
    }
}

void fgngDelegation() {
    comparison::Config cfg;
    cfg.mapping.graph.maxNodes=32;
    comparison::Mapper wrapper(cfg);
    bio_fgng::Mapper native(cfg.mapping);
    bio_fgng::FocusControl control;
    for (int i=0;i<8;++i) {
        wrapper.update(plane(i*.1),control);
        native.update(plane(i*.1),control);
        equal(wrapper.graph(),native.graph(),"existing F-GNG behavior changed by facade");
    }
    require(wrapper.attentionEnabled(),"F-GNG attention disabled");
}

void dblEdgeCutting() {
    dblgng::Config cfg;
    cfg.maxNodes=64;
    cfg.initialRegions=4;
    dblgng::DistributedBatchLearningGNG graph(cfg);
    std::vector<dblgng::Point3f> points;
    std::mt19937 random(3);
    std::normal_distribution<float> noise(0.0F,0.08F);
    for (int cluster=0;cluster<4;++cluster)
        for (int i=0;i<300;++i)
            points.push_back({cluster*1.5F+noise(random),noise(random),1.0F+noise(random)});
    for (int epoch=0;epoch<12;++epoch) graph.learnBatch(points);
    const auto before=copy(graph);
    graph.cutWeakEdges();
    const auto after=copy(graph);
    require(!after.edges.empty() && after.edges.size()<before.edges.size(),
            "DBL-GNG percentile edge cutting did not remove weak connections");
    check(after,cfg.maxNodes);
}

void invalidConfiguration() {
    const auto expect=[](const comparison::Config& cfg) {
        bool rejected=false;
        try { comparison::Mapper mapper(cfg); }
        catch (const std::invalid_argument&) { rejected=true; }
        require(rejected,"invalid configuration accepted before core allocation");
    };
    for (const std::string algorithm:{"fgng","ddgng","dblgng"}) {
        comparison::Config good; good.algorithm=algorithm;
        auto bad=good; bad.mapping.graph.maxNodes=-1; expect(bad);
        bad=good; bad.mapping.graph.maxNodes=1000000000; expect(bad);
        bad=good; bad.mapping.replayBudget=0; expect(bad);
        bad=good; bad.mapping.inputBudget=4000001; expect(bad);
        bad=good; bad.mapping.memoryCapacity=2; expect(bad);
        bad=good; bad.mapping.epochsPerFrame=0; expect(bad);
        bad=good; bad.mapping.voxelSize=0; expect(bad);
        bad=good; bad.mapping.graph.winnerRate=std::numeric_limits<float>::quiet_NaN(); expect(bad);
        bad=good; bad.mapping.edgeSupportRadius=-1; expect(bad);
        bad=good; bad.mapping.focus.minDepth=0; expect(bad);
        bad=good; bad.ddUpdates=0; expect(bad);
        bad=good; bad.ddUpdates=100001; expect(bad);
        bad=good; bad.dblEdgeCutPeriodFrames=0; expect(bad);
        bad=good; bad.memoryMode="unknown"; expect(bad);
    }
    comparison::Config bad; bad.algorithm="unknown"; expect(bad);
    bad=comparison::Config{}; bad.memoryMode="current"; expect(bad);
}
} // namespace

int main() try {
    invalidConfiguration();
    fgngDelegation();
    dblEdgeCutting();
    for (const std::string algorithm:{"ddgng","dblgng"}) {
        originalCoreAgreement(algorithm);
        baselineAttentionIndependence(algorithm);
        memoryAndFiltering(algorithm);
    }
    std::cout<<"comparison mapper: unchanged native agreement, world XYZ, budgets, history, "
               "dropout, deterministic reset, no baseline attention, surface filtering and validation passed\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
}
