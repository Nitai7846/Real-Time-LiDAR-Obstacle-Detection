#include "lidar/pcl_comparison.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
Cloud::Ptr create() { return Cloud::Ptr(new Cloud); }
void point(Cloud& c, float x, float y, float z) {
    pcl::PointXYZI p;
    p.x=x;p.y=y;p.z=z;p.intensity=1.0f;
    c.push_back(p);
}
lidar::PipelineConfig config() {
    lidar::PipelineConfig c;
    c.preprocessing.voxel_size=0.05f;
    c.ransac.max_iterations=150;
    c.ransac.distance_threshold=0.08f;
    c.ransac.max_tilt_degrees=15.0f;
    c.clustering.distance_tolerance=0.35f;
    c.clustering.min_cluster_size=5;
    c.clustering.max_cluster_size=500;
    return c;
}
Cloud::Ptr scene(int obstacles) {
    auto cloud=create();
    for (int ix=-10; ix<=10; ++ix)
        for (int iy=-10; iy<=10; ++iy)
            point(*cloud,ix*.5f,iy*.5f,-1.5f);
    for (int n=0; n<obstacles; ++n)
        for (int ix=0; ix<3; ++ix)
            for (int iy=0; iy<3; ++iy)
                for (int iz=0; iz<2; ++iz)
                    point(*cloud,(n==0?-3.0f:3.0f)+ix*.12f,
                          (n==0?2.0f:-2.0f)+iy*.12f,
                          -.4f+iz*.15f);
    return cloud;
}
void finiteNonnegative(double x) { check(std::isfinite(x) && x>=0,"invalid elapsed milliseconds"); }
void between01(const std::optional<double>& x) {
    if (x) check(std::isfinite(*x) && *x>=0 && *x<=1.0,"overlap out of range");
}
void rejectIncomplete() {
    bool caught=false;
    try { lidar::PclComparator::compare(lidar::PipelineResult{}); }
    catch (const std::invalid_argument&) {caught=true;}
    check(caught,"null cloud must throw");
}
void emptyCloud() {
    auto c=config();
    auto r=lidar::Pipeline(c).process(create());
    auto m=lidar::PclComparator::compare(r,c);
    check(m.custom_ground_inliers==0 && m.pcl_ground_inliers==0,"empty plane inliers");
    check(m.custom_cluster_count==0 && m.pcl_cluster_count==0,"empty clustering");
    check(m.mean_best_cluster_jaccard && *m.mean_best_cluster_jaccard==1.0,"empty partition similarity");
}
void flatGround() {
    auto c=config(); auto r=lidar::Pipeline(c).process(scene(0));
    auto m=lidar::PclComparator::compare(r,c);
    check(m.custom_ground_inliers>0 && m.pcl_ground_inliers>0,"ground was not detected by both methods");
    finiteNonnegative(m.custom_ransac_ms); finiteNonnegative(m.pcl_ransac_ms);
    between01(m.ground_inlier_jaccard);
    check(m.plane_normal_angle_deg && *m.plane_normal_angle_deg<=90.0,"normal angle invalid");
}
void oneObstacle() {
    auto c=config(); auto r=lidar::Pipeline(c).process(scene(1));
    auto m=lidar::PclComparator::compare(r,c);
    finiteNonnegative(m.custom_clustering_ms); finiteNonnegative(m.pcl_clustering_ms);
    check(m.custom_cluster_count==1 && m.pcl_cluster_count==1,"single obstacle should yield one cluster");
    check(m.custom_clustered_points>0 && m.pcl_clustered_points>0,"missing cluster points");
    between01(m.mean_best_cluster_jaccard);
}
void twoObstacles() {
    auto c=config(); auto r=lidar::Pipeline(c).process(scene(2));
    auto m=lidar::PclComparator::compare(r,c);
    check(m.custom_cluster_count==2 && m.pcl_cluster_count==2,"two separate obstacles should yield two clusters");
    check(m.mean_best_cluster_jaccard && *m.mean_best_cluster_jaccard>0.99,"unexpected cluster disagreement");
}
void repeatedComparisons() {
    auto c=config(); auto r=lidar::Pipeline(c).process(scene(1));
    auto m1=lidar::PclComparator::compare(r,c);
    auto m2=lidar::PclComparator::compare(r,c);
    check(m1.custom_cluster_count==m2.custom_cluster_count,"unstable clustering results");
    check(m1.pcl_cluster_count==m2.pcl_cluster_count,"unstable reference clustering");
}
void doesNotModifyPipelineResults() {
    auto c=config(); auto r=lidar::Pipeline(c).process(scene(1));
    const auto orig=r.clusters;
    const auto ground=r.separated.ground->size();
    (void)lidar::PclComparator::compare(r,c);
    check(r.clusters==orig,"clusters modified");
    check(r.separated.ground->size()==ground,"ground result modified");
}
void twoPoints() {
    auto cloud=create(); point(*cloud,0,0,0);point(*cloud,1,0,0);
    auto c=config(); auto r=lidar::Pipeline(c).process(cloud);
    auto m=lidar::PclComparator::compare(r,c);
    check(m.custom_ground_inliers==0 && m.pcl_ground_inliers==0,"two points must skip plane detection");
}
}
int main() {
    const std::vector<std::pair<std::string,std::function<void()>>> tests = {
        {"incomplete result error",rejectIncomplete}, {"empty cloud",emptyCloud},
        {"flat ground",flatGround}, {"one obstacle",oneObstacle},
        {"two obstacles",twoObstacles}, {"repeated comparison",repeatedComparisons},
        {"pipeline result unchanged",doesNotModifyPipelineResults},
        {"two points",twoPoints}
    };
    std::size_t passed=0;
    for(const auto& t:tests){
        try { t.second(); std::cout<<"[PASS] "<<t.first<<'\n'; ++passed; }
        catch(const std::exception& e){ std::cerr<<"[FAIL] "<<t.first<<": "<<e.what()<<'\n'; }
    }
    std::cout<<passed<<"/"<<tests.size()<<" PCL comparison tests passed\n";
    return passed==tests.size()?0:1;
}
