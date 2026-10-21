#include "lidar/metrics_logger.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace fs = std::filesystem;
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
struct TempDir {
    fs::path path;
    TempDir() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("m11_csv_" + std::to_string(stamp));
        fs::create_directories(path);
    }
    ~TempDir() { std::error_code ec; fs::remove_all(path,ec); }
};
std::vector<std::string> lines(const fs::path& path) {
    std::ifstream in(path);
    std::vector<std::string> values;
    for (std::string line; std::getline(in,line);) values.push_back(line);
    return values;
}
std::size_t fieldCount(const std::string& line) {
    bool quoted = false;
    std::size_t count=1;
    for (std::size_t i=0; i<line.size(); ++i) {
        if (line[i]=='"') {
            if (quoted && i+1<line.size() && line[i+1]=='"') ++i;
            else quoted = !quoted;
        } else if (line[i]==',' && !quoted) ++count;
    }
    return count;
}
void writesRowsAndHeader() {
    TempDir tmp;
    const auto dest = tmp.path / "report.csv";
    {
        lidar::MetricsLogger logger(dest);
        lidar::FrameMetrics row;
        row.frame_index=2; row.filename="scan.bin";
        row.raw_points=115; row.filtered_points=20; row.timings.total_ms=4.5;
        logger.write(row); logger.write(row);
    }
    const auto v=lines(dest);
    check(v.size()==3,"expected header and two rows");
    check(v[0].find("pipeline_total_ms")!=std::string::npos,"header invalid");
    check(v[1].find("4.500000")!=std::string::npos,"timing missing");
}
void correctColumnsWhenNoComparison() {
    TempDir tmp;
    const auto dest=tmp.path/"empty_cmp.csv";
    { lidar::MetricsLogger log(dest); lidar::FrameMetrics row; log.write(row); }
    const auto v=lines(dest);
    check(fieldCount(v[0]) == 30,"expected exactly 30 headers");
    check(fieldCount(v[1]) == fieldCount(v[0]),"missing comparison columns broke schema");
}
void correctColumnsWithComparison() {
    TempDir tmp;
    const auto dest=tmp.path/"cmp.csv";
    {
        lidar::MetricsLogger log(dest);
        lidar::FrameMetrics row;
        lidar::PclComparison cmp;
        cmp.pcl_ransac_ms=11.0;
        cmp.ground_inlier_jaccard=0.75;
        cmp.mean_best_cluster_jaccard=0.9;
        row.comparison=cmp;
        log.write(row);
    }
    const auto v=lines(dest);
    check(fieldCount(v[0])==fieldCount(v[1]),"comparison columns broke schema");
    check(v[1].find("0.750000")!=std::string::npos,"jaccard not serialized");
}
void escapedFilename() {
    TempDir tmp;
    const auto dest=tmp.path/"quotes.csv";
    { lidar::MetricsLogger log(dest); lidar::FrameMetrics row;
      row.filename="weird,\"frame\".bin"; log.write(row); }
    check(lines(dest)[1].find("\"weird,\"\"frame\"\".bin\"")!=std::string::npos,
          "CSV quoted name wrong");
}
void refusesOverwrite() {
    TempDir tmp;
    const auto dest=tmp.path/"exists.csv";
    { std::ofstream out(dest); out<<"DO NOT ERASE"; }
    bool caught=false;
    try { lidar::MetricsLogger logger(dest); }
    catch (const std::runtime_error&) { caught=true; }
    check(caught,"existing file was accepted");
    check(lines(dest).at(0)=="DO NOT ERASE","existing file was altered");
}
void invalidDestination() {
    bool caught=false;
    try { lidar::MetricsLogger logger(fs::path{}); }
    catch (const std::invalid_argument&) { caught=true; }
    check(caught,"empty destination should fail");
}
}

int main() {
    const std::vector<std::pair<std::string,std::function<void()>>> tests={
        {"CSV header and rows",writesRowsAndHeader},
        {"schema without comparison",correctColumnsWhenNoComparison},
        {"schema with comparison",correctColumnsWithComparison},
        {"quoted filename",escapedFilename},
        {"overwrite protection",refusesOverwrite},
        {"invalid destination",invalidDestination}
    };
    std::size_t pass=0;
    for (const auto& t:tests) {
        try { t.second(); std::cout<<"[PASS] "<<t.first<<'\n'; ++pass; }
        catch (const std::exception& e) { std::cerr<<"[FAIL] "<<t.first<<": "<<e.what()<<'\n'; }
    }
    std::cout<<pass<<"/"<<tests.size()<<" MetricsLogger tests passed\n";
    return pass==tests.size()?0:1;
}
