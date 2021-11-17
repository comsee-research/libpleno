#pragma once

#include <opencv2/imgproc.hpp>

#include "types.h"

#include "io/archive.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/mia.h"

#include "pointcloud.h"

class PointCloud;

struct DepthInfo {
	static constexpr double NO_DEPTH = 0.;
	static constexpr double NO_CONFIDENCE = -1.;
	
	enum State : std::uint16_t {
		UNINITIALIZED = 0, INITIALIZED, COMPUTED
	};	
	
	double depth = NO_DEPTH;
	double confidence = NO_CONFIDENCE;
	State state = UNINITIALIZED;	
};

void save(v::OutputArchive& archive, const DepthInfo& di);
void load(v::InputArchive& archive, DepthInfo& di);

//******************************************************************************
//******************************************************************************
//******************************************************************************
class DepthMap {
public:	
	using DepthMapContainer = Eigen::Matrix<DepthInfo, Eigen::Dynamic /* row */, Eigen::Dynamic /* col */>;
	
	enum DepthType : bool { VIRTUAL = true, METRIC = false };
	enum MapType : bool { COARSE = true, REFINED = false };
	
private:	
	DepthMapContainer map;
	
	double min_depth_;
	double max_depth_;
	
	DepthType depth_type;
	MapType map_type;
		
public:
//******************************************************************************
	DepthMap(
		std::size_t width = 0u, std::size_t height = 0u, 
		double mind = 2., double maxd = 15., 
		DepthType dtype = VIRTUAL, MapType mtype = COARSE
	);
	
	DepthMap(const DepthMap& o);
	DepthMap(DepthMap&& o);
	
	DepthMap(const PointCloud& pc, const PlenopticCamera& mfpc);

//******************************************************************************	
	std::size_t width() const;
	std::size_t height() const;

	double depth(std::size_t k, std::size_t l) const;
	double& depth(std::size_t k, std::size_t l);
	
	double confidence(std::size_t k, std::size_t l) const;
	double& confidence(std::size_t k, std::size_t l);
	
	DepthInfo::State state(std::size_t k, std::size_t l) const;
	DepthInfo::State& state(std::size_t k, std::size_t l);
	
	DepthInfo depth_with_info(std::size_t k, std::size_t l) const;
	DepthInfo& depth_with_info(std::size_t k, std::size_t l);
	
	double min_depth() const;
	void min_depth(double mind);
	
	double max_depth() const;
	void max_depth(double maxd);
	
	void copy_from(const DepthMap& o);
	void copy_to(DepthMap& o) const;
	
//******************************************************************************	
	bool is_virtual_depth() const;
	bool is_metric_depth() const;
	
	bool is_valid_depth(double d) const;
	
//******************************************************************************
	bool is_coarse_map() const;
	bool is_refined_map() const;
	
//******************************************************************************	
	DepthMap to_metric(const PlenopticCamera& pcm) const;
	DepthMap to_virtual(const PlenopticCamera& pcm) const;

protected:
//******************************************************************************
	bool is_depth_out_of_bounds(double d) const;
	bool is_disparity_estimation_possible(double d) const;	
	
//******************************************************************************
	friend void save(v::OutputArchive& archive, const DepthMap& dm);
	friend void load(v::InputArchive& archive, DepthMap& dm);		
};

void save(v::OutputArchive& archive, const DepthMap& dm);
void load(v::InputArchive& archive, DepthMap& dm);
