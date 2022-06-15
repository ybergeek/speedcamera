#include "ocvrs_common.hpp"
#include <opencv2/flann.hpp>
#include "flann_types.hpp"

extern "C" {
	void cvflann_flann_distance_type(Result<cvflann::flann_distance_t>* ocvrs_return) {
		try {
			cvflann::flann_distance_t ret = cvflann::flann_distance_type();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cvflann::flann_distance_t>))
	}
	
	void cvflann_set_distance_type_flann_distance_t_int(cvflann::flann_distance_t distance_type, int order, Result_void* ocvrs_return) {
		try {
			cvflann::set_distance_type(distance_type, order);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::flann::IndexParams* cv_AutotunedIndexParams_to_IndexParams(cv::flann::AutotunedIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_AutotunedIndexParams_delete(cv::flann::AutotunedIndexParams* instance) {
		delete instance;
	}
	void cv_flann_AutotunedIndexParams_AutotunedIndexParams_float_float_float_float(float target_precision, float build_weight, float memory_weight, float sample_fraction, Result<cv::flann::AutotunedIndexParams*>* ocvrs_return) {
		try {
			cv::flann::AutotunedIndexParams* ret = new cv::flann::AutotunedIndexParams(target_precision, build_weight, memory_weight, sample_fraction);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::AutotunedIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_CompositeIndexParams_to_IndexParams(cv::flann::CompositeIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_CompositeIndexParams_delete(cv::flann::CompositeIndexParams* instance) {
		delete instance;
	}
	void cv_flann_CompositeIndexParams_CompositeIndexParams_int_int_int_flann_centers_init_t_float(int trees, int branching, int iterations, cvflann::flann_centers_init_t centers_init, float cb_index, Result<cv::flann::CompositeIndexParams*>* ocvrs_return) {
		try {
			cv::flann::CompositeIndexParams* ret = new cv::flann::CompositeIndexParams(trees, branching, iterations, centers_init, cb_index);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::CompositeIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_HierarchicalClusteringIndexParams_to_IndexParams(cv::flann::HierarchicalClusteringIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_HierarchicalClusteringIndexParams_delete(cv::flann::HierarchicalClusteringIndexParams* instance) {
		delete instance;
	}
	void cv_flann_HierarchicalClusteringIndexParams_HierarchicalClusteringIndexParams_int_flann_centers_init_t_int_int(int branching, cvflann::flann_centers_init_t centers_init, int trees, int leaf_size, Result<cv::flann::HierarchicalClusteringIndexParams*>* ocvrs_return) {
		try {
			cv::flann::HierarchicalClusteringIndexParams* ret = new cv::flann::HierarchicalClusteringIndexParams(branching, centers_init, trees, leaf_size);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::HierarchicalClusteringIndexParams*>))
	}
	
	void cv_Index_delete(cv::flann::Index* instance) {
		delete instance;
	}
	void cv_flann_Index_Index(Result<cv::flann::Index*>* ocvrs_return) {
		try {
			cv::flann::Index* ret = new cv::flann::Index();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::Index*>))
	}
	
	void cv_flann_Index_Index_const__InputArrayR_const_IndexParamsR_flann_distance_t(const cv::_InputArray* features, const cv::flann::IndexParams* params, cvflann::flann_distance_t distType, Result<cv::flann::Index*>* ocvrs_return) {
		try {
			cv::flann::Index* ret = new cv::flann::Index(*features, *params, distType);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::Index*>))
	}
	
	void cv_flann_Index_build_const__InputArrayR_const_IndexParamsR_flann_distance_t(cv::flann::Index* instance, const cv::_InputArray* features, const cv::flann::IndexParams* params, cvflann::flann_distance_t distType, Result_void* ocvrs_return) {
		try {
			instance->build(*features, *params, distType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_Index_knnSearch_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_int_const_SearchParamsR(cv::flann::Index* instance, const cv::_InputArray* query, const cv::_OutputArray* indices, const cv::_OutputArray* dists, int knn, const cv::flann::SearchParams* params, Result_void* ocvrs_return) {
		try {
			instance->knnSearch(*query, *indices, *dists, knn, *params);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_Index_radiusSearch_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_double_int_const_SearchParamsR(cv::flann::Index* instance, const cv::_InputArray* query, const cv::_OutputArray* indices, const cv::_OutputArray* dists, double radius, int maxResults, const cv::flann::SearchParams* params, Result<int>* ocvrs_return) {
		try {
			int ret = instance->radiusSearch(*query, *indices, *dists, radius, maxResults, *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_flann_Index_save_const_const_StringR(const cv::flann::Index* instance, const char* filename, Result_void* ocvrs_return) {
		try {
			instance->save(std::string(filename));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_Index_load_const__InputArrayR_const_StringR(cv::flann::Index* instance, const cv::_InputArray* features, const char* filename, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->load(*features, std::string(filename));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_flann_Index_release(cv::flann::Index* instance, Result_void* ocvrs_return) {
		try {
			instance->release();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_Index_getDistance_const(const cv::flann::Index* instance, Result<cvflann::flann_distance_t>* ocvrs_return) {
		try {
			cvflann::flann_distance_t ret = instance->getDistance();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cvflann::flann_distance_t>))
	}
	
	void cv_flann_Index_getAlgorithm_const(const cv::flann::Index* instance, Result<cvflann::flann_algorithm_t>* ocvrs_return) {
		try {
			cvflann::flann_algorithm_t ret = instance->getAlgorithm();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cvflann::flann_algorithm_t>))
	}
	
	void* cv_flann_IndexParams_getPropParams(cv::flann::IndexParams* instance) {
			void* ret = instance->params;
			return ret;
	}
	
	void cv_flann_IndexParams_setPropParams_voidX(cv::flann::IndexParams* instance, void* val) {
			instance->params = val;
	}
	
	void cv_IndexParams_delete(cv::flann::IndexParams* instance) {
		delete instance;
	}
	void cv_flann_IndexParams_IndexParams(Result<cv::flann::IndexParams*>* ocvrs_return) {
		try {
			cv::flann::IndexParams* ret = new cv::flann::IndexParams();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::IndexParams*>))
	}
	
	void cv_flann_IndexParams_getString_const_const_StringR_const_StringR(const cv::flann::IndexParams* instance, const char* key, const char* defaultVal, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = instance->getString(std::string(key), std::string(defaultVal));
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_flann_IndexParams_getInt_const_const_StringR_int(const cv::flann::IndexParams* instance, const char* key, int defaultVal, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getInt(std::string(key), defaultVal);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_flann_IndexParams_getDouble_const_const_StringR_double(const cv::flann::IndexParams* instance, const char* key, double defaultVal, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getDouble(std::string(key), defaultVal);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_flann_IndexParams_setString_const_StringR_const_StringR(cv::flann::IndexParams* instance, const char* key, const char* value, Result_void* ocvrs_return) {
		try {
			instance->setString(std::string(key), std::string(value));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_setInt_const_StringR_int(cv::flann::IndexParams* instance, const char* key, int value, Result_void* ocvrs_return) {
		try {
			instance->setInt(std::string(key), value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_setDouble_const_StringR_double(cv::flann::IndexParams* instance, const char* key, double value, Result_void* ocvrs_return) {
		try {
			instance->setDouble(std::string(key), value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_setFloat_const_StringR_float(cv::flann::IndexParams* instance, const char* key, float value, Result_void* ocvrs_return) {
		try {
			instance->setFloat(std::string(key), value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_setBool_const_StringR_bool(cv::flann::IndexParams* instance, const char* key, bool value, Result_void* ocvrs_return) {
		try {
			instance->setBool(std::string(key), value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_setAlgorithm_int(cv::flann::IndexParams* instance, int value, Result_void* ocvrs_return) {
		try {
			instance->setAlgorithm(value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_flann_IndexParams_getAll_const_vector_String_R_vector_FlannIndexType_R_vector_String_R_vector_double_R(const cv::flann::IndexParams* instance, std::vector<cv::String>* names, std::vector<cv::flann::FlannIndexType>* types, std::vector<cv::String>* strValues, std::vector<double>* numValues, Result_void* ocvrs_return) {
		try {
			instance->getAll(*names, *types, *strValues, *numValues);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::flann::IndexParams* cv_KDTreeIndexParams_to_IndexParams(cv::flann::KDTreeIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_KDTreeIndexParams_delete(cv::flann::KDTreeIndexParams* instance) {
		delete instance;
	}
	void cv_flann_KDTreeIndexParams_KDTreeIndexParams_int(int trees, Result<cv::flann::KDTreeIndexParams*>* ocvrs_return) {
		try {
			cv::flann::KDTreeIndexParams* ret = new cv::flann::KDTreeIndexParams(trees);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::KDTreeIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_KMeansIndexParams_to_IndexParams(cv::flann::KMeansIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_KMeansIndexParams_delete(cv::flann::KMeansIndexParams* instance) {
		delete instance;
	}
	void cv_flann_KMeansIndexParams_KMeansIndexParams_int_int_flann_centers_init_t_float(int branching, int iterations, cvflann::flann_centers_init_t centers_init, float cb_index, Result<cv::flann::KMeansIndexParams*>* ocvrs_return) {
		try {
			cv::flann::KMeansIndexParams* ret = new cv::flann::KMeansIndexParams(branching, iterations, centers_init, cb_index);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::KMeansIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_LinearIndexParams_to_IndexParams(cv::flann::LinearIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_LinearIndexParams_delete(cv::flann::LinearIndexParams* instance) {
		delete instance;
	}
	void cv_flann_LinearIndexParams_LinearIndexParams(Result<cv::flann::LinearIndexParams*>* ocvrs_return) {
		try {
			cv::flann::LinearIndexParams* ret = new cv::flann::LinearIndexParams();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::LinearIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_LshIndexParams_to_IndexParams(cv::flann::LshIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_LshIndexParams_delete(cv::flann::LshIndexParams* instance) {
		delete instance;
	}
	void cv_flann_LshIndexParams_LshIndexParams_int_int_int(int table_number, int key_size, int multi_probe_level, Result<cv::flann::LshIndexParams*>* ocvrs_return) {
		try {
			cv::flann::LshIndexParams* ret = new cv::flann::LshIndexParams(table_number, key_size, multi_probe_level);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::LshIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_SavedIndexParams_to_IndexParams(cv::flann::SavedIndexParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_SavedIndexParams_delete(cv::flann::SavedIndexParams* instance) {
		delete instance;
	}
	void cv_flann_SavedIndexParams_SavedIndexParams_const_StringR(const char* filename, Result<cv::flann::SavedIndexParams*>* ocvrs_return) {
		try {
			cv::flann::SavedIndexParams* ret = new cv::flann::SavedIndexParams(std::string(filename));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::SavedIndexParams*>))
	}
	
	cv::flann::IndexParams* cv_SearchParams_to_IndexParams(cv::flann::SearchParams* instance) {
		return dynamic_cast<cv::flann::IndexParams*>(instance);
	}
	
	void cv_SearchParams_delete(cv::flann::SearchParams* instance) {
		delete instance;
	}
	void cv_flann_SearchParams_SearchParams_int_float_bool_bool(int checks, float eps, bool sorted, bool explore_all_trees, Result<cv::flann::SearchParams*>* ocvrs_return) {
		try {
			cv::flann::SearchParams* ret = new cv::flann::SearchParams(checks, eps, sorted, explore_all_trees);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::SearchParams*>))
	}
	
	void cv_flann_SearchParams_SearchParams_int_float_bool(int checks, float eps, bool sorted, Result<cv::flann::SearchParams*>* ocvrs_return) {
		try {
			cv::flann::SearchParams* ret = new cv::flann::SearchParams(checks, eps, sorted);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::flann::SearchParams*>))
	}
	
}
