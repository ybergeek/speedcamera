#include "ocvrs_common.hpp"
#include <opencv2/stitching.hpp>
#include "stitching_types.hpp"

extern "C" {
	void cv_detail_autoDetectWaveCorrectKind_const_vector_Mat_R(const std::vector<cv::Mat>* rmats, Result<cv::detail::WaveCorrectKind>* ocvrs_return) {
		try {
			cv::detail::WaveCorrectKind ret = cv::detail::autoDetectWaveCorrectKind(*rmats);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::WaveCorrectKind>))
	}
	
	void cv_detail_computeImageFeatures_const_Ptr_Feature2D_R_const__InputArrayR_ImageFeaturesR_const__InputArrayR(const cv::Ptr<cv::Feature2D>* featuresFinder, const cv::_InputArray* image, cv::detail::ImageFeatures* features, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			cv::detail::computeImageFeatures(*featuresFinder, *image, *features, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_computeImageFeatures_const_Ptr_Feature2D_R_const__InputArrayR_vector_ImageFeatures_R_const__InputArrayR(const cv::Ptr<cv::Feature2D>* featuresFinder, const cv::_InputArray* images, std::vector<cv::detail::ImageFeatures>* features, const cv::_InputArray* masks, Result_void* ocvrs_return) {
		try {
			cv::detail::computeImageFeatures(*featuresFinder, *images, *features, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_createLaplacePyrGpu_const__InputArrayR_int_vector_UMat_R(const cv::_InputArray* img, int num_levels, std::vector<cv::UMat>* pyr, Result_void* ocvrs_return) {
		try {
			cv::detail::createLaplacePyrGpu(*img, num_levels, *pyr);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_createLaplacePyr_const__InputArrayR_int_vector_UMat_R(const cv::_InputArray* img, int num_levels, std::vector<cv::UMat>* pyr, Result_void* ocvrs_return) {
		try {
			cv::detail::createLaplacePyr(*img, num_levels, *pyr);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_createWeightMap_const__InputArrayR_float_const__InputOutputArrayR(const cv::_InputArray* mask, float sharpness, const cv::_InputOutputArray* weight, Result_void* ocvrs_return) {
		try {
			cv::detail::createWeightMap(*mask, sharpness, *weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_findMaxSpanningTree_int_const_vector_MatchesInfo_R_GraphR_vector_int_R(int num_images, const std::vector<cv::detail::MatchesInfo>* pairwise_matches, cv::detail::Graph* span_tree, std::vector<int>* centers, Result_void* ocvrs_return) {
		try {
			cv::detail::findMaxSpanningTree(num_images, *pairwise_matches, *span_tree, *centers);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_leaveBiggestComponent_vector_ImageFeatures_R_vector_MatchesInfo_R_float(std::vector<cv::detail::ImageFeatures>* features, std::vector<cv::detail::MatchesInfo>* pairwise_matches, float conf_threshold, Result<std::vector<int>*>* ocvrs_return) {
		try {
			std::vector<int> ret = cv::detail::leaveBiggestComponent(*features, *pairwise_matches, conf_threshold);
			Ok(new std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_detail_matchesGraphAsString_vector_String_R_vector_MatchesInfo_R_float(std::vector<cv::String>* pathes, std::vector<cv::detail::MatchesInfo>* pairwise_matches, float conf_threshold, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = cv::detail::matchesGraphAsString(*pathes, *pairwise_matches, conf_threshold);
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_detail_normalizeUsingWeightMap_const__InputArrayR_const__InputOutputArrayR(const cv::_InputArray* weight, const cv::_InputOutputArray* src, Result_void* ocvrs_return) {
		try {
			cv::detail::normalizeUsingWeightMap(*weight, *src);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_overlapRoi_Point_Point_Size_Size_RectR(cv::Point* tl1, cv::Point* tl2, cv::Size* sz1, cv::Size* sz2, cv::Rect* roi, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::detail::overlapRoi(*tl1, *tl2, *sz1, *sz2, *roi);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_detail_restoreImageFromLaplacePyrGpu_vector_UMat_R(std::vector<cv::UMat>* pyr, Result_void* ocvrs_return) {
		try {
			cv::detail::restoreImageFromLaplacePyrGpu(*pyr);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_restoreImageFromLaplacePyr_vector_UMat_R(std::vector<cv::UMat>* pyr, Result_void* ocvrs_return) {
		try {
			cv::detail::restoreImageFromLaplacePyr(*pyr);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_resultRoiIntersection_const_vector_Point_R_const_vector_Size_R(const std::vector<cv::Point>* corners, const std::vector<cv::Size>* sizes, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = cv::detail::resultRoiIntersection(*corners, *sizes);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_resultRoi_const_vector_Point_R_const_vector_Size_R(const std::vector<cv::Point>* corners, const std::vector<cv::Size>* sizes, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = cv::detail::resultRoi(*corners, *sizes);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_resultRoi_const_vector_Point_R_const_vector_UMat_R(const std::vector<cv::Point>* corners, const std::vector<cv::UMat>* images, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = cv::detail::resultRoi(*corners, *images);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_resultTl_const_vector_Point_R(const std::vector<cv::Point>* corners, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = cv::detail::resultTl(*corners);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_selectRandomSubset_int_int_vector_int_R(int count, int size, std::vector<int>* subset, Result_void* ocvrs_return) {
		try {
			cv::detail::selectRandomSubset(count, size, *subset);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_stitchingLogLevel(Result<int>* ocvrs_return) {
		try {
			int ret = cv::detail::stitchingLogLevel();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_waveCorrect_vector_Mat_R_WaveCorrectKind(std::vector<cv::Mat>* rmats, cv::detail::WaveCorrectKind kind, Result_void* ocvrs_return) {
		try {
			cv::detail::waveCorrect(*rmats, kind);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AffineWarper_delete(cv::AffineWarper* instance) {
		delete instance;
	}
	void cv_AffineWarper_create_const_float(const cv::AffineWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_CompressedRectilinearPortraitWarper_delete(cv::CompressedRectilinearPortraitWarper* instance) {
		delete instance;
	}
	void cv_CompressedRectilinearPortraitWarper_CompressedRectilinearPortraitWarper_float_float(float A, float B, Result<cv::CompressedRectilinearPortraitWarper*>* ocvrs_return) {
		try {
			cv::CompressedRectilinearPortraitWarper* ret = new cv::CompressedRectilinearPortraitWarper(A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::CompressedRectilinearPortraitWarper*>))
	}
	
	void cv_CompressedRectilinearPortraitWarper_create_const_float(const cv::CompressedRectilinearPortraitWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_CompressedRectilinearWarper_delete(cv::CompressedRectilinearWarper* instance) {
		delete instance;
	}
	void cv_CompressedRectilinearWarper_CompressedRectilinearWarper_float_float(float A, float B, Result<cv::CompressedRectilinearWarper*>* ocvrs_return) {
		try {
			cv::CompressedRectilinearWarper* ret = new cv::CompressedRectilinearWarper(A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::CompressedRectilinearWarper*>))
	}
	
	void cv_CompressedRectilinearWarper_create_const_float(const cv::CompressedRectilinearWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_CylindricalWarper_delete(cv::CylindricalWarper* instance) {
		delete instance;
	}
	void cv_CylindricalWarper_create_const_float(const cv::CylindricalWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_FisheyeWarper_delete(cv::FisheyeWarper* instance) {
		delete instance;
	}
	void cv_FisheyeWarper_create_const_float(const cv::FisheyeWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_MercatorWarper_delete(cv::MercatorWarper* instance) {
		delete instance;
	}
	void cv_MercatorWarper_create_const_float(const cv::MercatorWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_PaniniPortraitWarper_delete(cv::PaniniPortraitWarper* instance) {
		delete instance;
	}
	void cv_PaniniPortraitWarper_PaniniPortraitWarper_float_float(float A, float B, Result<cv::PaniniPortraitWarper*>* ocvrs_return) {
		try {
			cv::PaniniPortraitWarper* ret = new cv::PaniniPortraitWarper(A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::PaniniPortraitWarper*>))
	}
	
	void cv_PaniniPortraitWarper_create_const_float(const cv::PaniniPortraitWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_PaniniWarper_delete(cv::PaniniWarper* instance) {
		delete instance;
	}
	void cv_PaniniWarper_PaniniWarper_float_float(float A, float B, Result<cv::PaniniWarper*>* ocvrs_return) {
		try {
			cv::PaniniWarper* ret = new cv::PaniniWarper(A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::PaniniWarper*>))
	}
	
	void cv_PaniniWarper_create_const_float(const cv::PaniniWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_PlaneWarper_delete(cv::PlaneWarper* instance) {
		delete instance;
	}
	void cv_PlaneWarper_create_const_float(const cv::PlaneWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_PyRotationWarper_delete(cv::PyRotationWarper* instance) {
		delete instance;
	}
	void cv_PyRotationWarper_PyRotationWarper_String_float(char* type, float scale, Result<cv::PyRotationWarper*>* ocvrs_return) {
		try {
			cv::PyRotationWarper* ret = new cv::PyRotationWarper(std::string(type), scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::PyRotationWarper*>))
	}
	
	void cv_PyRotationWarper_PyRotationWarper(Result<cv::PyRotationWarper*>* ocvrs_return) {
		try {
			cv::PyRotationWarper* ret = new cv::PyRotationWarper();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::PyRotationWarper*>))
	}
	
	void cv_PyRotationWarper_warpPoint_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::PyRotationWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPoint(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_PyRotationWarper_warpPointBackward_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::PyRotationWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPointBackward(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_PyRotationWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::PyRotationWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_PyRotationWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::PyRotationWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_PyRotationWarper_warpBackward_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_Size_const__OutputArrayR(cv::PyRotationWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, cv::Size* dst_size, const cv::_OutputArray* dst, Result_void* ocvrs_return) {
		try {
			instance->warpBackward(*src, *K, *R, interp_mode, border_mode, *dst_size, *dst);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_PyRotationWarper_warpRoi_Size_const__InputArrayR_const__InputArrayR(cv::PyRotationWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->warpRoi(*src_size, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_PyRotationWarper_getScale_const(const cv::PyRotationWarper* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_PyRotationWarper_setScale_float(cv::PyRotationWarper* instance, float unnamed, Result_void* ocvrs_return) {
		try {
			instance->setScale(unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_SphericalWarper_delete(cv::SphericalWarper* instance) {
		delete instance;
	}
	void cv_SphericalWarper_create_const_float(const cv::SphericalWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_StereographicWarper_delete(cv::StereographicWarper* instance) {
		delete instance;
	}
	void cv_StereographicWarper_create_const_float(const cv::StereographicWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_Stitcher_delete(cv::Stitcher* instance) {
		delete instance;
	}
	void cv_Stitcher_create_Mode(cv::Stitcher::Mode mode, Result<cv::Ptr<cv::Stitcher>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::Stitcher> ret = cv::Stitcher::create(mode);
			Ok(new cv::Ptr<cv::Stitcher>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::Stitcher>*>))
	}
	
	void cv_Stitcher_registrationResol_const(const cv::Stitcher* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->registrationResol();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_Stitcher_setRegistrationResol_double(cv::Stitcher* instance, double resol_mpx, Result_void* ocvrs_return) {
		try {
			instance->setRegistrationResol(resol_mpx);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_seamEstimationResol_const(const cv::Stitcher* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->seamEstimationResol();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_Stitcher_setSeamEstimationResol_double(cv::Stitcher* instance, double resol_mpx, Result_void* ocvrs_return) {
		try {
			instance->setSeamEstimationResol(resol_mpx);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_compositingResol_const(const cv::Stitcher* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->compositingResol();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_Stitcher_setCompositingResol_double(cv::Stitcher* instance, double resol_mpx, Result_void* ocvrs_return) {
		try {
			instance->setCompositingResol(resol_mpx);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_panoConfidenceThresh_const(const cv::Stitcher* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->panoConfidenceThresh();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_Stitcher_setPanoConfidenceThresh_double(cv::Stitcher* instance, double conf_thresh, Result_void* ocvrs_return) {
		try {
			instance->setPanoConfidenceThresh(conf_thresh);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_waveCorrection_const(const cv::Stitcher* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->waveCorrection();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_Stitcher_setWaveCorrection_bool(cv::Stitcher* instance, bool flag, Result_void* ocvrs_return) {
		try {
			instance->setWaveCorrection(flag);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_interpolationFlags_const(const cv::Stitcher* instance, Result<cv::InterpolationFlags>* ocvrs_return) {
		try {
			cv::InterpolationFlags ret = instance->interpolationFlags();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::InterpolationFlags>))
	}
	
	void cv_Stitcher_setInterpolationFlags_InterpolationFlags(cv::Stitcher* instance, cv::InterpolationFlags interp_flags, Result_void* ocvrs_return) {
		try {
			instance->setInterpolationFlags(interp_flags);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_waveCorrectKind_const(const cv::Stitcher* instance, Result<cv::detail::WaveCorrectKind>* ocvrs_return) {
		try {
			cv::detail::WaveCorrectKind ret = instance->waveCorrectKind();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::WaveCorrectKind>))
	}
	
	void cv_Stitcher_setWaveCorrectKind_WaveCorrectKind(cv::Stitcher* instance, cv::detail::WaveCorrectKind kind, Result_void* ocvrs_return) {
		try {
			instance->setWaveCorrectKind(kind);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_featuresFinder(cv::Stitcher* instance, Result<cv::Ptr<cv::Feature2D>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::Feature2D> ret = instance->featuresFinder();
			Ok(new cv::Ptr<cv::Feature2D>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::Feature2D>*>))
	}
	
	void cv_Stitcher_featuresFinder_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::Feature2D>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::Feature2D> ret = instance->featuresFinder();
			Ok(new const cv::Ptr<cv::Feature2D>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::Feature2D>*>))
	}
	
	void cv_Stitcher_setFeaturesFinder_Ptr_Feature2D_(cv::Stitcher* instance, cv::Ptr<cv::Feature2D>* features_finder, Result_void* ocvrs_return) {
		try {
			instance->setFeaturesFinder(*features_finder);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_featuresMatcher(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::FeaturesMatcher>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::FeaturesMatcher> ret = instance->featuresMatcher();
			Ok(new cv::Ptr<cv::detail::FeaturesMatcher>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::FeaturesMatcher>*>))
	}
	
	void cv_Stitcher_featuresMatcher_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::FeaturesMatcher>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::FeaturesMatcher> ret = instance->featuresMatcher();
			Ok(new const cv::Ptr<cv::detail::FeaturesMatcher>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::FeaturesMatcher>*>))
	}
	
	void cv_Stitcher_setFeaturesMatcher_Ptr_FeaturesMatcher_(cv::Stitcher* instance, cv::Ptr<cv::detail::FeaturesMatcher>* features_matcher, Result_void* ocvrs_return) {
		try {
			instance->setFeaturesMatcher(*features_matcher);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_matchingMask_const(const cv::Stitcher* instance, Result<cv::UMat*>* ocvrs_return) {
		try {
			const cv::UMat ret = instance->matchingMask();
			Ok(new const cv::UMat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::UMat*>))
	}
	
	void cv_Stitcher_setMatchingMask_const_UMatR(cv::Stitcher* instance, const cv::UMat* mask, Result_void* ocvrs_return) {
		try {
			instance->setMatchingMask(*mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_bundleAdjuster(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::BundleAdjusterBase>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::BundleAdjusterBase> ret = instance->bundleAdjuster();
			Ok(new cv::Ptr<cv::detail::BundleAdjusterBase>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::BundleAdjusterBase>*>))
	}
	
	void cv_Stitcher_bundleAdjuster_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::BundleAdjusterBase>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::BundleAdjusterBase> ret = instance->bundleAdjuster();
			Ok(new const cv::Ptr<cv::detail::BundleAdjusterBase>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::BundleAdjusterBase>*>))
	}
	
	void cv_Stitcher_setBundleAdjuster_Ptr_BundleAdjusterBase_(cv::Stitcher* instance, cv::Ptr<cv::detail::BundleAdjusterBase>* bundle_adjuster, Result_void* ocvrs_return) {
		try {
			instance->setBundleAdjuster(*bundle_adjuster);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_estimator(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::Estimator>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::Estimator> ret = instance->estimator();
			Ok(new cv::Ptr<cv::detail::Estimator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::Estimator>*>))
	}
	
	void cv_Stitcher_estimator_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::Estimator>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::Estimator> ret = instance->estimator();
			Ok(new const cv::Ptr<cv::detail::Estimator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::Estimator>*>))
	}
	
	void cv_Stitcher_setEstimator_Ptr_Estimator_(cv::Stitcher* instance, cv::Ptr<cv::detail::Estimator>* estimator, Result_void* ocvrs_return) {
		try {
			instance->setEstimator(*estimator);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_warper(cv::Stitcher* instance, Result<cv::Ptr<cv::WarperCreator>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::WarperCreator> ret = instance->warper();
			Ok(new cv::Ptr<cv::WarperCreator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::WarperCreator>*>))
	}
	
	void cv_Stitcher_warper_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::WarperCreator>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::WarperCreator> ret = instance->warper();
			Ok(new const cv::Ptr<cv::WarperCreator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::WarperCreator>*>))
	}
	
	void cv_Stitcher_setWarper_Ptr_WarperCreator_(cv::Stitcher* instance, cv::Ptr<cv::WarperCreator>* creator, Result_void* ocvrs_return) {
		try {
			instance->setWarper(*creator);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_exposureCompensator(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::ExposureCompensator>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::ExposureCompensator> ret = instance->exposureCompensator();
			Ok(new cv::Ptr<cv::detail::ExposureCompensator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::ExposureCompensator>*>))
	}
	
	void cv_Stitcher_exposureCompensator_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::ExposureCompensator>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::ExposureCompensator> ret = instance->exposureCompensator();
			Ok(new const cv::Ptr<cv::detail::ExposureCompensator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::ExposureCompensator>*>))
	}
	
	void cv_Stitcher_setExposureCompensator_Ptr_ExposureCompensator_(cv::Stitcher* instance, cv::Ptr<cv::detail::ExposureCompensator>* exposure_comp, Result_void* ocvrs_return) {
		try {
			instance->setExposureCompensator(*exposure_comp);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_seamFinder(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::SeamFinder>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::SeamFinder> ret = instance->seamFinder();
			Ok(new cv::Ptr<cv::detail::SeamFinder>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::SeamFinder>*>))
	}
	
	void cv_Stitcher_seamFinder_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::SeamFinder>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::SeamFinder> ret = instance->seamFinder();
			Ok(new const cv::Ptr<cv::detail::SeamFinder>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::SeamFinder>*>))
	}
	
	void cv_Stitcher_setSeamFinder_Ptr_SeamFinder_(cv::Stitcher* instance, cv::Ptr<cv::detail::SeamFinder>* seam_finder, Result_void* ocvrs_return) {
		try {
			instance->setSeamFinder(*seam_finder);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_blender(cv::Stitcher* instance, Result<cv::Ptr<cv::detail::Blender>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::Blender> ret = instance->blender();
			Ok(new cv::Ptr<cv::detail::Blender>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::Blender>*>))
	}
	
	void cv_Stitcher_blender_const(const cv::Stitcher* instance, Result<cv::Ptr<cv::detail::Blender>*>* ocvrs_return) {
		try {
			const cv::Ptr<cv::detail::Blender> ret = instance->blender();
			Ok(new const cv::Ptr<cv::detail::Blender>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::Blender>*>))
	}
	
	void cv_Stitcher_setBlender_Ptr_Blender_(cv::Stitcher* instance, cv::Ptr<cv::detail::Blender>* b, Result_void* ocvrs_return) {
		try {
			instance->setBlender(*b);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Stitcher_estimateTransform_const__InputArrayR_const__InputArrayR(cv::Stitcher* instance, const cv::_InputArray* images, const cv::_InputArray* masks, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->estimateTransform(*images, *masks);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_setTransform_const__InputArrayR_const_vector_CameraParams_R_const_vector_int_R(cv::Stitcher* instance, const cv::_InputArray* images, const std::vector<cv::detail::CameraParams>* cameras, const std::vector<int>* component, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->setTransform(*images, *cameras, *component);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_setTransform_const__InputArrayR_const_vector_CameraParams_R(cv::Stitcher* instance, const cv::_InputArray* images, const std::vector<cv::detail::CameraParams>* cameras, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->setTransform(*images, *cameras);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_composePanorama_const__OutputArrayR(cv::Stitcher* instance, const cv::_OutputArray* pano, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->composePanorama(*pano);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_composePanorama_const__InputArrayR_const__OutputArrayR(cv::Stitcher* instance, const cv::_InputArray* images, const cv::_OutputArray* pano, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->composePanorama(*images, *pano);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_stitch_const__InputArrayR_const__OutputArrayR(cv::Stitcher* instance, const cv::_InputArray* images, const cv::_OutputArray* pano, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->stitch(*images, *pano);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_stitch_const__InputArrayR_const__InputArrayR_const__OutputArrayR(cv::Stitcher* instance, const cv::_InputArray* images, const cv::_InputArray* masks, const cv::_OutputArray* pano, Result<cv::Stitcher::Status>* ocvrs_return) {
		try {
			cv::Stitcher::Status ret = instance->stitch(*images, *masks, *pano);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Stitcher::Status>))
	}
	
	void cv_Stitcher_component_const(const cv::Stitcher* instance, Result<std::vector<int>*>* ocvrs_return) {
		try {
			std::vector<int> ret = instance->component();
			Ok(new std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_Stitcher_cameras_const(const cv::Stitcher* instance, Result<std::vector<cv::detail::CameraParams>*>* ocvrs_return) {
		try {
			std::vector<cv::detail::CameraParams> ret = instance->cameras();
			Ok(new std::vector<cv::detail::CameraParams>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::detail::CameraParams>*>))
	}
	
	void cv_Stitcher_workScale_const(const cv::Stitcher* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->workScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_Stitcher_resultMask_const(const cv::Stitcher* instance, Result<cv::UMat*>* ocvrs_return) {
		try {
			cv::UMat ret = instance->resultMask();
			Ok(new cv::UMat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::UMat*>))
	}
	
	void cv_TransverseMercatorWarper_delete(cv::TransverseMercatorWarper* instance) {
		delete instance;
	}
	void cv_TransverseMercatorWarper_create_const_float(const cv::TransverseMercatorWarper* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_WarperCreator_create_const_float(const cv::WarperCreator* instance, float scale, Result<cv::Ptr<cv::detail::RotationWarper>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::RotationWarper> ret = instance->create(scale);
			Ok(new cv::Ptr<cv::detail::RotationWarper>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::RotationWarper>*>))
	}
	
	void cv_Detail_AffineBasedEstimator_delete(cv::detail::AffineBasedEstimator* instance) {
		delete instance;
	}
	void cv_detail_AffineBasedEstimator_AffineBasedEstimator(Result<cv::detail::AffineBasedEstimator*>* ocvrs_return) {
		try {
			cv::detail::AffineBasedEstimator* ret = new cv::detail::AffineBasedEstimator();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::AffineBasedEstimator*>))
	}
	
	cv::detail::BestOf2NearestMatcher* cv_Detail_AffineBestOf2NearestMatcher_to_Detail_BestOf2NearestMatcher(cv::detail::AffineBestOf2NearestMatcher* instance) {
		return dynamic_cast<cv::detail::BestOf2NearestMatcher*>(instance);
	}
	
	void cv_Detail_AffineBestOf2NearestMatcher_delete(cv::detail::AffineBestOf2NearestMatcher* instance) {
		delete instance;
	}
	void cv_detail_AffineBestOf2NearestMatcher_AffineBestOf2NearestMatcher_bool_bool_float_int(bool full_affine, bool try_use_gpu, float match_conf, int num_matches_thresh1, Result<cv::detail::AffineBestOf2NearestMatcher*>* ocvrs_return) {
		try {
			cv::detail::AffineBestOf2NearestMatcher* ret = new cv::detail::AffineBestOf2NearestMatcher(full_affine, try_use_gpu, match_conf, num_matches_thresh1);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::AffineBestOf2NearestMatcher*>))
	}
	
	cv::detail::PlaneWarper* cv_Detail_AffineWarper_to_Detail_PlaneWarper(cv::detail::AffineWarper* instance) {
		return dynamic_cast<cv::detail::PlaneWarper*>(instance);
	}
	
	void cv_Detail_AffineWarper_delete(cv::detail::AffineWarper* instance) {
		delete instance;
	}
	void cv_detail_AffineWarper_AffineWarper_float(float scale, Result<cv::detail::AffineWarper*>* ocvrs_return) {
		try {
			cv::detail::AffineWarper* ret = new cv::detail::AffineWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::AffineWarper*>))
	}
	
	void cv_detail_AffineWarper_warpPoint_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::AffineWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* H, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPoint(*pt, *K, *H);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_AffineWarper_warpPointBackward_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::AffineWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* H, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPointBackward(*pt, *K, *H);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_AffineWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::AffineWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* H, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *H, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_AffineWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::AffineWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* H, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *H, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_AffineWarper_warpRoi_Size_const__InputArrayR_const__InputArrayR(cv::detail::AffineWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* H, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->warpRoi(*src_size, *K, *H);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	cv::detail::AffineBestOf2NearestMatcher* cv_Detail_BestOf2NearestMatcher_to_Detail_AffineBestOf2NearestMatcher(cv::detail::BestOf2NearestMatcher* instance) {
		return dynamic_cast<cv::detail::AffineBestOf2NearestMatcher*>(instance);
	}
	
	cv::detail::BestOf2NearestRangeMatcher* cv_Detail_BestOf2NearestMatcher_to_Detail_BestOf2NearestRangeMatcher(cv::detail::BestOf2NearestMatcher* instance) {
		return dynamic_cast<cv::detail::BestOf2NearestRangeMatcher*>(instance);
	}
	
	void cv_Detail_BestOf2NearestMatcher_delete(cv::detail::BestOf2NearestMatcher* instance) {
		delete instance;
	}
	void cv_detail_BestOf2NearestMatcher_BestOf2NearestMatcher_bool_float_int_int(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2, Result<cv::detail::BestOf2NearestMatcher*>* ocvrs_return) {
		try {
			cv::detail::BestOf2NearestMatcher* ret = new cv::detail::BestOf2NearestMatcher(try_use_gpu, match_conf, num_matches_thresh1, num_matches_thresh2);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BestOf2NearestMatcher*>))
	}
	
	void cv_detail_BestOf2NearestMatcher_collectGarbage(cv::detail::BestOf2NearestMatcher* instance, Result_void* ocvrs_return) {
		try {
			instance->collectGarbage();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BestOf2NearestMatcher_create_bool_float_int_int(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2, Result<cv::Ptr<cv::detail::BestOf2NearestMatcher>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::BestOf2NearestMatcher> ret = cv::detail::BestOf2NearestMatcher::create(try_use_gpu, match_conf, num_matches_thresh1, num_matches_thresh2);
			Ok(new cv::Ptr<cv::detail::BestOf2NearestMatcher>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::BestOf2NearestMatcher>*>))
	}
	
	cv::detail::BestOf2NearestMatcher* cv_Detail_BestOf2NearestRangeMatcher_to_Detail_BestOf2NearestMatcher(cv::detail::BestOf2NearestRangeMatcher* instance) {
		return dynamic_cast<cv::detail::BestOf2NearestMatcher*>(instance);
	}
	
	void cv_Detail_BestOf2NearestRangeMatcher_delete(cv::detail::BestOf2NearestRangeMatcher* instance) {
		delete instance;
	}
	void cv_detail_BestOf2NearestRangeMatcher_BestOf2NearestRangeMatcher_int_bool_float_int_int(int range_width, bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2, Result<cv::detail::BestOf2NearestRangeMatcher*>* ocvrs_return) {
		try {
			cv::detail::BestOf2NearestRangeMatcher* ret = new cv::detail::BestOf2NearestRangeMatcher(range_width, try_use_gpu, match_conf, num_matches_thresh1, num_matches_thresh2);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BestOf2NearestRangeMatcher*>))
	}
	
	cv::detail::FeatherBlender* cv_Detail_Blender_to_Detail_FeatherBlender(cv::detail::Blender* instance) {
		return dynamic_cast<cv::detail::FeatherBlender*>(instance);
	}
	
	cv::detail::MultiBandBlender* cv_Detail_Blender_to_Detail_MultiBandBlender(cv::detail::Blender* instance) {
		return dynamic_cast<cv::detail::MultiBandBlender*>(instance);
	}
	
	void cv_Detail_Blender_delete(cv::detail::Blender* instance) {
		delete instance;
	}
	void cv_detail_Blender_createDefault_int_bool(int type, bool try_gpu, Result<cv::Ptr<cv::detail::Blender>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::Blender> ret = cv::detail::Blender::createDefault(type, try_gpu);
			Ok(new cv::Ptr<cv::detail::Blender>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::Blender>*>))
	}
	
	void cv_detail_Blender_prepare_const_vector_Point_R_const_vector_Size_R(cv::detail::Blender* instance, const std::vector<cv::Point>* corners, const std::vector<cv::Size>* sizes, Result_void* ocvrs_return) {
		try {
			instance->prepare(*corners, *sizes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_Blender_prepare_Rect(cv::detail::Blender* instance, cv::Rect* dst_roi, Result_void* ocvrs_return) {
		try {
			instance->prepare(*dst_roi);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_Blender_feed_const__InputArrayR_const__InputArrayR_Point(cv::detail::Blender* instance, const cv::_InputArray* img, const cv::_InputArray* mask, cv::Point* tl, Result_void* ocvrs_return) {
		try {
			instance->feed(*img, *mask, *tl);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_Blender_blend_const__InputOutputArrayR_const__InputOutputArrayR(cv::detail::Blender* instance, const cv::_InputOutputArray* dst, const cv::_InputOutputArray* dst_mask, Result_void* ocvrs_return) {
		try {
			instance->blend(*dst, *dst_mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_BlocksChannelsCompensator_delete(cv::detail::BlocksChannelsCompensator* instance) {
		delete instance;
	}
	void cv_detail_BlocksChannelsCompensator_BlocksChannelsCompensator_int_int_int(int bl_width, int bl_height, int nr_feeds, Result<cv::detail::BlocksChannelsCompensator*>* ocvrs_return) {
		try {
			cv::detail::BlocksChannelsCompensator* ret = new cv::detail::BlocksChannelsCompensator(bl_width, bl_height, nr_feeds);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BlocksChannelsCompensator*>))
	}
	
	void cv_detail_BlocksCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::BlocksCompensator* instance, int index, cv::Point* corner, const cv::_InputOutputArray* image, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->apply(index, *corner, *image, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_getMatGains_vector_Mat_R(cv::detail::BlocksCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_setMatGains_vector_Mat_R(cv::detail::BlocksCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_setNrFeeds_int(cv::detail::BlocksCompensator* instance, int nr_feeds, Result_void* ocvrs_return) {
		try {
			instance->setNrFeeds(nr_feeds);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_getNrFeeds(cv::detail::BlocksCompensator* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNrFeeds();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_BlocksCompensator_setSimilarityThreshold_double(cv::detail::BlocksCompensator* instance, double similarity_threshold, Result_void* ocvrs_return) {
		try {
			instance->setSimilarityThreshold(similarity_threshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_getSimilarityThreshold_const(const cv::detail::BlocksCompensator* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getSimilarityThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_detail_BlocksCompensator_setBlockSize_int_int(cv::detail::BlocksCompensator* instance, int width, int height, Result_void* ocvrs_return) {
		try {
			instance->setBlockSize(width, height);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_setBlockSize_Size(cv::detail::BlocksCompensator* instance, cv::Size* size, Result_void* ocvrs_return) {
		try {
			instance->setBlockSize(*size);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_getBlockSize_const(const cv::detail::BlocksCompensator* instance, Result<cv::Size>* ocvrs_return) {
		try {
			cv::Size ret = instance->getBlockSize();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Size>))
	}
	
	void cv_detail_BlocksCompensator_setNrGainsFilteringIterations_int(cv::detail::BlocksCompensator* instance, int nr_iterations, Result_void* ocvrs_return) {
		try {
			instance->setNrGainsFilteringIterations(nr_iterations);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksCompensator_getNrGainsFilteringIterations_const(const cv::detail::BlocksCompensator* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNrGainsFilteringIterations();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_Detail_BlocksGainCompensator_delete(cv::detail::BlocksGainCompensator* instance) {
		delete instance;
	}
	void cv_detail_BlocksGainCompensator_BlocksGainCompensator_int_int(int bl_width, int bl_height, Result<cv::detail::BlocksGainCompensator*>* ocvrs_return) {
		try {
			cv::detail::BlocksGainCompensator* ret = new cv::detail::BlocksGainCompensator(bl_width, bl_height);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BlocksGainCompensator*>))
	}
	
	void cv_detail_BlocksGainCompensator_BlocksGainCompensator_int_int_int(int bl_width, int bl_height, int nr_feeds, Result<cv::detail::BlocksGainCompensator*>* ocvrs_return) {
		try {
			cv::detail::BlocksGainCompensator* ret = new cv::detail::BlocksGainCompensator(bl_width, bl_height, nr_feeds);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BlocksGainCompensator*>))
	}
	
	void cv_detail_BlocksGainCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::BlocksGainCompensator* instance, int index, cv::Point* corner, const cv::_InputOutputArray* image, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->apply(index, *corner, *image, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksGainCompensator_getMatGains_vector_Mat_R(cv::detail::BlocksGainCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BlocksGainCompensator_setMatGains_vector_Mat_R(cv::detail::BlocksGainCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_BundleAdjusterAffine_delete(cv::detail::BundleAdjusterAffine* instance) {
		delete instance;
	}
	void cv_detail_BundleAdjusterAffine_BundleAdjusterAffine(Result<cv::detail::BundleAdjusterAffine*>* ocvrs_return) {
		try {
			cv::detail::BundleAdjusterAffine* ret = new cv::detail::BundleAdjusterAffine();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BundleAdjusterAffine*>))
	}
	
	void cv_Detail_BundleAdjusterAffinePartial_delete(cv::detail::BundleAdjusterAffinePartial* instance) {
		delete instance;
	}
	void cv_detail_BundleAdjusterAffinePartial_BundleAdjusterAffinePartial(Result<cv::detail::BundleAdjusterAffinePartial*>* ocvrs_return) {
		try {
			cv::detail::BundleAdjusterAffinePartial* ret = new cv::detail::BundleAdjusterAffinePartial();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BundleAdjusterAffinePartial*>))
	}
	
	void cv_detail_BundleAdjusterBase_refinementMask_const(const cv::detail::BundleAdjusterBase* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			const cv::Mat ret = instance->refinementMask();
			Ok(new const cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_detail_BundleAdjusterBase_setRefinementMask_const_MatR(cv::detail::BundleAdjusterBase* instance, const cv::Mat* mask, Result_void* ocvrs_return) {
		try {
			instance->setRefinementMask(*mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BundleAdjusterBase_confThresh_const(const cv::detail::BundleAdjusterBase* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->confThresh();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_detail_BundleAdjusterBase_setConfThresh_double(cv::detail::BundleAdjusterBase* instance, double conf_thresh, Result_void* ocvrs_return) {
		try {
			instance->setConfThresh(conf_thresh);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_BundleAdjusterBase_termCriteria(cv::detail::BundleAdjusterBase* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->termCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_detail_BundleAdjusterBase_setTermCriteria_const_TermCriteriaR(cv::detail::BundleAdjusterBase* instance, const cv::TermCriteria* term_criteria, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*term_criteria);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_BundleAdjusterRay_delete(cv::detail::BundleAdjusterRay* instance) {
		delete instance;
	}
	void cv_detail_BundleAdjusterRay_BundleAdjusterRay(Result<cv::detail::BundleAdjusterRay*>* ocvrs_return) {
		try {
			cv::detail::BundleAdjusterRay* ret = new cv::detail::BundleAdjusterRay();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BundleAdjusterRay*>))
	}
	
	void cv_Detail_BundleAdjusterReproj_delete(cv::detail::BundleAdjusterReproj* instance) {
		delete instance;
	}
	void cv_detail_BundleAdjusterReproj_BundleAdjusterReproj(Result<cv::detail::BundleAdjusterReproj*>* ocvrs_return) {
		try {
			cv::detail::BundleAdjusterReproj* ret = new cv::detail::BundleAdjusterReproj();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::BundleAdjusterReproj*>))
	}
	
	double cv_detail_CameraParams_getPropFocal_const(const cv::detail::CameraParams* instance) {
			double ret = instance->focal;
			return ret;
	}
	
	void cv_detail_CameraParams_setPropFocal_double(cv::detail::CameraParams* instance, double val) {
			instance->focal = val;
	}
	
	double cv_detail_CameraParams_getPropAspect_const(const cv::detail::CameraParams* instance) {
			double ret = instance->aspect;
			return ret;
	}
	
	void cv_detail_CameraParams_setPropAspect_double(cv::detail::CameraParams* instance, double val) {
			instance->aspect = val;
	}
	
	double cv_detail_CameraParams_getPropPpx_const(const cv::detail::CameraParams* instance) {
			double ret = instance->ppx;
			return ret;
	}
	
	void cv_detail_CameraParams_setPropPpx_double(cv::detail::CameraParams* instance, double val) {
			instance->ppx = val;
	}
	
	double cv_detail_CameraParams_getPropPpy_const(const cv::detail::CameraParams* instance) {
			double ret = instance->ppy;
			return ret;
	}
	
	void cv_detail_CameraParams_setPropPpy_double(cv::detail::CameraParams* instance, double val) {
			instance->ppy = val;
	}
	
	cv::Mat* cv_detail_CameraParams_getPropR_const(const cv::detail::CameraParams* instance) {
			cv::Mat ret = instance->R;
			return new cv::Mat(ret);
	}
	
	void cv_detail_CameraParams_setPropR_Mat(cv::detail::CameraParams* instance, cv::Mat* val) {
			instance->R = *val;
	}
	
	cv::Mat* cv_detail_CameraParams_getPropT_const(const cv::detail::CameraParams* instance) {
			cv::Mat ret = instance->t;
			return new cv::Mat(ret);
	}
	
	void cv_detail_CameraParams_setPropT_Mat(cv::detail::CameraParams* instance, cv::Mat* val) {
			instance->t = *val;
	}
	
	void cv_Detail_CameraParams_delete(cv::detail::CameraParams* instance) {
		delete instance;
	}
	void cv_detail_CameraParams_CameraParams(Result<cv::detail::CameraParams*>* ocvrs_return) {
		try {
			cv::detail::CameraParams* ret = new cv::detail::CameraParams();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CameraParams*>))
	}
	
	void cv_detail_CameraParams_CameraParams_const_CameraParamsR(const cv::detail::CameraParams* other, Result<cv::detail::CameraParams*>* ocvrs_return) {
		try {
			cv::detail::CameraParams* ret = new cv::detail::CameraParams(*other);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CameraParams*>))
	}
	
	void cv_detail_CameraParams_K_const(const cv::detail::CameraParams* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->K();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_Detail_ChannelsCompensator_delete(cv::detail::ChannelsCompensator* instance) {
		delete instance;
	}
	void cv_detail_ChannelsCompensator_ChannelsCompensator_int(int nr_feeds, Result<cv::detail::ChannelsCompensator*>* ocvrs_return) {
		try {
			cv::detail::ChannelsCompensator* ret = new cv::detail::ChannelsCompensator(nr_feeds);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::ChannelsCompensator*>))
	}
	
	void cv_detail_ChannelsCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::ChannelsCompensator* instance, int index, cv::Point* corner, const cv::_InputOutputArray* image, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->apply(index, *corner, *image, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ChannelsCompensator_getMatGains_vector_Mat_R(cv::detail::ChannelsCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ChannelsCompensator_setMatGains_vector_Mat_R(cv::detail::ChannelsCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ChannelsCompensator_setNrFeeds_int(cv::detail::ChannelsCompensator* instance, int nr_feeds, Result_void* ocvrs_return) {
		try {
			instance->setNrFeeds(nr_feeds);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ChannelsCompensator_getNrFeeds(cv::detail::ChannelsCompensator* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNrFeeds();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_ChannelsCompensator_setSimilarityThreshold_double(cv::detail::ChannelsCompensator* instance, double similarity_threshold, Result_void* ocvrs_return) {
		try {
			instance->setSimilarityThreshold(similarity_threshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ChannelsCompensator_getSimilarityThreshold_const(const cv::detail::ChannelsCompensator* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getSimilarityThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_detail_ChannelsCompensator_gains_const(const cv::detail::ChannelsCompensator* instance, Result<std::vector<cv::Scalar>*>* ocvrs_return) {
		try {
			std::vector<cv::Scalar> ret = instance->gains();
			Ok(new std::vector<cv::Scalar>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Scalar>*>))
	}
	
	float cv_detail_CompressedRectilinearPortraitProjector_getPropA_const(const cv::detail::CompressedRectilinearPortraitProjector* instance) {
			float ret = instance->a;
			return ret;
	}
	
	void cv_detail_CompressedRectilinearPortraitProjector_setPropA_float(cv::detail::CompressedRectilinearPortraitProjector* instance, float val) {
			instance->a = val;
	}
	
	float cv_detail_CompressedRectilinearPortraitProjector_getPropB_const(const cv::detail::CompressedRectilinearPortraitProjector* instance) {
			float ret = instance->b;
			return ret;
	}
	
	void cv_detail_CompressedRectilinearPortraitProjector_setPropB_float(cv::detail::CompressedRectilinearPortraitProjector* instance, float val) {
			instance->b = val;
	}
	
	cv::detail::ProjectorBase* cv_Detail_CompressedRectilinearPortraitProjector_to_Detail_ProjectorBase(cv::detail::CompressedRectilinearPortraitProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_CompressedRectilinearPortraitProjector_delete(cv::detail::CompressedRectilinearPortraitProjector* instance) {
		delete instance;
	}
	void cv_detail_CompressedRectilinearPortraitProjector_mapForward_float_float_floatR_floatR(cv::detail::CompressedRectilinearPortraitProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_CompressedRectilinearPortraitProjector_mapBackward_float_float_floatR_floatR(cv::detail::CompressedRectilinearPortraitProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_CompressedRectilinearPortraitWarper_delete(cv::detail::CompressedRectilinearPortraitWarper* instance) {
		delete instance;
	}
	void cv_detail_CompressedRectilinearPortraitWarper_CompressedRectilinearPortraitWarper_float_float_float(float scale, float A, float B, Result<cv::detail::CompressedRectilinearPortraitWarper*>* ocvrs_return) {
		try {
			cv::detail::CompressedRectilinearPortraitWarper* ret = new cv::detail::CompressedRectilinearPortraitWarper(scale, A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CompressedRectilinearPortraitWarper*>))
	}
	
	float cv_detail_CompressedRectilinearProjector_getPropA_const(const cv::detail::CompressedRectilinearProjector* instance) {
			float ret = instance->a;
			return ret;
	}
	
	void cv_detail_CompressedRectilinearProjector_setPropA_float(cv::detail::CompressedRectilinearProjector* instance, float val) {
			instance->a = val;
	}
	
	float cv_detail_CompressedRectilinearProjector_getPropB_const(const cv::detail::CompressedRectilinearProjector* instance) {
			float ret = instance->b;
			return ret;
	}
	
	void cv_detail_CompressedRectilinearProjector_setPropB_float(cv::detail::CompressedRectilinearProjector* instance, float val) {
			instance->b = val;
	}
	
	cv::detail::ProjectorBase* cv_Detail_CompressedRectilinearProjector_to_Detail_ProjectorBase(cv::detail::CompressedRectilinearProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_CompressedRectilinearProjector_delete(cv::detail::CompressedRectilinearProjector* instance) {
		delete instance;
	}
	void cv_detail_CompressedRectilinearProjector_mapForward_float_float_floatR_floatR(cv::detail::CompressedRectilinearProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_CompressedRectilinearProjector_mapBackward_float_float_floatR_floatR(cv::detail::CompressedRectilinearProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_CompressedRectilinearWarper_delete(cv::detail::CompressedRectilinearWarper* instance) {
		delete instance;
	}
	void cv_detail_CompressedRectilinearWarper_CompressedRectilinearWarper_float_float_float(float scale, float A, float B, Result<cv::detail::CompressedRectilinearWarper*>* ocvrs_return) {
		try {
			cv::detail::CompressedRectilinearWarper* ret = new cv::detail::CompressedRectilinearWarper(scale, A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CompressedRectilinearWarper*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_CylindricalPortraitProjector_to_Detail_ProjectorBase(cv::detail::CylindricalPortraitProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_CylindricalPortraitProjector_delete(cv::detail::CylindricalPortraitProjector* instance) {
		delete instance;
	}
	void cv_detail_CylindricalPortraitProjector_mapForward_float_float_floatR_floatR(cv::detail::CylindricalPortraitProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_CylindricalPortraitProjector_mapBackward_float_float_floatR_floatR(cv::detail::CylindricalPortraitProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_CylindricalPortraitWarper_delete(cv::detail::CylindricalPortraitWarper* instance) {
		delete instance;
	}
	void cv_detail_CylindricalPortraitWarper_CylindricalPortraitWarper_float(float scale, Result<cv::detail::CylindricalPortraitWarper*>* ocvrs_return) {
		try {
			cv::detail::CylindricalPortraitWarper* ret = new cv::detail::CylindricalPortraitWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CylindricalPortraitWarper*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_CylindricalProjector_to_Detail_ProjectorBase(cv::detail::CylindricalProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_CylindricalProjector_delete(cv::detail::CylindricalProjector* instance) {
		delete instance;
	}
	void cv_detail_CylindricalProjector_mapForward_float_float_floatR_floatR(cv::detail::CylindricalProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_CylindricalProjector_mapBackward_float_float_floatR_floatR(cv::detail::CylindricalProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::detail::CylindricalWarperGpu* cv_Detail_CylindricalWarper_to_Detail_CylindricalWarperGpu(cv::detail::CylindricalWarper* instance) {
		return dynamic_cast<cv::detail::CylindricalWarperGpu*>(instance);
	}
	
	void cv_Detail_CylindricalWarper_delete(cv::detail::CylindricalWarper* instance) {
		delete instance;
	}
	void cv_detail_CylindricalWarper_CylindricalWarper_float(float scale, Result<cv::detail::CylindricalWarper*>* ocvrs_return) {
		try {
			cv::detail::CylindricalWarper* ret = new cv::detail::CylindricalWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CylindricalWarper*>))
	}
	
	void cv_detail_CylindricalWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::CylindricalWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_CylindricalWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::CylindricalWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	cv::detail::CylindricalWarper* cv_Detail_CylindricalWarperGpu_to_Detail_CylindricalWarper(cv::detail::CylindricalWarperGpu* instance) {
		return dynamic_cast<cv::detail::CylindricalWarper*>(instance);
	}
	
	void cv_Detail_CylindricalWarperGpu_delete(cv::detail::CylindricalWarperGpu* instance) {
		delete instance;
	}
	void cv_detail_CylindricalWarperGpu_CylindricalWarperGpu_float(float scale, Result<cv::detail::CylindricalWarperGpu*>* ocvrs_return) {
		try {
			cv::detail::CylindricalWarperGpu* ret = new cv::detail::CylindricalWarperGpu(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::CylindricalWarperGpu*>))
	}
	
	void cv_detail_CylindricalWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::CylindricalWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_CylindricalWarperGpu_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::CylindricalWarperGpu* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_CylindricalWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_GpuMatR_GpuMatR(cv::detail::CylindricalWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, cv::cuda::GpuMat* xmap, cv::cuda::GpuMat* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_CylindricalWarperGpu_warp_const_GpuMatR_const__InputArrayR_const__InputArrayR_int_int_GpuMatR(cv::detail::CylindricalWarperGpu* instance, const cv::cuda::GpuMat* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, cv::cuda::GpuMat* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	std::vector<int>* cv_detail_DisjointSets_getPropParent_const(const cv::detail::DisjointSets* instance) {
			std::vector<int> ret = instance->parent;
			return new std::vector<int>(ret);
	}
	
	void cv_detail_DisjointSets_setPropParent_vector_int_(cv::detail::DisjointSets* instance, std::vector<int>* val) {
			instance->parent = *val;
	}
	
	std::vector<int>* cv_detail_DisjointSets_getPropSize_const(const cv::detail::DisjointSets* instance) {
			std::vector<int> ret = instance->size;
			return new std::vector<int>(ret);
	}
	
	void cv_detail_DisjointSets_setPropSize_vector_int_(cv::detail::DisjointSets* instance, std::vector<int>* val) {
			instance->size = *val;
	}
	
	void cv_Detail_DisjointSets_delete(cv::detail::DisjointSets* instance) {
		delete instance;
	}
	void cv_detail_DisjointSets_DisjointSets_int(int elem_count, Result<cv::detail::DisjointSets*>* ocvrs_return) {
		try {
			cv::detail::DisjointSets* ret = new cv::detail::DisjointSets(elem_count);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::DisjointSets*>))
	}
	
	void cv_detail_DisjointSets_createOneElemSets_int(cv::detail::DisjointSets* instance, int elem_count, Result_void* ocvrs_return) {
		try {
			instance->createOneElemSets(elem_count);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_DisjointSets_findSetByElem_int(cv::detail::DisjointSets* instance, int elem, Result<int>* ocvrs_return) {
		try {
			int ret = instance->findSetByElem(elem);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_DisjointSets_mergeSets_int_int(cv::detail::DisjointSets* instance, int set1, int set2, Result<int>* ocvrs_return) {
		try {
			int ret = instance->mergeSets(set1, set2);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_Detail_DpSeamFinder_delete(cv::detail::DpSeamFinder* instance) {
		delete instance;
	}
	void cv_detail_DpSeamFinder_DpSeamFinder_CostFunction(cv::detail::DpSeamFinder::CostFunction costFunc, Result<cv::detail::DpSeamFinder*>* ocvrs_return) {
		try {
			cv::detail::DpSeamFinder* ret = new cv::detail::DpSeamFinder(costFunc);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::DpSeamFinder*>))
	}
	
	void cv_detail_DpSeamFinder_DpSeamFinder_String(char* costFunc, Result<cv::detail::DpSeamFinder*>* ocvrs_return) {
		try {
			cv::detail::DpSeamFinder* ret = new cv::detail::DpSeamFinder(std::string(costFunc));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::DpSeamFinder*>))
	}
	
	void cv_detail_DpSeamFinder_costFunction_const(const cv::detail::DpSeamFinder* instance, Result<cv::detail::DpSeamFinder::CostFunction>* ocvrs_return) {
		try {
			cv::detail::DpSeamFinder::CostFunction ret = instance->costFunction();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::DpSeamFinder::CostFunction>))
	}
	
	void cv_detail_DpSeamFinder_setCostFunction_CostFunction(cv::detail::DpSeamFinder* instance, cv::detail::DpSeamFinder::CostFunction val, Result_void* ocvrs_return) {
		try {
			instance->setCostFunction(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_DpSeamFinder_setCostFunction_String(cv::detail::DpSeamFinder* instance, char* val, Result_void* ocvrs_return) {
		try {
			instance->setCostFunction(std::string(val));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_DpSeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::DpSeamFinder* instance, const std::vector<cv::UMat>* src, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*src, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_createDefault_int(int type, Result<cv::Ptr<cv::detail::ExposureCompensator>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::ExposureCompensator> ret = cv::detail::ExposureCompensator::createDefault(type);
			Ok(new cv::Ptr<cv::detail::ExposureCompensator>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::ExposureCompensator>*>))
	}
	
	void cv_detail_ExposureCompensator_feed_const_vector_Point_R_const_vector_UMat_R_const_vector_UMat_R(cv::detail::ExposureCompensator* instance, const std::vector<cv::Point>* corners, const std::vector<cv::UMat>* images, const std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->feed(*corners, *images, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::ExposureCompensator* instance, int index, cv::Point* corner, const cv::_InputOutputArray* image, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->apply(index, *corner, *image, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_getMatGains_vector_Mat_R(cv::detail::ExposureCompensator* instance, std::vector<cv::Mat>* unnamed, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_setMatGains_vector_Mat_R(cv::detail::ExposureCompensator* instance, std::vector<cv::Mat>* unnamed, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_setUpdateGain_bool(cv::detail::ExposureCompensator* instance, bool b, Result_void* ocvrs_return) {
		try {
			instance->setUpdateGain(b);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_ExposureCompensator_getUpdateGain(cv::detail::ExposureCompensator* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUpdateGain();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	cv::detail::Blender* cv_Detail_FeatherBlender_to_Detail_Blender(cv::detail::FeatherBlender* instance) {
		return dynamic_cast<cv::detail::Blender*>(instance);
	}
	
	void cv_Detail_FeatherBlender_delete(cv::detail::FeatherBlender* instance) {
		delete instance;
	}
	void cv_detail_FeatherBlender_FeatherBlender_float(float sharpness, Result<cv::detail::FeatherBlender*>* ocvrs_return) {
		try {
			cv::detail::FeatherBlender* ret = new cv::detail::FeatherBlender(sharpness);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::FeatherBlender*>))
	}
	
	void cv_detail_FeatherBlender_sharpness_const(const cv::detail::FeatherBlender* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->sharpness();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_detail_FeatherBlender_setSharpness_float(cv::detail::FeatherBlender* instance, float val, Result_void* ocvrs_return) {
		try {
			instance->setSharpness(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_FeatherBlender_prepare_Rect(cv::detail::FeatherBlender* instance, cv::Rect* dst_roi, Result_void* ocvrs_return) {
		try {
			instance->prepare(*dst_roi);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_FeatherBlender_feed_const__InputArrayR_const__InputArrayR_Point(cv::detail::FeatherBlender* instance, const cv::_InputArray* img, const cv::_InputArray* mask, cv::Point* tl, Result_void* ocvrs_return) {
		try {
			instance->feed(*img, *mask, *tl);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_FeatherBlender_blend_const__InputOutputArrayR_const__InputOutputArrayR(cv::detail::FeatherBlender* instance, const cv::_InputOutputArray* dst, const cv::_InputOutputArray* dst_mask, Result_void* ocvrs_return) {
		try {
			instance->blend(*dst, *dst_mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_FeatherBlender_createWeightMaps_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::FeatherBlender* instance, const std::vector<cv::UMat>* masks, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* weight_maps, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->createWeightMaps(*masks, *corners, *weight_maps);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_FeaturesMatcher_isThreadSafe_const(const cv::detail::FeaturesMatcher* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isThreadSafe();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_detail_FeaturesMatcher_collectGarbage(cv::detail::FeaturesMatcher* instance, Result_void* ocvrs_return) {
		try {
			instance->collectGarbage();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::detail::ProjectorBase* cv_Detail_FisheyeProjector_to_Detail_ProjectorBase(cv::detail::FisheyeProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_FisheyeProjector_delete(cv::detail::FisheyeProjector* instance) {
		delete instance;
	}
	void cv_detail_FisheyeProjector_mapForward_float_float_floatR_floatR(cv::detail::FisheyeProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_FisheyeProjector_mapBackward_float_float_floatR_floatR(cv::detail::FisheyeProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_FisheyeWarper_delete(cv::detail::FisheyeWarper* instance) {
		delete instance;
	}
	void cv_detail_FisheyeWarper_FisheyeWarper_float(float scale, Result<cv::detail::FisheyeWarper*>* ocvrs_return) {
		try {
			cv::detail::FisheyeWarper* ret = new cv::detail::FisheyeWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::FisheyeWarper*>))
	}
	
	void cv_Detail_GainCompensator_delete(cv::detail::GainCompensator* instance) {
		delete instance;
	}
	void cv_detail_GainCompensator_GainCompensator(Result<cv::detail::GainCompensator*>* ocvrs_return) {
		try {
			cv::detail::GainCompensator* ret = new cv::detail::GainCompensator();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::GainCompensator*>))
	}
	
	void cv_detail_GainCompensator_GainCompensator_int(int nr_feeds, Result<cv::detail::GainCompensator*>* ocvrs_return) {
		try {
			cv::detail::GainCompensator* ret = new cv::detail::GainCompensator(nr_feeds);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::GainCompensator*>))
	}
	
	void cv_detail_GainCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::GainCompensator* instance, int index, cv::Point* corner, const cv::_InputOutputArray* image, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->apply(index, *corner, *image, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_getMatGains_vector_Mat_R(cv::detail::GainCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_setMatGains_vector_Mat_R(cv::detail::GainCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_setNrFeeds_int(cv::detail::GainCompensator* instance, int nr_feeds, Result_void* ocvrs_return) {
		try {
			instance->setNrFeeds(nr_feeds);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_getNrFeeds(cv::detail::GainCompensator* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNrFeeds();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_GainCompensator_setSimilarityThreshold_double(cv::detail::GainCompensator* instance, double similarity_threshold, Result_void* ocvrs_return) {
		try {
			instance->setSimilarityThreshold(similarity_threshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_getSimilarityThreshold_const(const cv::detail::GainCompensator* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getSimilarityThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_detail_GainCompensator_prepareSimilarityMask_const_vector_Point_R_const_vector_UMat_R(cv::detail::GainCompensator* instance, const std::vector<cv::Point>* corners, const std::vector<cv::UMat>* images, Result_void* ocvrs_return) {
		try {
			instance->prepareSimilarityMask(*corners, *images);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_GainCompensator_gains_const(const cv::detail::GainCompensator* instance, Result<std::vector<double>*>* ocvrs_return) {
		try {
			std::vector<double> ret = instance->gains();
			Ok(new std::vector<double>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<double>*>))
	}
	
	void cv_Detail_Graph_delete(cv::detail::Graph* instance) {
		delete instance;
	}
	void cv_detail_Graph_Graph_int(int num_vertices, Result<cv::detail::Graph*>* ocvrs_return) {
		try {
			cv::detail::Graph* ret = new cv::detail::Graph(num_vertices);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::Graph*>))
	}
	
	void cv_detail_Graph_create_int(cv::detail::Graph* instance, int num_vertices, Result_void* ocvrs_return) {
		try {
			instance->create(num_vertices);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_Graph_numVertices_const(const cv::detail::Graph* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->numVertices();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_Graph_addEdge_int_int_float(cv::detail::Graph* instance, int from, int to, float weight, Result_void* ocvrs_return) {
		try {
			instance->addEdge(from, to, weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::detail::GraphCutSeamFinderBase* cv_Detail_GraphCutSeamFinder_to_Detail_GraphCutSeamFinderBase(cv::detail::GraphCutSeamFinder* instance) {
		return dynamic_cast<cv::detail::GraphCutSeamFinderBase*>(instance);
	}
	
	void cv_Detail_GraphCutSeamFinder_delete(cv::detail::GraphCutSeamFinder* instance) {
		delete instance;
	}
	void cv_detail_GraphCutSeamFinder_GraphCutSeamFinder_int_float_float(int cost_type, float terminal_cost, float bad_region_penalty, Result<cv::detail::GraphCutSeamFinder*>* ocvrs_return) {
		try {
			cv::detail::GraphCutSeamFinder* ret = new cv::detail::GraphCutSeamFinder(cost_type, terminal_cost, bad_region_penalty);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::GraphCutSeamFinder*>))
	}
	
	void cv_detail_GraphCutSeamFinder_GraphCutSeamFinder_String_float_float(char* cost_type, float terminal_cost, float bad_region_penalty, Result<cv::detail::GraphCutSeamFinder*>* ocvrs_return) {
		try {
			cv::detail::GraphCutSeamFinder* ret = new cv::detail::GraphCutSeamFinder(std::string(cost_type), terminal_cost, bad_region_penalty);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::GraphCutSeamFinder*>))
	}
	
	void cv_detail_GraphCutSeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::GraphCutSeamFinder* instance, const std::vector<cv::UMat>* src, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*src, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_GraphCutSeamFinderBase_delete(cv::detail::GraphCutSeamFinderBase* instance) {
		delete instance;
	}
	int cv_detail_GraphEdge_getPropFrom_const(const cv::detail::GraphEdge* instance) {
			int ret = instance->from;
			return ret;
	}
	
	void cv_detail_GraphEdge_setPropFrom_int(cv::detail::GraphEdge* instance, int val) {
			instance->from = val;
	}
	
	int cv_detail_GraphEdge_getPropTo_const(const cv::detail::GraphEdge* instance) {
			int ret = instance->to;
			return ret;
	}
	
	void cv_detail_GraphEdge_setPropTo_int(cv::detail::GraphEdge* instance, int val) {
			instance->to = val;
	}
	
	float cv_detail_GraphEdge_getPropWeight_const(const cv::detail::GraphEdge* instance) {
			float ret = instance->weight;
			return ret;
	}
	
	void cv_detail_GraphEdge_setPropWeight_float(cv::detail::GraphEdge* instance, float val) {
			instance->weight = val;
	}
	
	void cv_Detail_GraphEdge_delete(cv::detail::GraphEdge* instance) {
		delete instance;
	}
	void cv_detail_GraphEdge_GraphEdge_int_int_float(int from, int to, float weight, Result<cv::detail::GraphEdge*>* ocvrs_return) {
		try {
			cv::detail::GraphEdge* ret = new cv::detail::GraphEdge(from, to, weight);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::GraphEdge*>))
	}
	
	void cv_Detail_HomographyBasedEstimator_delete(cv::detail::HomographyBasedEstimator* instance) {
		delete instance;
	}
	void cv_detail_HomographyBasedEstimator_HomographyBasedEstimator_bool(bool is_focals_estimated, Result<cv::detail::HomographyBasedEstimator*>* ocvrs_return) {
		try {
			cv::detail::HomographyBasedEstimator* ret = new cv::detail::HomographyBasedEstimator(is_focals_estimated);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::HomographyBasedEstimator*>))
	}
	
	int cv_detail_ImageFeatures_getPropImg_idx_const(const cv::detail::ImageFeatures* instance) {
			int ret = instance->img_idx;
			return ret;
	}
	
	void cv_detail_ImageFeatures_setPropImg_idx_int(cv::detail::ImageFeatures* instance, int val) {
			instance->img_idx = val;
	}
	
	void cv_detail_ImageFeatures_getPropImg_size_const(const cv::detail::ImageFeatures* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->img_size;
			*ocvrs_return = ret;
	}
	
	void cv_detail_ImageFeatures_setPropImg_size_Size(cv::detail::ImageFeatures* instance, cv::Size* val) {
			instance->img_size = *val;
	}
	
	std::vector<cv::KeyPoint>* cv_detail_ImageFeatures_getPropKeypoints_const(const cv::detail::ImageFeatures* instance) {
			std::vector<cv::KeyPoint> ret = instance->keypoints;
			return new std::vector<cv::KeyPoint>(ret);
	}
	
	void cv_detail_ImageFeatures_setPropKeypoints_vector_KeyPoint_(cv::detail::ImageFeatures* instance, std::vector<cv::KeyPoint>* val) {
			instance->keypoints = *val;
	}
	
	cv::UMat* cv_detail_ImageFeatures_getPropDescriptors_const(const cv::detail::ImageFeatures* instance) {
			cv::UMat ret = instance->descriptors;
			return new cv::UMat(ret);
	}
	
	void cv_detail_ImageFeatures_setPropDescriptors_UMat(cv::detail::ImageFeatures* instance, cv::UMat* val) {
			instance->descriptors = *val;
	}
	
	void cv_Detail_ImageFeatures_delete(cv::detail::ImageFeatures* instance) {
		delete instance;
	}
	void cv_detail_ImageFeatures_getKeypoints(cv::detail::ImageFeatures* instance, Result<std::vector<cv::KeyPoint>*>* ocvrs_return) {
		try {
			std::vector<cv::KeyPoint> ret = instance->getKeypoints();
			Ok(new std::vector<cv::KeyPoint>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::KeyPoint>*>))
	}
	
	int cv_detail_MatchesInfo_getPropSrc_img_idx_const(const cv::detail::MatchesInfo* instance) {
			int ret = instance->src_img_idx;
			return ret;
	}
	
	void cv_detail_MatchesInfo_setPropSrc_img_idx_int(cv::detail::MatchesInfo* instance, int val) {
			instance->src_img_idx = val;
	}
	
	int cv_detail_MatchesInfo_getPropDst_img_idx_const(const cv::detail::MatchesInfo* instance) {
			int ret = instance->dst_img_idx;
			return ret;
	}
	
	void cv_detail_MatchesInfo_setPropDst_img_idx_int(cv::detail::MatchesInfo* instance, int val) {
			instance->dst_img_idx = val;
	}
	
	std::vector<cv::DMatch>* cv_detail_MatchesInfo_getPropMatches_const(const cv::detail::MatchesInfo* instance) {
			std::vector<cv::DMatch> ret = instance->matches;
			return new std::vector<cv::DMatch>(ret);
	}
	
	void cv_detail_MatchesInfo_setPropMatches_vector_DMatch_(cv::detail::MatchesInfo* instance, std::vector<cv::DMatch>* val) {
			instance->matches = *val;
	}
	
	std::vector<unsigned char>* cv_detail_MatchesInfo_getPropInliers_mask_const(const cv::detail::MatchesInfo* instance) {
			std::vector<unsigned char> ret = instance->inliers_mask;
			return new std::vector<unsigned char>(ret);
	}
	
	void cv_detail_MatchesInfo_setPropInliers_mask_vector_unsigned_char_(cv::detail::MatchesInfo* instance, std::vector<unsigned char>* val) {
			instance->inliers_mask = *val;
	}
	
	int cv_detail_MatchesInfo_getPropNum_inliers_const(const cv::detail::MatchesInfo* instance) {
			int ret = instance->num_inliers;
			return ret;
	}
	
	void cv_detail_MatchesInfo_setPropNum_inliers_int(cv::detail::MatchesInfo* instance, int val) {
			instance->num_inliers = val;
	}
	
	cv::Mat* cv_detail_MatchesInfo_getPropH_const(const cv::detail::MatchesInfo* instance) {
			cv::Mat ret = instance->H;
			return new cv::Mat(ret);
	}
	
	void cv_detail_MatchesInfo_setPropH_Mat(cv::detail::MatchesInfo* instance, cv::Mat* val) {
			instance->H = *val;
	}
	
	double cv_detail_MatchesInfo_getPropConfidence_const(const cv::detail::MatchesInfo* instance) {
			double ret = instance->confidence;
			return ret;
	}
	
	void cv_detail_MatchesInfo_setPropConfidence_double(cv::detail::MatchesInfo* instance, double val) {
			instance->confidence = val;
	}
	
	void cv_Detail_MatchesInfo_delete(cv::detail::MatchesInfo* instance) {
		delete instance;
	}
	void cv_detail_MatchesInfo_MatchesInfo(Result<cv::detail::MatchesInfo*>* ocvrs_return) {
		try {
			cv::detail::MatchesInfo* ret = new cv::detail::MatchesInfo();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::MatchesInfo*>))
	}
	
	void cv_detail_MatchesInfo_MatchesInfo_const_MatchesInfoR(const cv::detail::MatchesInfo* other, Result<cv::detail::MatchesInfo*>* ocvrs_return) {
		try {
			cv::detail::MatchesInfo* ret = new cv::detail::MatchesInfo(*other);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::MatchesInfo*>))
	}
	
	void cv_detail_MatchesInfo_getMatches(cv::detail::MatchesInfo* instance, Result<std::vector<cv::DMatch>*>* ocvrs_return) {
		try {
			std::vector<cv::DMatch> ret = instance->getMatches();
			Ok(new std::vector<cv::DMatch>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::DMatch>*>))
	}
	
	void cv_detail_MatchesInfo_getInliers(cv::detail::MatchesInfo* instance, Result<std::vector<unsigned char>*>* ocvrs_return) {
		try {
			std::vector<unsigned char> ret = instance->getInliers();
			Ok(new std::vector<unsigned char>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<unsigned char>*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_MercatorProjector_to_Detail_ProjectorBase(cv::detail::MercatorProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_MercatorProjector_delete(cv::detail::MercatorProjector* instance) {
		delete instance;
	}
	void cv_detail_MercatorProjector_mapForward_float_float_floatR_floatR(cv::detail::MercatorProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_MercatorProjector_mapBackward_float_float_floatR_floatR(cv::detail::MercatorProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_MercatorWarper_delete(cv::detail::MercatorWarper* instance) {
		delete instance;
	}
	void cv_detail_MercatorWarper_MercatorWarper_float(float scale, Result<cv::detail::MercatorWarper*>* ocvrs_return) {
		try {
			cv::detail::MercatorWarper* ret = new cv::detail::MercatorWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::MercatorWarper*>))
	}
	
	cv::detail::Blender* cv_Detail_MultiBandBlender_to_Detail_Blender(cv::detail::MultiBandBlender* instance) {
		return dynamic_cast<cv::detail::Blender*>(instance);
	}
	
	void cv_Detail_MultiBandBlender_delete(cv::detail::MultiBandBlender* instance) {
		delete instance;
	}
	void cv_detail_MultiBandBlender_MultiBandBlender_int_int_int(int try_gpu, int num_bands, int weight_type, Result<cv::detail::MultiBandBlender*>* ocvrs_return) {
		try {
			cv::detail::MultiBandBlender* ret = new cv::detail::MultiBandBlender(try_gpu, num_bands, weight_type);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::MultiBandBlender*>))
	}
	
	void cv_detail_MultiBandBlender_numBands_const(const cv::detail::MultiBandBlender* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->numBands();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_detail_MultiBandBlender_setNumBands_int(cv::detail::MultiBandBlender* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setNumBands(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_MultiBandBlender_prepare_Rect(cv::detail::MultiBandBlender* instance, cv::Rect* dst_roi, Result_void* ocvrs_return) {
		try {
			instance->prepare(*dst_roi);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_MultiBandBlender_feed_const__InputArrayR_const__InputArrayR_Point(cv::detail::MultiBandBlender* instance, const cv::_InputArray* img, const cv::_InputArray* mask, cv::Point* tl, Result_void* ocvrs_return) {
		try {
			instance->feed(*img, *mask, *tl);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_MultiBandBlender_blend_const__InputOutputArrayR_const__InputOutputArrayR(cv::detail::MultiBandBlender* instance, const cv::_InputOutputArray* dst, const cv::_InputOutputArray* dst_mask, Result_void* ocvrs_return) {
		try {
			instance->blend(*dst, *dst_mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_NoBundleAdjuster_delete(cv::detail::NoBundleAdjuster* instance) {
		delete instance;
	}
	void cv_detail_NoBundleAdjuster_NoBundleAdjuster(Result<cv::detail::NoBundleAdjuster*>* ocvrs_return) {
		try {
			cv::detail::NoBundleAdjuster* ret = new cv::detail::NoBundleAdjuster();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::NoBundleAdjuster*>))
	}
	
	void cv_Detail_NoExposureCompensator_delete(cv::detail::NoExposureCompensator* instance) {
		delete instance;
	}
	void cv_detail_NoExposureCompensator_apply_int_Point_const__InputOutputArrayR_const__InputArrayR(cv::detail::NoExposureCompensator* instance, int unnamed, cv::Point* unnamed_1, const cv::_InputOutputArray* unnamed_2, const cv::_InputArray* unnamed_3, Result_void* ocvrs_return) {
		try {
			instance->apply(unnamed, *unnamed_1, *unnamed_2, *unnamed_3);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_NoExposureCompensator_getMatGains_vector_Mat_R(cv::detail::NoExposureCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->getMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_NoExposureCompensator_setMatGains_vector_Mat_R(cv::detail::NoExposureCompensator* instance, std::vector<cv::Mat>* umv, Result_void* ocvrs_return) {
		try {
			instance->setMatGains(*umv);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_NoSeamFinder_delete(cv::detail::NoSeamFinder* instance) {
		delete instance;
	}
	void cv_detail_NoSeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::NoSeamFinder* instance, const std::vector<cv::UMat>* unnamed, const std::vector<cv::Point>* unnamed_1, std::vector<cv::UMat>* unnamed_2, Result_void* ocvrs_return) {
		try {
			instance->find(*unnamed, *unnamed_1, *unnamed_2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_PairwiseSeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::PairwiseSeamFinder* instance, const std::vector<cv::UMat>* src, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*src, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	float cv_detail_PaniniPortraitProjector_getPropA_const(const cv::detail::PaniniPortraitProjector* instance) {
			float ret = instance->a;
			return ret;
	}
	
	void cv_detail_PaniniPortraitProjector_setPropA_float(cv::detail::PaniniPortraitProjector* instance, float val) {
			instance->a = val;
	}
	
	float cv_detail_PaniniPortraitProjector_getPropB_const(const cv::detail::PaniniPortraitProjector* instance) {
			float ret = instance->b;
			return ret;
	}
	
	void cv_detail_PaniniPortraitProjector_setPropB_float(cv::detail::PaniniPortraitProjector* instance, float val) {
			instance->b = val;
	}
	
	cv::detail::ProjectorBase* cv_Detail_PaniniPortraitProjector_to_Detail_ProjectorBase(cv::detail::PaniniPortraitProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_PaniniPortraitProjector_delete(cv::detail::PaniniPortraitProjector* instance) {
		delete instance;
	}
	void cv_detail_PaniniPortraitProjector_mapForward_float_float_floatR_floatR(cv::detail::PaniniPortraitProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_PaniniPortraitProjector_mapBackward_float_float_floatR_floatR(cv::detail::PaniniPortraitProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_PaniniPortraitWarper_delete(cv::detail::PaniniPortraitWarper* instance) {
		delete instance;
	}
	void cv_detail_PaniniPortraitWarper_PaniniPortraitWarper_float_float_float(float scale, float A, float B, Result<cv::detail::PaniniPortraitWarper*>* ocvrs_return) {
		try {
			cv::detail::PaniniPortraitWarper* ret = new cv::detail::PaniniPortraitWarper(scale, A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::PaniniPortraitWarper*>))
	}
	
	float cv_detail_PaniniProjector_getPropA_const(const cv::detail::PaniniProjector* instance) {
			float ret = instance->a;
			return ret;
	}
	
	void cv_detail_PaniniProjector_setPropA_float(cv::detail::PaniniProjector* instance, float val) {
			instance->a = val;
	}
	
	float cv_detail_PaniniProjector_getPropB_const(const cv::detail::PaniniProjector* instance) {
			float ret = instance->b;
			return ret;
	}
	
	void cv_detail_PaniniProjector_setPropB_float(cv::detail::PaniniProjector* instance, float val) {
			instance->b = val;
	}
	
	cv::detail::ProjectorBase* cv_Detail_PaniniProjector_to_Detail_ProjectorBase(cv::detail::PaniniProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_PaniniProjector_delete(cv::detail::PaniniProjector* instance) {
		delete instance;
	}
	void cv_detail_PaniniProjector_mapForward_float_float_floatR_floatR(cv::detail::PaniniProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_PaniniProjector_mapBackward_float_float_floatR_floatR(cv::detail::PaniniProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_PaniniWarper_delete(cv::detail::PaniniWarper* instance) {
		delete instance;
	}
	void cv_detail_PaniniWarper_PaniniWarper_float_float_float(float scale, float A, float B, Result<cv::detail::PaniniWarper*>* ocvrs_return) {
		try {
			cv::detail::PaniniWarper* ret = new cv::detail::PaniniWarper(scale, A, B);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::PaniniWarper*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_PlanePortraitProjector_to_Detail_ProjectorBase(cv::detail::PlanePortraitProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_PlanePortraitProjector_delete(cv::detail::PlanePortraitProjector* instance) {
		delete instance;
	}
	void cv_detail_PlanePortraitProjector_mapForward_float_float_floatR_floatR(cv::detail::PlanePortraitProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_PlanePortraitProjector_mapBackward_float_float_floatR_floatR(cv::detail::PlanePortraitProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_PlanePortraitWarper_delete(cv::detail::PlanePortraitWarper* instance) {
		delete instance;
	}
	void cv_detail_PlanePortraitWarper_PlanePortraitWarper_float(float scale, Result<cv::detail::PlanePortraitWarper*>* ocvrs_return) {
		try {
			cv::detail::PlanePortraitWarper* ret = new cv::detail::PlanePortraitWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::PlanePortraitWarper*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_PlaneProjector_to_Detail_ProjectorBase(cv::detail::PlaneProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_PlaneProjector_delete(cv::detail::PlaneProjector* instance) {
		delete instance;
	}
	void cv_detail_PlaneProjector_mapForward_float_float_floatR_floatR(cv::detail::PlaneProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_PlaneProjector_mapBackward_float_float_floatR_floatR(cv::detail::PlaneProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::detail::AffineWarper* cv_Detail_PlaneWarper_to_Detail_AffineWarper(cv::detail::PlaneWarper* instance) {
		return dynamic_cast<cv::detail::AffineWarper*>(instance);
	}
	
	cv::detail::PlaneWarperGpu* cv_Detail_PlaneWarper_to_Detail_PlaneWarperGpu(cv::detail::PlaneWarper* instance) {
		return dynamic_cast<cv::detail::PlaneWarperGpu*>(instance);
	}
	
	void cv_Detail_PlaneWarper_delete(cv::detail::PlaneWarper* instance) {
		delete instance;
	}
	void cv_detail_PlaneWarper_PlaneWarper_float(float scale, Result<cv::detail::PlaneWarper*>* ocvrs_return) {
		try {
			cv::detail::PlaneWarper* ret = new cv::detail::PlaneWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::PlaneWarper*>))
	}
	
	void cv_detail_PlaneWarper_warpPoint_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPoint(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_PlaneWarper_warpPoint_const_Point2fR_const__InputArrayR_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPoint(*pt, *K, *R, *T);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_PlaneWarper_warpPointBackward_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPointBackward(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_PlaneWarper_warpPointBackward_const_Point2fR_const__InputArrayR_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPointBackward(*pt, *K, *R, *T);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_PlaneWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::PlaneWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *T, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::PlaneWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::PlaneWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_PlaneWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::PlaneWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, *T, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_PlaneWarper_warpRoi_Size_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->warpRoi(*src_size, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarper_warpRoi_Size_const__InputArrayR_const__InputArrayR_const__InputArrayR(cv::detail::PlaneWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->warpRoi(*src_size, *K, *R, *T);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	cv::detail::PlaneWarper* cv_Detail_PlaneWarperGpu_to_Detail_PlaneWarper(cv::detail::PlaneWarperGpu* instance) {
		return dynamic_cast<cv::detail::PlaneWarper*>(instance);
	}
	
	void cv_Detail_PlaneWarperGpu_delete(cv::detail::PlaneWarperGpu* instance) {
		delete instance;
	}
	void cv_detail_PlaneWarperGpu_PlaneWarperGpu_float(float scale, Result<cv::detail::PlaneWarperGpu*>* ocvrs_return) {
		try {
			cv::detail::PlaneWarperGpu* ret = new cv::detail::PlaneWarperGpu(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::PlaneWarperGpu*>))
	}
	
	void cv_detail_PlaneWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::PlaneWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::PlaneWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *T, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarperGpu_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::PlaneWarperGpu* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_PlaneWarperGpu_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::PlaneWarperGpu* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, *T, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_PlaneWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_GpuMatR_GpuMatR(cv::detail::PlaneWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, cv::cuda::GpuMat* xmap, cv::cuda::GpuMat* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__InputArrayR_GpuMatR_GpuMatR(cv::detail::PlaneWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, cv::cuda::GpuMat* xmap, cv::cuda::GpuMat* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *T, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_PlaneWarperGpu_warp_const_GpuMatR_const__InputArrayR_const__InputArrayR_int_int_GpuMatR(cv::detail::PlaneWarperGpu* instance, const cv::cuda::GpuMat* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, cv::cuda::GpuMat* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_PlaneWarperGpu_warp_const_GpuMatR_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_GpuMatR(cv::detail::PlaneWarperGpu* instance, const cv::cuda::GpuMat* src, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, int interp_mode, int border_mode, cv::cuda::GpuMat* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, *T, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	float cv_detail_ProjectorBase_getPropScale_const(const cv::detail::ProjectorBase* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_detail_ProjectorBase_setPropScale_float(cv::detail::ProjectorBase* instance, float val) {
			instance->scale = val;
	}
	
	float** cv_detail_ProjectorBase_getPropK(cv::detail::ProjectorBase* instance) {
			float(*ret)[9] = &instance->k;
			return (float**)ret;
	}
	
	float** cv_detail_ProjectorBase_getPropRinv(cv::detail::ProjectorBase* instance) {
			float(*ret)[9] = &instance->rinv;
			return (float**)ret;
	}
	
	float** cv_detail_ProjectorBase_getPropR_kinv(cv::detail::ProjectorBase* instance) {
			float(*ret)[9] = &instance->r_kinv;
			return (float**)ret;
	}
	
	float** cv_detail_ProjectorBase_getPropK_rinv(cv::detail::ProjectorBase* instance) {
			float(*ret)[9] = &instance->k_rinv;
			return (float**)ret;
	}
	
	float** cv_detail_ProjectorBase_getPropT(cv::detail::ProjectorBase* instance) {
			float(*ret)[3] = &instance->t;
			return (float**)ret;
	}
	
	void cv_Detail_ProjectorBase_delete(cv::detail::ProjectorBase* instance) {
		delete instance;
	}
	void cv_detail_ProjectorBase_setCameraParams_const__InputArrayR_const__InputArrayR_const__InputArrayR(cv::detail::ProjectorBase* instance, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_InputArray* T, Result_void* ocvrs_return) {
		try {
			instance->setCameraParams(*K, *R, *T);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_RotationWarper_warpPoint_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::RotationWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPoint(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_RotationWarper_warpPointBackward_const_Point2fR_const__InputArrayR_const__InputArrayR(cv::detail::RotationWarper* instance, const cv::Point2f* pt, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Point2f>* ocvrs_return) {
		try {
			cv::Point2f ret = instance->warpPointBackward(*pt, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point2f>))
	}
	
	void cv_detail_RotationWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::RotationWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_RotationWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::RotationWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_RotationWarper_warpBackward_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_Size_const__OutputArrayR(cv::detail::RotationWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, cv::Size* dst_size, const cv::_OutputArray* dst, Result_void* ocvrs_return) {
		try {
			instance->warpBackward(*src, *K, *R, interp_mode, border_mode, *dst_size, *dst);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_RotationWarper_warpRoi_Size_const__InputArrayR_const__InputArrayR(cv::detail::RotationWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->warpRoi(*src_size, *K, *R);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_RotationWarper_getScale_const(const cv::detail::RotationWarper* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_detail_RotationWarper_setScale_float(cv::detail::RotationWarper* instance, float unnamed, Result_void* ocvrs_return) {
		try {
			instance->setScale(unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_SeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::SeamFinder* instance, const std::vector<cv::UMat>* src, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*src, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_SeamFinder_createDefault_int(int type, Result<cv::Ptr<cv::detail::SeamFinder>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::detail::SeamFinder> ret = cv::detail::SeamFinder::createDefault(type);
			Ok(new cv::Ptr<cv::detail::SeamFinder>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::detail::SeamFinder>*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_SphericalPortraitProjector_to_Detail_ProjectorBase(cv::detail::SphericalPortraitProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_SphericalPortraitProjector_delete(cv::detail::SphericalPortraitProjector* instance) {
		delete instance;
	}
	void cv_detail_SphericalPortraitProjector_mapForward_float_float_floatR_floatR(cv::detail::SphericalPortraitProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_SphericalPortraitProjector_mapBackward_float_float_floatR_floatR(cv::detail::SphericalPortraitProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_SphericalPortraitWarper_delete(cv::detail::SphericalPortraitWarper* instance) {
		delete instance;
	}
	void cv_detail_SphericalPortraitWarper_SphericalPortraitWarper_float(float scale, Result<cv::detail::SphericalPortraitWarper*>* ocvrs_return) {
		try {
			cv::detail::SphericalPortraitWarper* ret = new cv::detail::SphericalPortraitWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::SphericalPortraitWarper*>))
	}
	
	void cv_detail_SphericalProjector_mapForward_float_float_floatR_floatR(cv::detail::SphericalProjector instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance.mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_SphericalProjector_mapBackward_float_float_floatR_floatR(cv::detail::SphericalProjector instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance.mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::detail::SphericalWarperGpu* cv_Detail_SphericalWarper_to_Detail_SphericalWarperGpu(cv::detail::SphericalWarper* instance) {
		return dynamic_cast<cv::detail::SphericalWarperGpu*>(instance);
	}
	
	void cv_Detail_SphericalWarper_delete(cv::detail::SphericalWarper* instance) {
		delete instance;
	}
	void cv_detail_SphericalWarper_SphericalWarper_float(float scale, Result<cv::detail::SphericalWarper*>* ocvrs_return) {
		try {
			cv::detail::SphericalWarper* ret = new cv::detail::SphericalWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::SphericalWarper*>))
	}
	
	void cv_detail_SphericalWarper_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::SphericalWarper* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_SphericalWarper_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::SphericalWarper* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	cv::detail::SphericalWarper* cv_Detail_SphericalWarperGpu_to_Detail_SphericalWarper(cv::detail::SphericalWarperGpu* instance) {
		return dynamic_cast<cv::detail::SphericalWarper*>(instance);
	}
	
	void cv_Detail_SphericalWarperGpu_delete(cv::detail::SphericalWarperGpu* instance) {
		delete instance;
	}
	void cv_detail_SphericalWarperGpu_SphericalWarperGpu_float(float scale, Result<cv::detail::SphericalWarperGpu*>* ocvrs_return) {
		try {
			cv::detail::SphericalWarperGpu* ret = new cv::detail::SphericalWarperGpu(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::SphericalWarperGpu*>))
	}
	
	void cv_detail_SphericalWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::detail::SphericalWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, const cv::_OutputArray* xmap, const cv::_OutputArray* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_SphericalWarperGpu_warp_const__InputArrayR_const__InputArrayR_const__InputArrayR_int_int_const__OutputArrayR(cv::detail::SphericalWarperGpu* instance, const cv::_InputArray* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, const cv::_OutputArray* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_detail_SphericalWarperGpu_buildMaps_Size_const__InputArrayR_const__InputArrayR_GpuMatR_GpuMatR(cv::detail::SphericalWarperGpu* instance, cv::Size* src_size, const cv::_InputArray* K, const cv::_InputArray* R, cv::cuda::GpuMat* xmap, cv::cuda::GpuMat* ymap, Result<cv::Rect>* ocvrs_return) {
		try {
			cv::Rect ret = instance->buildMaps(*src_size, *K, *R, *xmap, *ymap);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Rect>))
	}
	
	void cv_detail_SphericalWarperGpu_warp_const_GpuMatR_const__InputArrayR_const__InputArrayR_int_int_GpuMatR(cv::detail::SphericalWarperGpu* instance, const cv::cuda::GpuMat* src, const cv::_InputArray* K, const cv::_InputArray* R, int interp_mode, int border_mode, cv::cuda::GpuMat* dst, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->warp(*src, *K, *R, interp_mode, border_mode, *dst);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_StereographicProjector_to_Detail_ProjectorBase(cv::detail::StereographicProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_StereographicProjector_delete(cv::detail::StereographicProjector* instance) {
		delete instance;
	}
	void cv_detail_StereographicProjector_mapForward_float_float_floatR_floatR(cv::detail::StereographicProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_StereographicProjector_mapBackward_float_float_floatR_floatR(cv::detail::StereographicProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_StereographicWarper_delete(cv::detail::StereographicWarper* instance) {
		delete instance;
	}
	void cv_detail_StereographicWarper_StereographicWarper_float(float scale, Result<cv::detail::StereographicWarper*>* ocvrs_return) {
		try {
			cv::detail::StereographicWarper* ret = new cv::detail::StereographicWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::StereographicWarper*>))
	}
	
	cv::detail::ProjectorBase* cv_Detail_TransverseMercatorProjector_to_Detail_ProjectorBase(cv::detail::TransverseMercatorProjector* instance) {
		return dynamic_cast<cv::detail::ProjectorBase*>(instance);
	}
	
	void cv_Detail_TransverseMercatorProjector_delete(cv::detail::TransverseMercatorProjector* instance) {
		delete instance;
	}
	void cv_detail_TransverseMercatorProjector_mapForward_float_float_floatR_floatR(cv::detail::TransverseMercatorProjector* instance, float x, float y, float* u, float* v, Result_void* ocvrs_return) {
		try {
			instance->mapForward(x, y, *u, *v);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_TransverseMercatorProjector_mapBackward_float_float_floatR_floatR(cv::detail::TransverseMercatorProjector* instance, float u, float v, float* x, float* y, Result_void* ocvrs_return) {
		try {
			instance->mapBackward(u, v, *x, *y);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Detail_TransverseMercatorWarper_delete(cv::detail::TransverseMercatorWarper* instance) {
		delete instance;
	}
	void cv_detail_TransverseMercatorWarper_TransverseMercatorWarper_float(float scale, Result<cv::detail::TransverseMercatorWarper*>* ocvrs_return) {
		try {
			cv::detail::TransverseMercatorWarper* ret = new cv::detail::TransverseMercatorWarper(scale);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::detail::TransverseMercatorWarper*>))
	}
	
	void cv_Detail_VoronoiSeamFinder_delete(cv::detail::VoronoiSeamFinder* instance) {
		delete instance;
	}
	void cv_detail_VoronoiSeamFinder_find_const_vector_UMat_R_const_vector_Point_R_vector_UMat_R(cv::detail::VoronoiSeamFinder* instance, const std::vector<cv::UMat>* src, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*src, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detail_VoronoiSeamFinder_find_const_vector_Size_R_const_vector_Point_R_vector_UMat_R(cv::detail::VoronoiSeamFinder* instance, const std::vector<cv::Size>* size, const std::vector<cv::Point>* corners, std::vector<cv::UMat>* masks, Result_void* ocvrs_return) {
		try {
			instance->find(*size, *corners, *masks);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
}
