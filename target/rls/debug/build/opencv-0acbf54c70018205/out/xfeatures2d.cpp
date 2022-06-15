#include "xfeatures2d.hpp"
#include "xfeatures2d_types.hpp"

extern "C" {
	void cv_xfeatures2d_FASTForPointSet_const__InputArrayR_vector_KeyPoint_R_int_bool_DetectorType(const cv::_InputArray* image, std::vector<cv::KeyPoint>* keypoints, int threshold, bool nonmaxSuppression, cv::FastFeatureDetector::DetectorType type, Result_void* ocvrs_return) {
		try {
			cv::xfeatures2d::FASTForPointSet(*image, *keypoints, threshold, nonmaxSuppression, type);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_matchGMS_const_SizeR_const_SizeR_const_vector_KeyPoint_R_const_vector_KeyPoint_R_const_vector_DMatch_R_vector_DMatch_R_const_bool_const_bool_const_double(const cv::Size* size1, const cv::Size* size2, const std::vector<cv::KeyPoint>* keypoints1, const std::vector<cv::KeyPoint>* keypoints2, const std::vector<cv::DMatch>* matches1to2, std::vector<cv::DMatch>* matchesGMS, const bool withRotation, const bool withScale, const double thresholdFactor, Result_void* ocvrs_return) {
		try {
			cv::xfeatures2d::matchGMS(*size1, *size2, *keypoints1, *keypoints2, *matches1to2, *matchesGMS, withRotation, withScale, thresholdFactor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_matchLOGOS_const_vector_KeyPoint_R_const_vector_KeyPoint_R_const_vector_int_R_const_vector_int_R_vector_DMatch_R(const std::vector<cv::KeyPoint>* keypoints1, const std::vector<cv::KeyPoint>* keypoints2, const std::vector<int>* nn1, const std::vector<int>* nn2, std::vector<cv::DMatch>* matches1to2, Result_void* ocvrs_return) {
		try {
			cv::xfeatures2d::matchLOGOS(*keypoints1, *keypoints2, *nn1, *nn2, *matches1to2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	double cv_cuda_SURF_CUDA_getPropHessianThreshold_const(const cv::cuda::SURF_CUDA* instance) {
			double ret = instance->hessianThreshold;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropHessianThreshold_double(cv::cuda::SURF_CUDA* instance, double val) {
			instance->hessianThreshold = val;
	}
	
	int cv_cuda_SURF_CUDA_getPropNOctaves_const(const cv::cuda::SURF_CUDA* instance) {
			int ret = instance->nOctaves;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropNOctaves_int(cv::cuda::SURF_CUDA* instance, int val) {
			instance->nOctaves = val;
	}
	
	int cv_cuda_SURF_CUDA_getPropNOctaveLayers_const(const cv::cuda::SURF_CUDA* instance) {
			int ret = instance->nOctaveLayers;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropNOctaveLayers_int(cv::cuda::SURF_CUDA* instance, int val) {
			instance->nOctaveLayers = val;
	}
	
	bool cv_cuda_SURF_CUDA_getPropExtended_const(const cv::cuda::SURF_CUDA* instance) {
			bool ret = instance->extended;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropExtended_bool(cv::cuda::SURF_CUDA* instance, bool val) {
			instance->extended = val;
	}
	
	bool cv_cuda_SURF_CUDA_getPropUpright_const(const cv::cuda::SURF_CUDA* instance) {
			bool ret = instance->upright;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropUpright_bool(cv::cuda::SURF_CUDA* instance, bool val) {
			instance->upright = val;
	}
	
	float cv_cuda_SURF_CUDA_getPropKeypointsRatio_const(const cv::cuda::SURF_CUDA* instance) {
			float ret = instance->keypointsRatio;
			return ret;
	}
	
	void cv_cuda_SURF_CUDA_setPropKeypointsRatio_float(cv::cuda::SURF_CUDA* instance, float val) {
			instance->keypointsRatio = val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropSum_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->sum;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropSum_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->sum = *val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropMask1_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->mask1;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropMask1_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->mask1 = *val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropMaskSum_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->maskSum;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropMaskSum_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->maskSum = *val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropDet_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->det;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropDet_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->det = *val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropTrace_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->trace;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropTrace_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->trace = *val;
	}
	
	cv::cuda::GpuMat* cv_cuda_SURF_CUDA_getPropMaxPosBuffer_const(const cv::cuda::SURF_CUDA* instance) {
			cv::cuda::GpuMat ret = instance->maxPosBuffer;
			return new cv::cuda::GpuMat(ret);
	}
	
	void cv_cuda_SURF_CUDA_setPropMaxPosBuffer_GpuMat(cv::cuda::SURF_CUDA* instance, cv::cuda::GpuMat* val) {
			instance->maxPosBuffer = *val;
	}
	
	void cv_SURF_CUDA_delete(cv::cuda::SURF_CUDA* instance) {
		delete instance;
	}
	void cv_cuda_SURF_CUDA_SURF_CUDA(Result<cv::cuda::SURF_CUDA*>* ocvrs_return) {
		try {
			cv::cuda::SURF_CUDA* ret = new cv::cuda::SURF_CUDA();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::cuda::SURF_CUDA*>))
	}
	
	void cv_cuda_SURF_CUDA_SURF_CUDA_double_int_int_bool_float_bool(double _hessianThreshold, int _nOctaves, int _nOctaveLayers, bool _extended, float _keypointsRatio, bool _upright, Result<cv::cuda::SURF_CUDA*>* ocvrs_return) {
		try {
			cv::cuda::SURF_CUDA* ret = new cv::cuda::SURF_CUDA(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, _keypointsRatio, _upright);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::cuda::SURF_CUDA*>))
	}
	
	void cv_cuda_SURF_CUDA_create_double_int_int_bool_float_bool(double _hessianThreshold, int _nOctaves, int _nOctaveLayers, bool _extended, float _keypointsRatio, bool _upright, Result<cv::Ptr<cv::cuda::SURF_CUDA>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::cuda::SURF_CUDA> ret = cv::cuda::SURF_CUDA::create(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, _keypointsRatio, _upright);
			Ok(new cv::Ptr<cv::cuda::SURF_CUDA>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::cuda::SURF_CUDA>*>))
	}
	
	void cv_cuda_SURF_CUDA_descriptorSize_const(const cv::cuda::SURF_CUDA* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->descriptorSize();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_cuda_SURF_CUDA_defaultNorm_const(const cv::cuda::SURF_CUDA* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->defaultNorm();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_cuda_SURF_CUDA_uploadKeypoints_const_vector_KeyPoint_R_GpuMatR(cv::cuda::SURF_CUDA* instance, const std::vector<cv::KeyPoint>* keypoints, cv::cuda::GpuMat* keypointsGPU, Result_void* ocvrs_return) {
		try {
			instance->uploadKeypoints(*keypoints, *keypointsGPU);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_SURF_CUDA_downloadKeypoints_const_GpuMatR_vector_KeyPoint_R(cv::cuda::SURF_CUDA* instance, const cv::cuda::GpuMat* keypointsGPU, std::vector<cv::KeyPoint>* keypoints, Result_void* ocvrs_return) {
		try {
			instance->downloadKeypoints(*keypointsGPU, *keypoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_SURF_CUDA_downloadDescriptors_const_GpuMatR_vector_float_R(cv::cuda::SURF_CUDA* instance, const cv::cuda::GpuMat* descriptorsGPU, std::vector<float>* descriptors, Result_void* ocvrs_return) {
		try {
			instance->downloadDescriptors(*descriptorsGPU, *descriptors);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_SURF_CUDA_detect_const_GpuMatR_const_GpuMatR_GpuMatR(cv::cuda::SURF_CUDA* instance, const cv::cuda::GpuMat* img, const cv::cuda::GpuMat* mask, cv::cuda::GpuMat* keypoints, Result_void* ocvrs_return) {
		try {
			instance->detect(*img, *mask, *keypoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_SURF_CUDA_detectWithDescriptors_const_GpuMatR_const_GpuMatR_GpuMatR_GpuMatR_bool(cv::cuda::SURF_CUDA* instance, const cv::cuda::GpuMat* img, const cv::cuda::GpuMat* mask, cv::cuda::GpuMat* keypoints, cv::cuda::GpuMat* descriptors, bool useProvidedKeypoints, Result_void* ocvrs_return) {
		try {
			instance->detectWithDescriptors(*img, *mask, *keypoints, *descriptors, useProvidedKeypoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_SURF_CUDA_releaseMemory(cv::cuda::SURF_CUDA* instance, Result_void* ocvrs_return) {
		try {
			instance->releaseMemory();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_AffineFeature2D_create_Ptr_Feature2D__Ptr_Feature2D_(cv::Ptr<cv::Feature2D>* keypoint_detector, cv::Ptr<cv::Feature2D>* descriptor_extractor, Result<cv::Ptr<cv::xfeatures2d::AffineFeature2D>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::AffineFeature2D> ret = cv::xfeatures2d::AffineFeature2D::create(*keypoint_detector, *descriptor_extractor);
			Ok(new cv::Ptr<cv::xfeatures2d::AffineFeature2D>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::AffineFeature2D>*>))
	}
	
	void cv_xfeatures2d_AffineFeature2D_create_Ptr_Feature2D_(cv::Ptr<cv::Feature2D>* keypoint_detector, Result<cv::Ptr<cv::xfeatures2d::AffineFeature2D>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::AffineFeature2D> ret = cv::xfeatures2d::AffineFeature2D::create(*keypoint_detector);
			Ok(new cv::Ptr<cv::xfeatures2d::AffineFeature2D>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::AffineFeature2D>*>))
	}
	
	void cv_xfeatures2d_AffineFeature2D_detect_const__InputArrayR_vector_Elliptic_KeyPoint_R_const__InputArrayR(cv::xfeatures2d::AffineFeature2D* instance, const cv::_InputArray* image, std::vector<cv::xfeatures2d::Elliptic_KeyPoint>* keypoints, const cv::_InputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->detect(*image, *keypoints, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_AffineFeature2D_detectAndCompute_const__InputArrayR_const__InputArrayR_vector_Elliptic_KeyPoint_R_const__OutputArrayR_bool(cv::xfeatures2d::AffineFeature2D* instance, const cv::_InputArray* image, const cv::_InputArray* mask, std::vector<cv::xfeatures2d::Elliptic_KeyPoint>* keypoints, const cv::_OutputArray* descriptors, bool useProvidedKeypoints, Result_void* ocvrs_return) {
		try {
			instance->detectAndCompute(*image, *mask, *keypoints, *descriptors, useProvidedKeypoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::Algorithm* cv_BEBLID_to_Algorithm(cv::xfeatures2d::BEBLID* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_BEBLID_to_Feature2D(cv::xfeatures2d::BEBLID* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_BEBLID_delete(cv::xfeatures2d::BEBLID* instance) {
		delete instance;
	}
	void cv_xfeatures2d_BEBLID_create_float_int(float scale_factor, int n_bits, Result<cv::Ptr<cv::xfeatures2d::BEBLID>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::BEBLID> ret = cv::xfeatures2d::BEBLID::create(scale_factor, n_bits);
			Ok(new cv::Ptr<cv::xfeatures2d::BEBLID>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::BEBLID>*>))
	}
	
	void cv_xfeatures2d_BoostDesc_create_int_bool_float(int desc, bool use_scale_orientation, float scale_factor, Result<cv::Ptr<cv::xfeatures2d::BoostDesc>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::BoostDesc> ret = cv::xfeatures2d::BoostDesc::create(desc, use_scale_orientation, scale_factor);
			Ok(new cv::Ptr<cv::xfeatures2d::BoostDesc>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::BoostDesc>*>))
	}
	
	void cv_xfeatures2d_BoostDesc_setUseScaleOrientation_const_bool(cv::xfeatures2d::BoostDesc* instance, const bool use_scale_orientation, Result_void* ocvrs_return) {
		try {
			instance->setUseScaleOrientation(use_scale_orientation);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_BoostDesc_getUseScaleOrientation_const(const cv::xfeatures2d::BoostDesc* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUseScaleOrientation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_BoostDesc_setScaleFactor_const_float(cv::xfeatures2d::BoostDesc* instance, const float scale_factor, Result_void* ocvrs_return) {
		try {
			instance->setScaleFactor(scale_factor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_BoostDesc_getScaleFactor_const(const cv::xfeatures2d::BoostDesc* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScaleFactor();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	cv::Algorithm* cv_BriefDescriptorExtractor_to_Algorithm(cv::xfeatures2d::BriefDescriptorExtractor* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_BriefDescriptorExtractor_to_Feature2D(cv::xfeatures2d::BriefDescriptorExtractor* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_BriefDescriptorExtractor_delete(cv::xfeatures2d::BriefDescriptorExtractor* instance) {
		delete instance;
	}
	void cv_xfeatures2d_BriefDescriptorExtractor_create_int_bool(int bytes, bool use_orientation, Result<cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> ret = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, use_orientation);
			Ok(new cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor>*>))
	}
	
	void cv_xfeatures2d_DAISY_create_float_int_int_int_NormalizationType_const__InputArrayR_bool_bool(float radius, int q_radius, int q_theta, int q_hist, cv::xfeatures2d::DAISY::NormalizationType norm, const cv::_InputArray* H, bool interpolation, bool use_orientation, Result<cv::Ptr<cv::xfeatures2d::DAISY>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::DAISY> ret = cv::xfeatures2d::DAISY::create(radius, q_radius, q_theta, q_hist, norm, *H, interpolation, use_orientation);
			Ok(new cv::Ptr<cv::xfeatures2d::DAISY>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::DAISY>*>))
	}
	
	void cv_xfeatures2d_DAISY_compute_const__InputArrayR_vector_KeyPoint_R_const__OutputArrayR(cv::xfeatures2d::DAISY* instance, const cv::_InputArray* image, std::vector<cv::KeyPoint>* keypoints, const cv::_OutputArray* descriptors, Result_void* ocvrs_return) {
		try {
			instance->compute(*image, *keypoints, *descriptors);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_compute_const__InputArrayR_vector_vector_KeyPoint__R_const__OutputArrayR(cv::xfeatures2d::DAISY* instance, const cv::_InputArray* images, std::vector<std::vector<cv::KeyPoint>>* keypoints, const cv::_OutputArray* descriptors, Result_void* ocvrs_return) {
		try {
			instance->compute(*images, *keypoints, *descriptors);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_compute_const__InputArrayR_Rect_const__OutputArrayR(cv::xfeatures2d::DAISY* instance, const cv::_InputArray* image, cv::Rect* roi, const cv::_OutputArray* descriptors, Result_void* ocvrs_return) {
		try {
			instance->compute(*image, *roi, *descriptors);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_compute_const__InputArrayR_const__OutputArrayR(cv::xfeatures2d::DAISY* instance, const cv::_InputArray* image, const cv::_OutputArray* descriptors, Result_void* ocvrs_return) {
		try {
			instance->compute(*image, *descriptors);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_GetDescriptor_const_double_double_int_floatX(const cv::xfeatures2d::DAISY* instance, double y, double x, int orientation, float* descriptor, Result_void* ocvrs_return) {
		try {
			instance->GetDescriptor(y, x, orientation, descriptor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_GetDescriptor_const_double_double_int_floatX_doubleX(const cv::xfeatures2d::DAISY* instance, double y, double x, int orientation, float* descriptor, double* H, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->GetDescriptor(y, x, orientation, descriptor, H);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_DAISY_GetUnnormalizedDescriptor_const_double_double_int_floatX(const cv::xfeatures2d::DAISY* instance, double y, double x, int orientation, float* descriptor, Result_void* ocvrs_return) {
		try {
			instance->GetUnnormalizedDescriptor(y, x, orientation, descriptor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_DAISY_GetUnnormalizedDescriptor_const_double_double_int_floatX_doubleX(const cv::xfeatures2d::DAISY* instance, double y, double x, int orientation, float* descriptor, double* H, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->GetUnnormalizedDescriptor(y, x, orientation, descriptor, H);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_getPropAxes_const(const cv::xfeatures2d::Elliptic_KeyPoint* instance, cv::Size_<float>* ocvrs_return) {
			cv::Size_<float> ret = instance->axes;
			*ocvrs_return = ret;
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_setPropAxes_Size__float_(cv::xfeatures2d::Elliptic_KeyPoint* instance, cv::Size_<float>* val) {
			instance->axes = *val;
	}
	
	float cv_xfeatures2d_Elliptic_KeyPoint_getPropSi_const(const cv::xfeatures2d::Elliptic_KeyPoint* instance) {
			float ret = instance->si;
			return ret;
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_setPropSi_float(cv::xfeatures2d::Elliptic_KeyPoint* instance, float val) {
			instance->si = val;
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_getPropTransf_const(const cv::xfeatures2d::Elliptic_KeyPoint* instance, cv::Matx23f* ocvrs_return) {
			cv::Matx23f ret = instance->transf;
			*ocvrs_return = ret;
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_setPropTransf_Matx23f(cv::xfeatures2d::Elliptic_KeyPoint* instance, cv::Matx23f* val) {
			instance->transf = *val;
	}
	
	void cv_Elliptic_KeyPoint_delete(cv::xfeatures2d::Elliptic_KeyPoint* instance) {
		delete instance;
	}
	void cv_xfeatures2d_Elliptic_KeyPoint_Elliptic_KeyPoint(Result<cv::xfeatures2d::Elliptic_KeyPoint*>* ocvrs_return) {
		try {
			cv::xfeatures2d::Elliptic_KeyPoint* ret = new cv::xfeatures2d::Elliptic_KeyPoint();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::xfeatures2d::Elliptic_KeyPoint*>))
	}
	
	void cv_xfeatures2d_Elliptic_KeyPoint_Elliptic_KeyPoint_Point2f_float_Size_float_float(cv::Point2f* pt, float angle, cv::Size* axes, float size, float si, Result<cv::xfeatures2d::Elliptic_KeyPoint*>* ocvrs_return) {
		try {
			cv::xfeatures2d::Elliptic_KeyPoint* ret = new cv::xfeatures2d::Elliptic_KeyPoint(*pt, angle, *axes, size, si);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::xfeatures2d::Elliptic_KeyPoint*>))
	}
	
	cv::Algorithm* cv_FREAK_to_Algorithm(cv::xfeatures2d::FREAK* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_FREAK_to_Feature2D(cv::xfeatures2d::FREAK* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_FREAK_delete(cv::xfeatures2d::FREAK* instance) {
		delete instance;
	}
	void cv_xfeatures2d_FREAK_create_bool_bool_float_int_const_vector_int_R(bool orientationNormalized, bool scaleNormalized, float patternScale, int nOctaves, const std::vector<int>* selectedPairs, Result<cv::Ptr<cv::xfeatures2d::FREAK>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::FREAK> ret = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves, *selectedPairs);
			Ok(new cv::Ptr<cv::xfeatures2d::FREAK>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::FREAK>*>))
	}
	
	cv::Algorithm* cv_HarrisLaplaceFeatureDetector_to_Algorithm(cv::xfeatures2d::HarrisLaplaceFeatureDetector* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_HarrisLaplaceFeatureDetector_to_Feature2D(cv::xfeatures2d::HarrisLaplaceFeatureDetector* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_HarrisLaplaceFeatureDetector_delete(cv::xfeatures2d::HarrisLaplaceFeatureDetector* instance) {
		delete instance;
	}
	void cv_xfeatures2d_HarrisLaplaceFeatureDetector_create_int_float_float_int_int(int numOctaves, float corn_thresh, float DOG_thresh, int maxCorners, int num_layers, Result<cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector> ret = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create(numOctaves, corn_thresh, DOG_thresh, maxCorners, num_layers);
			Ok(new cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector>*>))
	}
	
	cv::Algorithm* cv_LATCH_to_Algorithm(cv::xfeatures2d::LATCH* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_LATCH_to_Feature2D(cv::xfeatures2d::LATCH* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_LATCH_delete(cv::xfeatures2d::LATCH* instance) {
		delete instance;
	}
	void cv_xfeatures2d_LATCH_create_int_bool_int_double(int bytes, bool rotationInvariance, int half_ssd_size, double sigma, Result<cv::Ptr<cv::xfeatures2d::LATCH>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::LATCH> ret = cv::xfeatures2d::LATCH::create(bytes, rotationInvariance, half_ssd_size, sigma);
			Ok(new cv::Ptr<cv::xfeatures2d::LATCH>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::LATCH>*>))
	}
	
	cv::Algorithm* cv_LUCID_to_Algorithm(cv::xfeatures2d::LUCID* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_LUCID_to_Feature2D(cv::xfeatures2d::LUCID* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_LUCID_delete(cv::xfeatures2d::LUCID* instance) {
		delete instance;
	}
	void cv_xfeatures2d_LUCID_create_const_int_const_int(const int lucid_kernel, const int blur_kernel, Result<cv::Ptr<cv::xfeatures2d::LUCID>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::LUCID> ret = cv::xfeatures2d::LUCID::create(lucid_kernel, blur_kernel);
			Ok(new cv::Ptr<cv::xfeatures2d::LUCID>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::LUCID>*>))
	}
	
	cv::Algorithm* cv_MSDDetector_to_Algorithm(cv::xfeatures2d::MSDDetector* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_MSDDetector_to_Feature2D(cv::xfeatures2d::MSDDetector* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_MSDDetector_delete(cv::xfeatures2d::MSDDetector* instance) {
		delete instance;
	}
	void cv_xfeatures2d_MSDDetector_create_int_int_int_int_float_int_float_int_bool(int m_patch_radius, int m_search_area_radius, int m_nms_radius, int m_nms_scale_radius, float m_th_saliency, int m_kNN, float m_scale_factor, int m_n_scales, bool m_compute_orientation, Result<cv::Ptr<cv::xfeatures2d::MSDDetector>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::MSDDetector> ret = cv::xfeatures2d::MSDDetector::create(m_patch_radius, m_search_area_radius, m_nms_radius, m_nms_scale_radius, m_th_saliency, m_kNN, m_scale_factor, m_n_scales, m_compute_orientation);
			Ok(new cv::Ptr<cv::xfeatures2d::MSDDetector>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::MSDDetector>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_create_const_int_const_int_const_int(const int initSampleCount, const int initSeedCount, const int pointDistribution, Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::PCTSignatures> ret = cv::xfeatures2d::PCTSignatures::create(initSampleCount, initSeedCount, pointDistribution);
			Ok(new cv::Ptr<cv::xfeatures2d::PCTSignatures>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_create_const_vector_Point2f_R_const_int(const std::vector<cv::Point2f>* initSamplingPoints, const int initSeedCount, Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::PCTSignatures> ret = cv::xfeatures2d::PCTSignatures::create(*initSamplingPoints, initSeedCount);
			Ok(new cv::Ptr<cv::xfeatures2d::PCTSignatures>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_create_const_vector_Point2f_R_const_vector_int_R(const std::vector<cv::Point2f>* initSamplingPoints, const std::vector<int>* initClusterSeedIndexes, Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::PCTSignatures> ret = cv::xfeatures2d::PCTSignatures::create(*initSamplingPoints, *initClusterSeedIndexes);
			Ok(new cv::Ptr<cv::xfeatures2d::PCTSignatures>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::PCTSignatures>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_computeSignature_const_const__InputArrayR_const__OutputArrayR(const cv::xfeatures2d::PCTSignatures* instance, const cv::_InputArray* image, const cv::_OutputArray* signature, Result_void* ocvrs_return) {
		try {
			instance->computeSignature(*image, *signature);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_computeSignatures_const_const_vector_Mat_R_vector_Mat_R(const cv::xfeatures2d::PCTSignatures* instance, const std::vector<cv::Mat>* images, std::vector<cv::Mat>* signatures, Result_void* ocvrs_return) {
		try {
			instance->computeSignatures(*images, *signatures);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_drawSignature_const__InputArrayR_const__InputArrayR_const__OutputArrayR_float_int(const cv::_InputArray* source, const cv::_InputArray* signature, const cv::_OutputArray* result, float radiusToShorterSideRatio, int borderThickness, Result_void* ocvrs_return) {
		try {
			cv::xfeatures2d::PCTSignatures::drawSignature(*source, *signature, *result, radiusToShorterSideRatio, borderThickness);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_generateInitPoints_vector_Point2f_R_const_int_int(std::vector<cv::Point2f>* initPoints, const int count, int pointDistribution, Result_void* ocvrs_return) {
		try {
			cv::xfeatures2d::PCTSignatures::generateInitPoints(*initPoints, count, pointDistribution);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getSampleCount_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getSampleCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_getGrayscaleBits_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getGrayscaleBits();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setGrayscaleBits_int(cv::xfeatures2d::PCTSignatures* instance, int grayscaleBits, Result_void* ocvrs_return) {
		try {
			instance->setGrayscaleBits(grayscaleBits);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWindowRadius_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getWindowRadius();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWindowRadius_int(cv::xfeatures2d::PCTSignatures* instance, int radius, Result_void* ocvrs_return) {
		try {
			instance->setWindowRadius(radius);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightX_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightX();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightX_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightX(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightY_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightY();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightY_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightY(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightL_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightL();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightL_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightL(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightA_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightA();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightA_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightA(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightB_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightB();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightB_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightB(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightContrast_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightContrast();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightContrast_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightContrast(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getWeightEntropy_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getWeightEntropy();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeightEntropy_float(cv::xfeatures2d::PCTSignatures* instance, float weight, Result_void* ocvrs_return) {
		try {
			instance->setWeightEntropy(weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getSamplingPoints_const(const cv::xfeatures2d::PCTSignatures* instance, Result<std::vector<cv::Point2f>*>* ocvrs_return) {
		try {
			std::vector<cv::Point2f> ret = instance->getSamplingPoints();
			Ok(new std::vector<cv::Point2f>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Point2f>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeight_int_float(cv::xfeatures2d::PCTSignatures* instance, int idx, float value, Result_void* ocvrs_return) {
		try {
			instance->setWeight(idx, value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_setWeights_const_vector_float_R(cv::xfeatures2d::PCTSignatures* instance, const std::vector<float>* weights, Result_void* ocvrs_return) {
		try {
			instance->setWeights(*weights);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_setTranslation_int_float(cv::xfeatures2d::PCTSignatures* instance, int idx, float value, Result_void* ocvrs_return) {
		try {
			instance->setTranslation(idx, value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_setTranslations_const_vector_float_R(cv::xfeatures2d::PCTSignatures* instance, const std::vector<float>* translations, Result_void* ocvrs_return) {
		try {
			instance->setTranslations(*translations);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_setSamplingPoints_vector_Point2f_(cv::xfeatures2d::PCTSignatures* instance, std::vector<cv::Point2f>* samplingPoints, Result_void* ocvrs_return) {
		try {
			instance->setSamplingPoints(*samplingPoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getInitSeedIndexes_const(const cv::xfeatures2d::PCTSignatures* instance, Result<std::vector<int>*>* ocvrs_return) {
		try {
			std::vector<int> ret = instance->getInitSeedIndexes();
			Ok(new std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setInitSeedIndexes_vector_int_(cv::xfeatures2d::PCTSignatures* instance, std::vector<int>* initSeedIndexes, Result_void* ocvrs_return) {
		try {
			instance->setInitSeedIndexes(*initSeedIndexes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getInitSeedCount_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getInitSeedCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_getIterationCount_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getIterationCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setIterationCount_int(cv::xfeatures2d::PCTSignatures* instance, int iterationCount, Result_void* ocvrs_return) {
		try {
			instance->setIterationCount(iterationCount);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getMaxClustersCount_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxClustersCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setMaxClustersCount_int(cv::xfeatures2d::PCTSignatures* instance, int maxClustersCount, Result_void* ocvrs_return) {
		try {
			instance->setMaxClustersCount(maxClustersCount);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getClusterMinSize_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getClusterMinSize();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setClusterMinSize_int(cv::xfeatures2d::PCTSignatures* instance, int clusterMinSize, Result_void* ocvrs_return) {
		try {
			instance->setClusterMinSize(clusterMinSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getJoiningDistance_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getJoiningDistance();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setJoiningDistance_float(cv::xfeatures2d::PCTSignatures* instance, float joiningDistance, Result_void* ocvrs_return) {
		try {
			instance->setJoiningDistance(joiningDistance);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getDropThreshold_const(const cv::xfeatures2d::PCTSignatures* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getDropThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setDropThreshold_float(cv::xfeatures2d::PCTSignatures* instance, float dropThreshold, Result_void* ocvrs_return) {
		try {
			instance->setDropThreshold(dropThreshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignatures_getDistanceFunction_const(const cv::xfeatures2d::PCTSignatures* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getDistanceFunction();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_PCTSignatures_setDistanceFunction_int(cv::xfeatures2d::PCTSignatures* instance, int distanceFunction, Result_void* ocvrs_return) {
		try {
			instance->setDistanceFunction(distanceFunction);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_PCTSignaturesSQFD_create_const_int_const_int_const_float(const int distanceFunction, const int similarityFunction, const float similarityParameter, Result<cv::Ptr<cv::xfeatures2d::PCTSignaturesSQFD>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::PCTSignaturesSQFD> ret = cv::xfeatures2d::PCTSignaturesSQFD::create(distanceFunction, similarityFunction, similarityParameter);
			Ok(new cv::Ptr<cv::xfeatures2d::PCTSignaturesSQFD>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::PCTSignaturesSQFD>*>))
	}
	
	void cv_xfeatures2d_PCTSignaturesSQFD_computeQuadraticFormDistance_const_const__InputArrayR_const__InputArrayR(const cv::xfeatures2d::PCTSignaturesSQFD* instance, const cv::_InputArray* _signature0, const cv::_InputArray* _signature1, Result<float>* ocvrs_return) {
		try {
			float ret = instance->computeQuadraticFormDistance(*_signature0, *_signature1);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_PCTSignaturesSQFD_computeQuadraticFormDistances_const_const_MatR_const_vector_Mat_R_vector_float_R(const cv::xfeatures2d::PCTSignaturesSQFD* instance, const cv::Mat* sourceSignature, const std::vector<cv::Mat>* imageSignatures, std::vector<float>* distances, Result_void* ocvrs_return) {
		try {
			instance->computeQuadraticFormDistances(*sourceSignature, *imageSignatures, *distances);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_create_double_int_int_bool_bool(double hessianThreshold, int nOctaves, int nOctaveLayers, bool extended, bool upright, Result<cv::Ptr<cv::xfeatures2d::SURF>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::SURF> ret = cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
			Ok(new cv::Ptr<cv::xfeatures2d::SURF>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::SURF>*>))
	}
	
	void cv_xfeatures2d_SURF_setHessianThreshold_double(cv::xfeatures2d::SURF* instance, double hessianThreshold, Result_void* ocvrs_return) {
		try {
			instance->setHessianThreshold(hessianThreshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_getHessianThreshold_const(const cv::xfeatures2d::SURF* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getHessianThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_xfeatures2d_SURF_setNOctaves_int(cv::xfeatures2d::SURF* instance, int nOctaves, Result_void* ocvrs_return) {
		try {
			instance->setNOctaves(nOctaves);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_getNOctaves_const(const cv::xfeatures2d::SURF* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNOctaves();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_SURF_setNOctaveLayers_int(cv::xfeatures2d::SURF* instance, int nOctaveLayers, Result_void* ocvrs_return) {
		try {
			instance->setNOctaveLayers(nOctaveLayers);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_getNOctaveLayers_const(const cv::xfeatures2d::SURF* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNOctaveLayers();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_SURF_setExtended_bool(cv::xfeatures2d::SURF* instance, bool extended, Result_void* ocvrs_return) {
		try {
			instance->setExtended(extended);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_getExtended_const(const cv::xfeatures2d::SURF* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getExtended();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_SURF_setUpright_bool(cv::xfeatures2d::SURF* instance, bool upright, Result_void* ocvrs_return) {
		try {
			instance->setUpright(upright);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_SURF_getUpright_const(const cv::xfeatures2d::SURF* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUpright();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	cv::Algorithm* cv_StarDetector_to_Algorithm(cv::xfeatures2d::StarDetector* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::Feature2D* cv_StarDetector_to_Feature2D(cv::xfeatures2d::StarDetector* instance) {
		return dynamic_cast<cv::Feature2D*>(instance);
	}
	
	void cv_StarDetector_delete(cv::xfeatures2d::StarDetector* instance) {
		delete instance;
	}
	void cv_xfeatures2d_StarDetector_create_int_int_int_int_int(int maxSize, int responseThreshold, int lineThresholdProjected, int lineThresholdBinarized, int suppressNonmaxSize, Result<cv::Ptr<cv::xfeatures2d::StarDetector>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::StarDetector> ret = cv::xfeatures2d::StarDetector::create(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);
			Ok(new cv::Ptr<cv::xfeatures2d::StarDetector>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::StarDetector>*>))
	}
	
	void cv_xfeatures2d_TBMR_create_int_float_float_int(int min_area, float max_area_relative, float scale_factor, int n_scales, Result<cv::Ptr<cv::xfeatures2d::TBMR>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::TBMR> ret = cv::xfeatures2d::TBMR::create(min_area, max_area_relative, scale_factor, n_scales);
			Ok(new cv::Ptr<cv::xfeatures2d::TBMR>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::TBMR>*>))
	}
	
	void cv_xfeatures2d_TBMR_setMinArea_int(cv::xfeatures2d::TBMR* instance, int minArea, Result_void* ocvrs_return) {
		try {
			instance->setMinArea(minArea);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_TBMR_getMinArea_const(const cv::xfeatures2d::TBMR* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMinArea();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_TBMR_setMaxAreaRelative_float(cv::xfeatures2d::TBMR* instance, float maxArea, Result_void* ocvrs_return) {
		try {
			instance->setMaxAreaRelative(maxArea);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_TBMR_getMaxAreaRelative_const(const cv::xfeatures2d::TBMR* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getMaxAreaRelative();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_TBMR_setScaleFactor_float(cv::xfeatures2d::TBMR* instance, float scale_factor, Result_void* ocvrs_return) {
		try {
			instance->setScaleFactor(scale_factor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_TBMR_getScaleFactor_const(const cv::xfeatures2d::TBMR* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScaleFactor();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_TBMR_setNScales_int(cv::xfeatures2d::TBMR* instance, int n_scales, Result_void* ocvrs_return) {
		try {
			instance->setNScales(n_scales);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_TBMR_getNScales_const(const cv::xfeatures2d::TBMR* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNScales();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_xfeatures2d_VGG_create_int_float_bool_bool_float_bool(int desc, float isigma, bool img_normalize, bool use_scale_orientation, float scale_factor, bool dsc_normalize, Result<cv::Ptr<cv::xfeatures2d::VGG>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::xfeatures2d::VGG> ret = cv::xfeatures2d::VGG::create(desc, isigma, img_normalize, use_scale_orientation, scale_factor, dsc_normalize);
			Ok(new cv::Ptr<cv::xfeatures2d::VGG>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::xfeatures2d::VGG>*>))
	}
	
	void cv_xfeatures2d_VGG_setSigma_const_float(cv::xfeatures2d::VGG* instance, const float isigma, Result_void* ocvrs_return) {
		try {
			instance->setSigma(isigma);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_VGG_getSigma_const(const cv::xfeatures2d::VGG* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getSigma();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_VGG_setUseNormalizeImage_const_bool(cv::xfeatures2d::VGG* instance, const bool img_normalize, Result_void* ocvrs_return) {
		try {
			instance->setUseNormalizeImage(img_normalize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_VGG_getUseNormalizeImage_const(const cv::xfeatures2d::VGG* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUseNormalizeImage();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_VGG_setUseScaleOrientation_const_bool(cv::xfeatures2d::VGG* instance, const bool use_scale_orientation, Result_void* ocvrs_return) {
		try {
			instance->setUseScaleOrientation(use_scale_orientation);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_VGG_getUseScaleOrientation_const(const cv::xfeatures2d::VGG* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUseScaleOrientation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_xfeatures2d_VGG_setScaleFactor_const_float(cv::xfeatures2d::VGG* instance, const float scale_factor, Result_void* ocvrs_return) {
		try {
			instance->setScaleFactor(scale_factor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_VGG_getScaleFactor_const(const cv::xfeatures2d::VGG* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScaleFactor();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_xfeatures2d_VGG_setUseNormalizeDescriptor_const_bool(cv::xfeatures2d::VGG* instance, const bool dsc_normalize, Result_void* ocvrs_return) {
		try {
			instance->setUseNormalizeDescriptor(dsc_normalize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_xfeatures2d_VGG_getUseNormalizeDescriptor_const(const cv::xfeatures2d::VGG* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUseNormalizeDescriptor();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
}
