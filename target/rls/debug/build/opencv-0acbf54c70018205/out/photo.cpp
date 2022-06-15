#include "photo.hpp"
#include "photo_types.hpp"

extern "C" {
	void cv_colorChange_const__InputArrayR_const__InputArrayR_const__OutputArrayR_float_float_float(const cv::_InputArray* src, const cv::_InputArray* mask, const cv::_OutputArray* dst, float red_mul, float green_mul, float blue_mul, Result_void* ocvrs_return) {
		try {
			cv::colorChange(*src, *mask, *dst, red_mul, green_mul, blue_mul);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_createAlignMTB_int_int_bool(int max_bits, int exclude_range, bool cut, Result<cv::Ptr<cv::AlignMTB>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::AlignMTB> ret = cv::createAlignMTB(max_bits, exclude_range, cut);
			Ok(new cv::Ptr<cv::AlignMTB>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::AlignMTB>*>))
	}
	
	void cv_createCalibrateDebevec_int_float_bool(int samples, float lambda, bool random, Result<cv::Ptr<cv::CalibrateDebevec>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::CalibrateDebevec> ret = cv::createCalibrateDebevec(samples, lambda, random);
			Ok(new cv::Ptr<cv::CalibrateDebevec>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::CalibrateDebevec>*>))
	}
	
	void cv_createCalibrateRobertson_int_float(int max_iter, float threshold, Result<cv::Ptr<cv::CalibrateRobertson>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::CalibrateRobertson> ret = cv::createCalibrateRobertson(max_iter, threshold);
			Ok(new cv::Ptr<cv::CalibrateRobertson>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::CalibrateRobertson>*>))
	}
	
	void cv_createMergeDebevec(Result<cv::Ptr<cv::MergeDebevec>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::MergeDebevec> ret = cv::createMergeDebevec();
			Ok(new cv::Ptr<cv::MergeDebevec>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::MergeDebevec>*>))
	}
	
	void cv_createMergeMertens_float_float_float(float contrast_weight, float saturation_weight, float exposure_weight, Result<cv::Ptr<cv::MergeMertens>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::MergeMertens> ret = cv::createMergeMertens(contrast_weight, saturation_weight, exposure_weight);
			Ok(new cv::Ptr<cv::MergeMertens>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::MergeMertens>*>))
	}
	
	void cv_createMergeRobertson(Result<cv::Ptr<cv::MergeRobertson>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::MergeRobertson> ret = cv::createMergeRobertson();
			Ok(new cv::Ptr<cv::MergeRobertson>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::MergeRobertson>*>))
	}
	
	void cv_createTonemapDrago_float_float_float(float gamma, float saturation, float bias, Result<cv::Ptr<cv::TonemapDrago>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::TonemapDrago> ret = cv::createTonemapDrago(gamma, saturation, bias);
			Ok(new cv::Ptr<cv::TonemapDrago>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::TonemapDrago>*>))
	}
	
	void cv_createTonemapMantiuk_float_float_float(float gamma, float scale, float saturation, Result<cv::Ptr<cv::TonemapMantiuk>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::TonemapMantiuk> ret = cv::createTonemapMantiuk(gamma, scale, saturation);
			Ok(new cv::Ptr<cv::TonemapMantiuk>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::TonemapMantiuk>*>))
	}
	
	void cv_createTonemapReinhard_float_float_float_float(float gamma, float intensity, float light_adapt, float color_adapt, Result<cv::Ptr<cv::TonemapReinhard>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::TonemapReinhard> ret = cv::createTonemapReinhard(gamma, intensity, light_adapt, color_adapt);
			Ok(new cv::Ptr<cv::TonemapReinhard>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::TonemapReinhard>*>))
	}
	
	void cv_createTonemap_float(float gamma, Result<cv::Ptr<cv::Tonemap>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::Tonemap> ret = cv::createTonemap(gamma);
			Ok(new cv::Ptr<cv::Tonemap>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::Tonemap>*>))
	}
	
	void cv_cuda_fastNlMeansDenoisingColored_const__InputArrayR_const__OutputArrayR_float_float_int_int_StreamR(const cv::_InputArray* src, const cv::_OutputArray* dst, float h_luminance, float photo_render, int search_window, int block_size, cv::cuda::Stream* stream, Result_void* ocvrs_return) {
		try {
			cv::cuda::fastNlMeansDenoisingColored(*src, *dst, h_luminance, photo_render, search_window, block_size, *stream);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_fastNlMeansDenoising_const__InputArrayR_const__OutputArrayR_float_int_int_StreamR(const cv::_InputArray* src, const cv::_OutputArray* dst, float h, int search_window, int block_size, cv::cuda::Stream* stream, Result_void* ocvrs_return) {
		try {
			cv::cuda::fastNlMeansDenoising(*src, *dst, h, search_window, block_size, *stream);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_cuda_nonLocalMeans_const__InputArrayR_const__OutputArrayR_float_int_int_int_StreamR(const cv::_InputArray* src, const cv::_OutputArray* dst, float h, int search_window, int block_size, int borderMode, cv::cuda::Stream* stream, Result_void* ocvrs_return) {
		try {
			cv::cuda::nonLocalMeans(*src, *dst, h, search_window, block_size, borderMode, *stream);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_decolor_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(const cv::_InputArray* src, const cv::_OutputArray* grayscale, const cv::_OutputArray* color_boost, Result_void* ocvrs_return) {
		try {
			cv::decolor(*src, *grayscale, *color_boost);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_denoise_TVL1_const_vector_Mat_R_MatR_double_int(const std::vector<cv::Mat>* observations, cv::Mat* result, double lambda, int niters, Result_void* ocvrs_return) {
		try {
			cv::denoise_TVL1(*observations, *result, lambda, niters);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_detailEnhance_const__InputArrayR_const__OutputArrayR_float_float(const cv::_InputArray* src, const cv::_OutputArray* dst, float sigma_s, float sigma_r, Result_void* ocvrs_return) {
		try {
			cv::detailEnhance(*src, *dst, sigma_s, sigma_r);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_edgePreservingFilter_const__InputArrayR_const__OutputArrayR_int_float_float(const cv::_InputArray* src, const cv::_OutputArray* dst, int flags, float sigma_s, float sigma_r, Result_void* ocvrs_return) {
		try {
			cv::edgePreservingFilter(*src, *dst, flags, sigma_s, sigma_r);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoisingColoredMulti_const__InputArrayR_const__OutputArrayR_int_int_float_float_int_int(const cv::_InputArray* srcImgs, const cv::_OutputArray* dst, int imgToDenoiseIndex, int temporalWindowSize, float h, float hColor, int templateWindowSize, int searchWindowSize, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoisingColoredMulti(*srcImgs, *dst, imgToDenoiseIndex, temporalWindowSize, h, hColor, templateWindowSize, searchWindowSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoisingColored_const__InputArrayR_const__OutputArrayR_float_float_int_int(const cv::_InputArray* src, const cv::_OutputArray* dst, float h, float hColor, int templateWindowSize, int searchWindowSize, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoisingColored(*src, *dst, h, hColor, templateWindowSize, searchWindowSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoisingMulti_const__InputArrayR_const__OutputArrayR_int_int_const_vector_float_R_int_int_int(const cv::_InputArray* srcImgs, const cv::_OutputArray* dst, int imgToDenoiseIndex, int temporalWindowSize, const std::vector<float>* h, int templateWindowSize, int searchWindowSize, int normType, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoisingMulti(*srcImgs, *dst, imgToDenoiseIndex, temporalWindowSize, *h, templateWindowSize, searchWindowSize, normType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoisingMulti_const__InputArrayR_const__OutputArrayR_int_int_float_int_int(const cv::_InputArray* srcImgs, const cv::_OutputArray* dst, int imgToDenoiseIndex, int temporalWindowSize, float h, int templateWindowSize, int searchWindowSize, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoisingMulti(*srcImgs, *dst, imgToDenoiseIndex, temporalWindowSize, h, templateWindowSize, searchWindowSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoising_const__InputArrayR_const__OutputArrayR_const_vector_float_R_int_int_int(const cv::_InputArray* src, const cv::_OutputArray* dst, const std::vector<float>* h, int templateWindowSize, int searchWindowSize, int normType, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoising(*src, *dst, *h, templateWindowSize, searchWindowSize, normType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_fastNlMeansDenoising_const__InputArrayR_const__OutputArrayR_float_int_int(const cv::_InputArray* src, const cv::_OutputArray* dst, float h, int templateWindowSize, int searchWindowSize, Result_void* ocvrs_return) {
		try {
			cv::fastNlMeansDenoising(*src, *dst, h, templateWindowSize, searchWindowSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_illuminationChange_const__InputArrayR_const__InputArrayR_const__OutputArrayR_float_float(const cv::_InputArray* src, const cv::_InputArray* mask, const cv::_OutputArray* dst, float alpha, float beta, Result_void* ocvrs_return) {
		try {
			cv::illuminationChange(*src, *mask, *dst, alpha, beta);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_inpaint_const__InputArrayR_const__InputArrayR_const__OutputArrayR_double_int(const cv::_InputArray* src, const cv::_InputArray* inpaintMask, const cv::_OutputArray* dst, double inpaintRadius, int flags, Result_void* ocvrs_return) {
		try {
			cv::inpaint(*src, *inpaintMask, *dst, inpaintRadius, flags);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_pencilSketch_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_float_float_float(const cv::_InputArray* src, const cv::_OutputArray* dst1, const cv::_OutputArray* dst2, float sigma_s, float sigma_r, float shade_factor, Result_void* ocvrs_return) {
		try {
			cv::pencilSketch(*src, *dst1, *dst2, sigma_s, sigma_r, shade_factor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_seamlessClone_const__InputArrayR_const__InputArrayR_const__InputArrayR_Point_const__OutputArrayR_int(const cv::_InputArray* src, const cv::_InputArray* dst, const cv::_InputArray* mask, cv::Point* p, const cv::_OutputArray* blend, int flags, Result_void* ocvrs_return) {
		try {
			cv::seamlessClone(*src, *dst, *mask, *p, *blend, flags);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_stylization_const__InputArrayR_const__OutputArrayR_float_float(const cv::_InputArray* src, const cv::_OutputArray* dst, float sigma_s, float sigma_r, Result_void* ocvrs_return) {
		try {
			cv::stylization(*src, *dst, sigma_s, sigma_r);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_textureFlattening_const__InputArrayR_const__InputArrayR_const__OutputArrayR_float_float_int(const cv::_InputArray* src, const cv::_InputArray* mask, const cv::_OutputArray* dst, float low_threshold, float high_threshold, int kernel_size, Result_void* ocvrs_return) {
		try {
			cv::textureFlattening(*src, *mask, *dst, low_threshold, high_threshold, kernel_size);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignExposures_process_const__InputArrayR_vector_Mat_R_const__InputArrayR_const__InputArrayR(cv::AlignExposures* instance, const cv::_InputArray* src, std::vector<cv::Mat>* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_process_const__InputArrayR_vector_Mat_R_const__InputArrayR_const__InputArrayR(cv::AlignMTB* instance, const cv::_InputArray* src, std::vector<cv::Mat>* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_process_const__InputArrayR_vector_Mat_R(cv::AlignMTB* instance, const cv::_InputArray* src, std::vector<cv::Mat>* dst, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_calculateShift_const__InputArrayR_const__InputArrayR(cv::AlignMTB* instance, const cv::_InputArray* img0, const cv::_InputArray* img1, Result<cv::Point>* ocvrs_return) {
		try {
			cv::Point ret = instance->calculateShift(*img0, *img1);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Point>))
	}
	
	void cv_AlignMTB_shiftMat_const__InputArrayR_const__OutputArrayR_const_Point(cv::AlignMTB* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::Point* shift, Result_void* ocvrs_return) {
		try {
			instance->shiftMat(*src, *dst, *shift);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_computeBitmaps_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::AlignMTB* instance, const cv::_InputArray* img, const cv::_OutputArray* tb, const cv::_OutputArray* eb, Result_void* ocvrs_return) {
		try {
			instance->computeBitmaps(*img, *tb, *eb);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_getMaxBits_const(const cv::AlignMTB* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxBits();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_AlignMTB_setMaxBits_int(cv::AlignMTB* instance, int max_bits, Result_void* ocvrs_return) {
		try {
			instance->setMaxBits(max_bits);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_getExcludeRange_const(const cv::AlignMTB* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getExcludeRange();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_AlignMTB_setExcludeRange_int(cv::AlignMTB* instance, int exclude_range, Result_void* ocvrs_return) {
		try {
			instance->setExcludeRange(exclude_range);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AlignMTB_getCut_const(const cv::AlignMTB* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getCut();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_AlignMTB_setCut_bool(cv::AlignMTB* instance, bool value, Result_void* ocvrs_return) {
		try {
			instance->setCut(value);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateCRF_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR(cv::CalibrateCRF* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateDebevec_getLambda_const(const cv::CalibrateDebevec* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getLambda();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_CalibrateDebevec_setLambda_float(cv::CalibrateDebevec* instance, float lambda, Result_void* ocvrs_return) {
		try {
			instance->setLambda(lambda);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateDebevec_getSamples_const(const cv::CalibrateDebevec* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getSamples();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_CalibrateDebevec_setSamples_int(cv::CalibrateDebevec* instance, int samples, Result_void* ocvrs_return) {
		try {
			instance->setSamples(samples);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateDebevec_getRandom_const(const cv::CalibrateDebevec* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getRandom();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_CalibrateDebevec_setRandom_bool(cv::CalibrateDebevec* instance, bool random, Result_void* ocvrs_return) {
		try {
			instance->setRandom(random);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateRobertson_getMaxIter_const(const cv::CalibrateRobertson* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxIter();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_CalibrateRobertson_setMaxIter_int(cv::CalibrateRobertson* instance, int max_iter, Result_void* ocvrs_return) {
		try {
			instance->setMaxIter(max_iter);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateRobertson_getThreshold_const(const cv::CalibrateRobertson* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_CalibrateRobertson_setThreshold_float(cv::CalibrateRobertson* instance, float threshold, Result_void* ocvrs_return) {
		try {
			instance->setThreshold(threshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_CalibrateRobertson_getRadiance_const(const cv::CalibrateRobertson* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getRadiance();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_MergeDebevec_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR_const__InputArrayR(cv::MergeDebevec* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeDebevec_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR(cv::MergeDebevec* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeExposures_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR_const__InputArrayR(cv::MergeExposures* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeMertens_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR_const__InputArrayR(cv::MergeMertens* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeMertens_process_const__InputArrayR_const__OutputArrayR(cv::MergeMertens* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeMertens_getContrastWeight_const(const cv::MergeMertens* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getContrastWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_MergeMertens_setContrastWeight_float(cv::MergeMertens* instance, float contrast_weiht, Result_void* ocvrs_return) {
		try {
			instance->setContrastWeight(contrast_weiht);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeMertens_getSaturationWeight_const(const cv::MergeMertens* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getSaturationWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_MergeMertens_setSaturationWeight_float(cv::MergeMertens* instance, float saturation_weight, Result_void* ocvrs_return) {
		try {
			instance->setSaturationWeight(saturation_weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeMertens_getExposureWeight_const(const cv::MergeMertens* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getExposureWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_MergeMertens_setExposureWeight_float(cv::MergeMertens* instance, float exposure_weight, Result_void* ocvrs_return) {
		try {
			instance->setExposureWeight(exposure_weight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeRobertson_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR_const__InputArrayR(cv::MergeRobertson* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, const cv::_InputArray* response, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times, *response);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_MergeRobertson_process_const__InputArrayR_const__OutputArrayR_const__InputArrayR(cv::MergeRobertson* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, const cv::_InputArray* times, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst, *times);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Tonemap_process_const__InputArrayR_const__OutputArrayR(cv::Tonemap* instance, const cv::_InputArray* src, const cv::_OutputArray* dst, Result_void* ocvrs_return) {
		try {
			instance->process(*src, *dst);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_Tonemap_getGamma_const(const cv::Tonemap* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getGamma();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_Tonemap_setGamma_float(cv::Tonemap* instance, float gamma, Result_void* ocvrs_return) {
		try {
			instance->setGamma(gamma);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapDrago_getSaturation_const(const cv::TonemapDrago* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getSaturation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapDrago_setSaturation_float(cv::TonemapDrago* instance, float saturation, Result_void* ocvrs_return) {
		try {
			instance->setSaturation(saturation);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapDrago_getBias_const(const cv::TonemapDrago* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getBias();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapDrago_setBias_float(cv::TonemapDrago* instance, float bias, Result_void* ocvrs_return) {
		try {
			instance->setBias(bias);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapMantiuk_getScale_const(const cv::TonemapMantiuk* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapMantiuk_setScale_float(cv::TonemapMantiuk* instance, float scale, Result_void* ocvrs_return) {
		try {
			instance->setScale(scale);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapMantiuk_getSaturation_const(const cv::TonemapMantiuk* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getSaturation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapMantiuk_setSaturation_float(cv::TonemapMantiuk* instance, float saturation, Result_void* ocvrs_return) {
		try {
			instance->setSaturation(saturation);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapReinhard_getIntensity_const(const cv::TonemapReinhard* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getIntensity();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapReinhard_setIntensity_float(cv::TonemapReinhard* instance, float intensity, Result_void* ocvrs_return) {
		try {
			instance->setIntensity(intensity);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapReinhard_getLightAdaptation_const(const cv::TonemapReinhard* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getLightAdaptation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapReinhard_setLightAdaptation_float(cv::TonemapReinhard* instance, float light_adapt, Result_void* ocvrs_return) {
		try {
			instance->setLightAdaptation(light_adapt);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_TonemapReinhard_getColorAdaptation_const(const cv::TonemapReinhard* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getColorAdaptation();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_TonemapReinhard_setColorAdaptation_float(cv::TonemapReinhard* instance, float color_adapt, Result_void* ocvrs_return) {
		try {
			instance->setColorAdaptation(color_adapt);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
}
