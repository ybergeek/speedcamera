#include "ocvrs_common.hpp"
#include <opencv2/phase_unwrapping.hpp>
#include "phase_unwrapping_types.hpp"

extern "C" {
	void cv_phase_unwrapping_HistogramPhaseUnwrapping_create_const_ParamsR(const cv::phase_unwrapping::HistogramPhaseUnwrapping::Params* parameters, Result<cv::Ptr<cv::phase_unwrapping::HistogramPhaseUnwrapping>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::phase_unwrapping::HistogramPhaseUnwrapping> ret = cv::phase_unwrapping::HistogramPhaseUnwrapping::create(*parameters);
			Ok(new cv::Ptr<cv::phase_unwrapping::HistogramPhaseUnwrapping>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::phase_unwrapping::HistogramPhaseUnwrapping>*>))
	}
	
	void cv_phase_unwrapping_HistogramPhaseUnwrapping_getInverseReliabilityMap_const__OutputArrayR(cv::phase_unwrapping::HistogramPhaseUnwrapping* instance, const cv::_OutputArray* reliabilityMap, Result_void* ocvrs_return) {
		try {
			instance->getInverseReliabilityMap(*reliabilityMap);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_phase_unwrapping_HistogramPhaseUnwrapping_Params_Params(Result<cv::phase_unwrapping::HistogramPhaseUnwrapping::Params>* ocvrs_return) {
		try {
			cv::phase_unwrapping::HistogramPhaseUnwrapping::Params ret;
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::phase_unwrapping::HistogramPhaseUnwrapping::Params>))
	}
	
	void cv_phase_unwrapping_PhaseUnwrapping_unwrapPhaseMap_const__InputArrayR_const__OutputArrayR_const__InputArrayR(cv::phase_unwrapping::PhaseUnwrapping* instance, const cv::_InputArray* wrappedPhaseMap, const cv::_OutputArray* unwrappedPhaseMap, const cv::_InputArray* shadowMask, Result_void* ocvrs_return) {
		try {
			instance->unwrapPhaseMap(*wrappedPhaseMap, *unwrappedPhaseMap, *shadowMask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
}
