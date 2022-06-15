template struct Result<bool>;
template struct Result<cv::Mat*>;
template struct Result<cv::Matx<double, 2, 3>>;
template struct Result<cv::Point3_<int>>;
template struct Result<cv::Point_<int>>;
template struct Result<cv::Ptr<cv::StereoMatcher>*>;
template struct Result<cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::ContourFitting>*>;
template struct Result<cv::Ptr<cv::ximgproc::DTFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::DisparityWLSFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::EdgeAwareInterpolator>*>;
template struct Result<cv::Ptr<cv::ximgproc::EdgeBoxes>*>;
template struct Result<cv::Ptr<cv::ximgproc::EdgeDrawing>*>;
template struct Result<cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::FastLineDetector>*>;
template struct Result<cv::Ptr<cv::ximgproc::GuidedFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::RFFeatureGetter>*>;
template struct Result<cv::Ptr<cv::ximgproc::RICInterpolator>*>;
template struct Result<cv::Ptr<cv::ximgproc::RidgeDetectionFilter>*>;
template struct Result<cv::Ptr<cv::ximgproc::ScanSegment>*>;
template struct Result<cv::Ptr<cv::ximgproc::StructuredEdgeDetection>*>;
template struct Result<cv::Ptr<cv::ximgproc::SuperpixelLSC>*>;
template struct Result<cv::Ptr<cv::ximgproc::SuperpixelSEEDS>*>;
template struct Result<cv::Ptr<cv::ximgproc::SuperpixelSLIC>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture>*>;
template struct Result<cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation>*>;
template struct Result<cv::Rect_<int>>;
template struct Result<cv::Vec<int, 4>>;
template struct Result<cv::ximgproc::ContourFitting*>;
template struct Result<cv::ximgproc::EdgeDrawing::Params>;
template struct Result<double>;
template struct Result<float>;
template struct Result<int>;
template struct Result<std::vector<cv::Point_<int>>*>;
template struct Result<std::vector<int>*>;
template struct Result<std::vector<std::vector<cv::Point_<int>>>*>;
extern "C" {
	void cv_PtrOfAdaptiveManifoldFilter_delete(cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::AdaptiveManifoldFilter* cv_PtrOfAdaptiveManifoldFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::AdaptiveManifoldFilter* cv_PtrOfAdaptiveManifoldFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	cv::Ptr<cv::ximgproc::ContourFitting>* cv_PtrOfContourFitting_new(cv::ximgproc::ContourFitting* val) {
		return new cv::Ptr<cv::ximgproc::ContourFitting>(val);
	}
	
	void cv_PtrOfContourFitting_delete(cv::Ptr<cv::ximgproc::ContourFitting>* instance) {
		delete instance;
	}

	const cv::ximgproc::ContourFitting* cv_PtrOfContourFitting_get_inner_ptr(const cv::Ptr<cv::ximgproc::ContourFitting>* instance) {
		return instance->get();
	}

	cv::ximgproc::ContourFitting* cv_PtrOfContourFitting_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::ContourFitting>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfDTFilter_delete(cv::Ptr<cv::ximgproc::DTFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::DTFilter* cv_PtrOfDTFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::DTFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::DTFilter* cv_PtrOfDTFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::DTFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfDisparityWLSFilter_delete(cv::Ptr<cv::ximgproc::DisparityWLSFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::DisparityWLSFilter* cv_PtrOfDisparityWLSFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::DisparityWLSFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::DisparityWLSFilter* cv_PtrOfDisparityWLSFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::DisparityWLSFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfEdgeAwareInterpolator_delete(cv::Ptr<cv::ximgproc::EdgeAwareInterpolator>* instance) {
		delete instance;
	}

	const cv::ximgproc::EdgeAwareInterpolator* cv_PtrOfEdgeAwareInterpolator_get_inner_ptr(const cv::Ptr<cv::ximgproc::EdgeAwareInterpolator>* instance) {
		return instance->get();
	}

	cv::ximgproc::EdgeAwareInterpolator* cv_PtrOfEdgeAwareInterpolator_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::EdgeAwareInterpolator>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfEdgeBoxes_delete(cv::Ptr<cv::ximgproc::EdgeBoxes>* instance) {
		delete instance;
	}

	const cv::ximgproc::EdgeBoxes* cv_PtrOfEdgeBoxes_get_inner_ptr(const cv::Ptr<cv::ximgproc::EdgeBoxes>* instance) {
		return instance->get();
	}

	cv::ximgproc::EdgeBoxes* cv_PtrOfEdgeBoxes_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::EdgeBoxes>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfEdgeDrawing_delete(cv::Ptr<cv::ximgproc::EdgeDrawing>* instance) {
		delete instance;
	}

	const cv::ximgproc::EdgeDrawing* cv_PtrOfEdgeDrawing_get_inner_ptr(const cv::Ptr<cv::ximgproc::EdgeDrawing>* instance) {
		return instance->get();
	}

	cv::ximgproc::EdgeDrawing* cv_PtrOfEdgeDrawing_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::EdgeDrawing>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfFastBilateralSolverFilter_delete(cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::FastBilateralSolverFilter* cv_PtrOfFastBilateralSolverFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::FastBilateralSolverFilter* cv_PtrOfFastBilateralSolverFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfFastGlobalSmootherFilter_delete(cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::FastGlobalSmootherFilter* cv_PtrOfFastGlobalSmootherFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::FastGlobalSmootherFilter* cv_PtrOfFastGlobalSmootherFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfFastLineDetector_delete(cv::Ptr<cv::ximgproc::FastLineDetector>* instance) {
		delete instance;
	}

	const cv::ximgproc::FastLineDetector* cv_PtrOfFastLineDetector_get_inner_ptr(const cv::Ptr<cv::ximgproc::FastLineDetector>* instance) {
		return instance->get();
	}

	cv::ximgproc::FastLineDetector* cv_PtrOfFastLineDetector_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::FastLineDetector>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfGraphSegmentation_delete(cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::GraphSegmentation* cv_PtrOfGraphSegmentation_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::GraphSegmentation* cv_PtrOfGraphSegmentation_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfGuidedFilter_delete(cv::Ptr<cv::ximgproc::GuidedFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::GuidedFilter* cv_PtrOfGuidedFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::GuidedFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::GuidedFilter* cv_PtrOfGuidedFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::GuidedFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfRFFeatureGetter_delete(cv::Ptr<cv::ximgproc::RFFeatureGetter>* instance) {
		delete instance;
	}

	const cv::ximgproc::RFFeatureGetter* cv_PtrOfRFFeatureGetter_get_inner_ptr(const cv::Ptr<cv::ximgproc::RFFeatureGetter>* instance) {
		return instance->get();
	}

	cv::ximgproc::RFFeatureGetter* cv_PtrOfRFFeatureGetter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::RFFeatureGetter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfRICInterpolator_delete(cv::Ptr<cv::ximgproc::RICInterpolator>* instance) {
		delete instance;
	}

	const cv::ximgproc::RICInterpolator* cv_PtrOfRICInterpolator_get_inner_ptr(const cv::Ptr<cv::ximgproc::RICInterpolator>* instance) {
		return instance->get();
	}

	cv::ximgproc::RICInterpolator* cv_PtrOfRICInterpolator_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::RICInterpolator>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfRidgeDetectionFilter_delete(cv::Ptr<cv::ximgproc::RidgeDetectionFilter>* instance) {
		delete instance;
	}

	const cv::ximgproc::RidgeDetectionFilter* cv_PtrOfRidgeDetectionFilter_get_inner_ptr(const cv::Ptr<cv::ximgproc::RidgeDetectionFilter>* instance) {
		return instance->get();
	}

	cv::ximgproc::RidgeDetectionFilter* cv_PtrOfRidgeDetectionFilter_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::RidgeDetectionFilter>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfScanSegment_delete(cv::Ptr<cv::ximgproc::ScanSegment>* instance) {
		delete instance;
	}

	const cv::ximgproc::ScanSegment* cv_PtrOfScanSegment_get_inner_ptr(const cv::Ptr<cv::ximgproc::ScanSegment>* instance) {
		return instance->get();
	}

	cv::ximgproc::ScanSegment* cv_PtrOfScanSegment_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::ScanSegment>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentation_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentation* cv_PtrOfSelectiveSearchSegmentation_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentation* cv_PtrOfSelectiveSearchSegmentation_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategy_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy* cv_PtrOfSelectiveSearchSegmentationStrategy_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy* cv_PtrOfSelectiveSearchSegmentationStrategy_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategyColor_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor* cv_PtrOfSelectiveSearchSegmentationStrategyColor_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor* cv_PtrOfSelectiveSearchSegmentationStrategyColor_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor>* instance) {
		return instance->get();
	}
	
	cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* cv_PtrOfSelectiveSearchSegmentationStrategyColor_to_PtrOfSelectiveSearchSegmentationStrategy(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyColor>* instance) {
		return new cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>(instance->dynamicCast<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>());
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategyFill_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill* cv_PtrOfSelectiveSearchSegmentationStrategyFill_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill* cv_PtrOfSelectiveSearchSegmentationStrategyFill_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill>* instance) {
		return instance->get();
	}
	
	cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* cv_PtrOfSelectiveSearchSegmentationStrategyFill_to_PtrOfSelectiveSearchSegmentationStrategy(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyFill>* instance) {
		return new cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>(instance->dynamicCast<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>());
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategyMultiple_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple* cv_PtrOfSelectiveSearchSegmentationStrategyMultiple_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple* cv_PtrOfSelectiveSearchSegmentationStrategyMultiple_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple>* instance) {
		return instance->get();
	}
	
	cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* cv_PtrOfSelectiveSearchSegmentationStrategyMultiple_to_PtrOfSelectiveSearchSegmentationStrategy(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyMultiple>* instance) {
		return new cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>(instance->dynamicCast<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>());
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategySize_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize* cv_PtrOfSelectiveSearchSegmentationStrategySize_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize* cv_PtrOfSelectiveSearchSegmentationStrategySize_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize>* instance) {
		return instance->get();
	}
	
	cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* cv_PtrOfSelectiveSearchSegmentationStrategySize_to_PtrOfSelectiveSearchSegmentationStrategy(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategySize>* instance) {
		return new cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>(instance->dynamicCast<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>());
	}
}

extern "C" {
	void cv_PtrOfSelectiveSearchSegmentationStrategyTexture_delete(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture>* instance) {
		delete instance;
	}

	const cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture* cv_PtrOfSelectiveSearchSegmentationStrategyTexture_get_inner_ptr(const cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture>* instance) {
		return instance->get();
	}

	cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture* cv_PtrOfSelectiveSearchSegmentationStrategyTexture_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture>* instance) {
		return instance->get();
	}
	
	cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>* cv_PtrOfSelectiveSearchSegmentationStrategyTexture_to_PtrOfSelectiveSearchSegmentationStrategy(cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategyTexture>* instance) {
		return new cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>(instance->dynamicCast<cv::ximgproc::segmentation::SelectiveSearchSegmentationStrategy>());
	}
}

extern "C" {
	void cv_PtrOfStructuredEdgeDetection_delete(cv::Ptr<cv::ximgproc::StructuredEdgeDetection>* instance) {
		delete instance;
	}

	const cv::ximgproc::StructuredEdgeDetection* cv_PtrOfStructuredEdgeDetection_get_inner_ptr(const cv::Ptr<cv::ximgproc::StructuredEdgeDetection>* instance) {
		return instance->get();
	}

	cv::ximgproc::StructuredEdgeDetection* cv_PtrOfStructuredEdgeDetection_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::StructuredEdgeDetection>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSuperpixelLSC_delete(cv::Ptr<cv::ximgproc::SuperpixelLSC>* instance) {
		delete instance;
	}

	const cv::ximgproc::SuperpixelLSC* cv_PtrOfSuperpixelLSC_get_inner_ptr(const cv::Ptr<cv::ximgproc::SuperpixelLSC>* instance) {
		return instance->get();
	}

	cv::ximgproc::SuperpixelLSC* cv_PtrOfSuperpixelLSC_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::SuperpixelLSC>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSuperpixelSEEDS_delete(cv::Ptr<cv::ximgproc::SuperpixelSEEDS>* instance) {
		delete instance;
	}

	const cv::ximgproc::SuperpixelSEEDS* cv_PtrOfSuperpixelSEEDS_get_inner_ptr(const cv::Ptr<cv::ximgproc::SuperpixelSEEDS>* instance) {
		return instance->get();
	}

	cv::ximgproc::SuperpixelSEEDS* cv_PtrOfSuperpixelSEEDS_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::SuperpixelSEEDS>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfSuperpixelSLIC_delete(cv::Ptr<cv::ximgproc::SuperpixelSLIC>* instance) {
		delete instance;
	}

	const cv::ximgproc::SuperpixelSLIC* cv_PtrOfSuperpixelSLIC_get_inner_ptr(const cv::Ptr<cv::ximgproc::SuperpixelSLIC>* instance) {
		return instance->get();
	}

	cv::ximgproc::SuperpixelSLIC* cv_PtrOfSuperpixelSLIC_get_inner_ptr_mut(cv::Ptr<cv::ximgproc::SuperpixelSLIC>* instance) {
		return instance->get();
	}
}

