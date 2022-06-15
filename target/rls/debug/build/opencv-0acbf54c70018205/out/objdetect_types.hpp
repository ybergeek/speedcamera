template struct Result<bool>;
template struct Result<const cv::DetectionBasedTracker::Parameters*>;
template struct Result<cv::CascadeClassifier*>;
template struct Result<cv::DetectionBasedTracker*>;
template struct Result<cv::DetectionBasedTracker::ExtObject*>;
template struct Result<cv::DetectionBasedTracker::ObjectStatus>;
template struct Result<cv::DetectionBasedTracker::Parameters*>;
template struct Result<cv::DetectionROI*>;
template struct Result<cv::HOGDescriptor*>;
template struct Result<cv::HOGDescriptor::HistogramNormType>;
template struct Result<cv::Mat*>;
template struct Result<cv::Point_<int>>;
template struct Result<cv::Ptr<cv::BaseCascadeClassifier::MaskGenerator>*>;
template struct Result<cv::Ptr<cv::BaseCascadeClassifier>*>;
template struct Result<cv::Ptr<cv::FaceDetectorYN>*>;
template struct Result<cv::Ptr<cv::FaceRecognizerSF>*>;
template struct Result<cv::Ptr<cv::QRCodeEncoder>*>;
template struct Result<cv::QRCodeDetector*>;
template struct Result<cv::QRCodeEncoder::CorrectionLevel>;
template struct Result<cv::QRCodeEncoder::EncodeMode>;
template struct Result<cv::QRCodeEncoder::Params>;
template struct Result<cv::Rect_<int>>;
template struct Result<cv::SimilarRects*>;
template struct Result<cv::Size_<int>>;
template struct Result<cv::UMat*>;
template struct Result<double>;
template struct Result<float>;
template struct Result<int>;
template struct Result<std::vector<cv::Point_<int>>*>;
template struct Result<std::vector<double>*>;
template struct Result<std::vector<float>*>;
template struct Result<unsigned long>;
template struct Result<void*>;
extern "C" {
	void cv_PtrOfBaseCascadeClassifier_delete(cv::Ptr<cv::BaseCascadeClassifier>* instance) {
		delete instance;
	}

	const cv::BaseCascadeClassifier* cv_PtrOfBaseCascadeClassifier_get_inner_ptr(const cv::Ptr<cv::BaseCascadeClassifier>* instance) {
		return instance->get();
	}

	cv::BaseCascadeClassifier* cv_PtrOfBaseCascadeClassifier_get_inner_ptr_mut(cv::Ptr<cv::BaseCascadeClassifier>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfBaseCascadeClassifier_MaskGenerator_delete(cv::Ptr<cv::BaseCascadeClassifier::MaskGenerator>* instance) {
		delete instance;
	}

	const cv::BaseCascadeClassifier::MaskGenerator* cv_PtrOfBaseCascadeClassifier_MaskGenerator_get_inner_ptr(const cv::Ptr<cv::BaseCascadeClassifier::MaskGenerator>* instance) {
		return instance->get();
	}

	cv::BaseCascadeClassifier::MaskGenerator* cv_PtrOfBaseCascadeClassifier_MaskGenerator_get_inner_ptr_mut(cv::Ptr<cv::BaseCascadeClassifier::MaskGenerator>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfDetectionBasedTracker_IDetector_delete(cv::Ptr<cv::DetectionBasedTracker::IDetector>* instance) {
		delete instance;
	}

	const cv::DetectionBasedTracker::IDetector* cv_PtrOfDetectionBasedTracker_IDetector_get_inner_ptr(const cv::Ptr<cv::DetectionBasedTracker::IDetector>* instance) {
		return instance->get();
	}

	cv::DetectionBasedTracker::IDetector* cv_PtrOfDetectionBasedTracker_IDetector_get_inner_ptr_mut(cv::Ptr<cv::DetectionBasedTracker::IDetector>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfFaceDetectorYN_delete(cv::Ptr<cv::FaceDetectorYN>* instance) {
		delete instance;
	}

	const cv::FaceDetectorYN* cv_PtrOfFaceDetectorYN_get_inner_ptr(const cv::Ptr<cv::FaceDetectorYN>* instance) {
		return instance->get();
	}

	cv::FaceDetectorYN* cv_PtrOfFaceDetectorYN_get_inner_ptr_mut(cv::Ptr<cv::FaceDetectorYN>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfFaceRecognizerSF_delete(cv::Ptr<cv::FaceRecognizerSF>* instance) {
		delete instance;
	}

	const cv::FaceRecognizerSF* cv_PtrOfFaceRecognizerSF_get_inner_ptr(const cv::Ptr<cv::FaceRecognizerSF>* instance) {
		return instance->get();
	}

	cv::FaceRecognizerSF* cv_PtrOfFaceRecognizerSF_get_inner_ptr_mut(cv::Ptr<cv::FaceRecognizerSF>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_PtrOfQRCodeEncoder_delete(cv::Ptr<cv::QRCodeEncoder>* instance) {
		delete instance;
	}

	const cv::QRCodeEncoder* cv_PtrOfQRCodeEncoder_get_inner_ptr(const cv::Ptr<cv::QRCodeEncoder>* instance) {
		return instance->get();
	}

	cv::QRCodeEncoder* cv_PtrOfQRCodeEncoder_get_inner_ptr_mut(cv::Ptr<cv::QRCodeEncoder>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_VectorOfDetectionBasedTracker_ExtObject_delete(std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		delete instance;
	}

	std::vector<cv::DetectionBasedTracker::ExtObject>* cv_VectorOfDetectionBasedTracker_ExtObject_new() {
		return new std::vector<cv::DetectionBasedTracker::ExtObject>();
	}

	size_t cv_VectorOfDetectionBasedTracker_ExtObject_len(const std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		return instance->size();
	}

	bool cv_VectorOfDetectionBasedTracker_ExtObject_is_empty(const std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		return instance->empty();
	}

	size_t cv_VectorOfDetectionBasedTracker_ExtObject_capacity(const std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		return instance->capacity();
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_shrink_to_fit(std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		instance->shrink_to_fit();
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_reserve(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t additional) {
		instance->reserve(instance->size() + additional);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_remove(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t index) {
		instance->erase(instance->begin() + index);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_swap(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t index1, size_t index2) {
		std::swap((*instance)[index1], (*instance)[index2]);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_clear(std::vector<cv::DetectionBasedTracker::ExtObject>* instance) {
		instance->clear();
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_push(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, cv::DetectionBasedTracker::ExtObject* val) {
		instance->push_back(*val);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_insert(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t index, cv::DetectionBasedTracker::ExtObject* val) {
		instance->insert(instance->begin() + index, *val);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_get(const std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t index, cv::DetectionBasedTracker::ExtObject** ocvrs_return) {
		*ocvrs_return = new cv::DetectionBasedTracker::ExtObject((*instance)[index]);
	}

	void cv_VectorOfDetectionBasedTracker_ExtObject_set(std::vector<cv::DetectionBasedTracker::ExtObject>* instance, size_t index, cv::DetectionBasedTracker::ExtObject* val) {
		(*instance)[index] = *val;
	}

}


extern "C" {
	void cv_VectorOfDetectionROI_delete(std::vector<cv::DetectionROI>* instance) {
		delete instance;
	}

	std::vector<cv::DetectionROI>* cv_VectorOfDetectionROI_new() {
		return new std::vector<cv::DetectionROI>();
	}

	size_t cv_VectorOfDetectionROI_len(const std::vector<cv::DetectionROI>* instance) {
		return instance->size();
	}

	bool cv_VectorOfDetectionROI_is_empty(const std::vector<cv::DetectionROI>* instance) {
		return instance->empty();
	}

	size_t cv_VectorOfDetectionROI_capacity(const std::vector<cv::DetectionROI>* instance) {
		return instance->capacity();
	}

	void cv_VectorOfDetectionROI_shrink_to_fit(std::vector<cv::DetectionROI>* instance) {
		instance->shrink_to_fit();
	}

	void cv_VectorOfDetectionROI_reserve(std::vector<cv::DetectionROI>* instance, size_t additional) {
		instance->reserve(instance->size() + additional);
	}

	void cv_VectorOfDetectionROI_remove(std::vector<cv::DetectionROI>* instance, size_t index) {
		instance->erase(instance->begin() + index);
	}

	void cv_VectorOfDetectionROI_swap(std::vector<cv::DetectionROI>* instance, size_t index1, size_t index2) {
		std::swap((*instance)[index1], (*instance)[index2]);
	}

	void cv_VectorOfDetectionROI_clear(std::vector<cv::DetectionROI>* instance) {
		instance->clear();
	}

	void cv_VectorOfDetectionROI_push(std::vector<cv::DetectionROI>* instance, cv::DetectionROI* val) {
		instance->push_back(*val);
	}

	void cv_VectorOfDetectionROI_insert(std::vector<cv::DetectionROI>* instance, size_t index, cv::DetectionROI* val) {
		instance->insert(instance->begin() + index, *val);
	}

	void cv_VectorOfDetectionROI_get(const std::vector<cv::DetectionROI>* instance, size_t index, cv::DetectionROI** ocvrs_return) {
		*ocvrs_return = new cv::DetectionROI((*instance)[index]);
	}

	void cv_VectorOfDetectionROI_set(std::vector<cv::DetectionROI>* instance, size_t index, cv::DetectionROI* val) {
		(*instance)[index] = *val;
	}

}


