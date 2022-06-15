template struct Result<char>;
template struct Result<cv::DMatch>;
template struct Result<cv::Mat*>;
template struct Result<cv::Point_<float>>;
template struct Result<cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>*>;
template struct Result<cv::Ptr<cv::line_descriptor::BinaryDescriptor>*>;
template struct Result<cv::Ptr<cv::line_descriptor::LSDDetector>*>;
template struct Result<cv::line_descriptor::BinaryDescriptorMatcher*>;
template struct Result<cv::line_descriptor::BinaryDescriptor*>;
template struct Result<cv::line_descriptor::BinaryDescriptor::Params*>;
template struct Result<cv::line_descriptor::KeyLine>;
template struct Result<cv::line_descriptor::LSDDetector*>;
template struct Result<cv::line_descriptor::LSDParam>;
template struct Result<double>;
template struct Result<float>;
template struct Result<int>;
template struct Result<std::vector<cv::DMatch>*>;
template struct Result<std::vector<cv::line_descriptor::KeyLine>*>;
extern "C" {
	cv::Ptr<cv::line_descriptor::BinaryDescriptor>* cv_PtrOfBinaryDescriptor_new(cv::line_descriptor::BinaryDescriptor* val) {
		return new cv::Ptr<cv::line_descriptor::BinaryDescriptor>(val);
	}
	
	void cv_PtrOfBinaryDescriptor_delete(cv::Ptr<cv::line_descriptor::BinaryDescriptor>* instance) {
		delete instance;
	}

	const cv::line_descriptor::BinaryDescriptor* cv_PtrOfBinaryDescriptor_get_inner_ptr(const cv::Ptr<cv::line_descriptor::BinaryDescriptor>* instance) {
		return instance->get();
	}

	cv::line_descriptor::BinaryDescriptor* cv_PtrOfBinaryDescriptor_get_inner_ptr_mut(cv::Ptr<cv::line_descriptor::BinaryDescriptor>* instance) {
		return instance->get();
	}
}

extern "C" {
	cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>* cv_PtrOfBinaryDescriptorMatcher_new(cv::line_descriptor::BinaryDescriptorMatcher* val) {
		return new cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>(val);
	}
	
	void cv_PtrOfBinaryDescriptorMatcher_delete(cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>* instance) {
		delete instance;
	}

	const cv::line_descriptor::BinaryDescriptorMatcher* cv_PtrOfBinaryDescriptorMatcher_get_inner_ptr(const cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>* instance) {
		return instance->get();
	}

	cv::line_descriptor::BinaryDescriptorMatcher* cv_PtrOfBinaryDescriptorMatcher_get_inner_ptr_mut(cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>* instance) {
		return instance->get();
	}
}

extern "C" {
	cv::Ptr<cv::line_descriptor::LSDDetector>* cv_PtrOfLSDDetector_new(cv::line_descriptor::LSDDetector* val) {
		return new cv::Ptr<cv::line_descriptor::LSDDetector>(val);
	}
	
	void cv_PtrOfLSDDetector_delete(cv::Ptr<cv::line_descriptor::LSDDetector>* instance) {
		delete instance;
	}

	const cv::line_descriptor::LSDDetector* cv_PtrOfLSDDetector_get_inner_ptr(const cv::Ptr<cv::line_descriptor::LSDDetector>* instance) {
		return instance->get();
	}

	cv::line_descriptor::LSDDetector* cv_PtrOfLSDDetector_get_inner_ptr_mut(cv::Ptr<cv::line_descriptor::LSDDetector>* instance) {
		return instance->get();
	}
}

extern "C" {
	void cv_VectorOfKeyLine_delete(std::vector<cv::line_descriptor::KeyLine>* instance) {
		delete instance;
	}

	std::vector<cv::line_descriptor::KeyLine>* cv_VectorOfKeyLine_new() {
		return new std::vector<cv::line_descriptor::KeyLine>();
	}

	size_t cv_VectorOfKeyLine_len(const std::vector<cv::line_descriptor::KeyLine>* instance) {
		return instance->size();
	}

	bool cv_VectorOfKeyLine_is_empty(const std::vector<cv::line_descriptor::KeyLine>* instance) {
		return instance->empty();
	}

	size_t cv_VectorOfKeyLine_capacity(const std::vector<cv::line_descriptor::KeyLine>* instance) {
		return instance->capacity();
	}

	void cv_VectorOfKeyLine_shrink_to_fit(std::vector<cv::line_descriptor::KeyLine>* instance) {
		instance->shrink_to_fit();
	}

	void cv_VectorOfKeyLine_reserve(std::vector<cv::line_descriptor::KeyLine>* instance, size_t additional) {
		instance->reserve(instance->size() + additional);
	}

	void cv_VectorOfKeyLine_remove(std::vector<cv::line_descriptor::KeyLine>* instance, size_t index) {
		instance->erase(instance->begin() + index);
	}

	void cv_VectorOfKeyLine_swap(std::vector<cv::line_descriptor::KeyLine>* instance, size_t index1, size_t index2) {
		std::swap((*instance)[index1], (*instance)[index2]);
	}

	void cv_VectorOfKeyLine_clear(std::vector<cv::line_descriptor::KeyLine>* instance) {
		instance->clear();
	}

	void cv_VectorOfKeyLine_push(std::vector<cv::line_descriptor::KeyLine>* instance, cv::line_descriptor::KeyLine* val) {
		instance->push_back(*val);
	}

	void cv_VectorOfKeyLine_insert(std::vector<cv::line_descriptor::KeyLine>* instance, size_t index, cv::line_descriptor::KeyLine* val) {
		instance->insert(instance->begin() + index, *val);
	}

	void cv_VectorOfKeyLine_get(const std::vector<cv::line_descriptor::KeyLine>* instance, size_t index, cv::line_descriptor::KeyLine* ocvrs_return) {
		*ocvrs_return = (*instance)[index];
	}

	void cv_VectorOfKeyLine_set(std::vector<cv::line_descriptor::KeyLine>* instance, size_t index, cv::line_descriptor::KeyLine* val) {
		(*instance)[index] = *val;
	}

	const cv::line_descriptor::KeyLine* cv_VectorOfKeyLine_data(const std::vector<cv::line_descriptor::KeyLine>* instance) {
		return instance->data();
	}
	
	cv::line_descriptor::KeyLine* cv_VectorOfKeyLine_data_mut(std::vector<cv::line_descriptor::KeyLine>* instance) {
		return instance->data();
	}
	
	std::vector<cv::line_descriptor::KeyLine>* cv_VectorOfKeyLine_clone(const std::vector<cv::line_descriptor::KeyLine>* instance) {
		return new std::vector<cv::line_descriptor::KeyLine>(*instance);
	}
	
	std::vector<cv::line_descriptor::KeyLine>* cv_VectorOfKeyLine_from_slice(const cv::line_descriptor::KeyLine* data, size_t len) {
		return new std::vector<cv::line_descriptor::KeyLine>(data, data + len);
	}
}


extern "C" {
	void cv_VectorOfVectorOfKeyLine_delete(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		delete instance;
	}

	std::vector<std::vector<cv::line_descriptor::KeyLine>>* cv_VectorOfVectorOfKeyLine_new() {
		return new std::vector<std::vector<cv::line_descriptor::KeyLine>>();
	}

	size_t cv_VectorOfVectorOfKeyLine_len(const std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		return instance->size();
	}

	bool cv_VectorOfVectorOfKeyLine_is_empty(const std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		return instance->empty();
	}

	size_t cv_VectorOfVectorOfKeyLine_capacity(const std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		return instance->capacity();
	}

	void cv_VectorOfVectorOfKeyLine_shrink_to_fit(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		instance->shrink_to_fit();
	}

	void cv_VectorOfVectorOfKeyLine_reserve(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t additional) {
		instance->reserve(instance->size() + additional);
	}

	void cv_VectorOfVectorOfKeyLine_remove(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t index) {
		instance->erase(instance->begin() + index);
	}

	void cv_VectorOfVectorOfKeyLine_swap(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t index1, size_t index2) {
		std::swap((*instance)[index1], (*instance)[index2]);
	}

	void cv_VectorOfVectorOfKeyLine_clear(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance) {
		instance->clear();
	}

	void cv_VectorOfVectorOfKeyLine_push(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, std::vector<cv::line_descriptor::KeyLine>* val) {
		instance->push_back(*val);
	}

	void cv_VectorOfVectorOfKeyLine_insert(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t index, std::vector<cv::line_descriptor::KeyLine>* val) {
		instance->insert(instance->begin() + index, *val);
	}

	void cv_VectorOfVectorOfKeyLine_get(const std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t index, std::vector<cv::line_descriptor::KeyLine>** ocvrs_return) {
		*ocvrs_return = new std::vector<cv::line_descriptor::KeyLine>((*instance)[index]);
	}

	void cv_VectorOfVectorOfKeyLine_set(std::vector<std::vector<cv::line_descriptor::KeyLine>>* instance, size_t index, std::vector<cv::line_descriptor::KeyLine>* val) {
		(*instance)[index] = *val;
	}

}


