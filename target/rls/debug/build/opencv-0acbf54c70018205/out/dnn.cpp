#include "dnn.hpp"
#include "dnn_types.hpp"

extern "C" {
	void cv_dnn_NMSBoxes_const_vector_Rect2d_R_const_vector_float_R_const_float_const_float_vector_int_R_const_float_const_int(const std::vector<cv::Rect2d>* bboxes, const std::vector<float>* scores, const float score_threshold, const float nms_threshold, std::vector<int>* indices, const float eta, const int top_k, Result_void* ocvrs_return) {
		try {
			cv::dnn::NMSBoxes(*bboxes, *scores, score_threshold, nms_threshold, *indices, eta, top_k);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_NMSBoxes_const_vector_Rect_R_const_vector_float_R_const_float_const_float_vector_int_R_const_float_const_int(const std::vector<cv::Rect>* bboxes, const std::vector<float>* scores, const float score_threshold, const float nms_threshold, std::vector<int>* indices, const float eta, const int top_k, Result_void* ocvrs_return) {
		try {
			cv::dnn::NMSBoxes(*bboxes, *scores, score_threshold, nms_threshold, *indices, eta, top_k);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_NMSBoxes_const_vector_RotatedRect_R_const_vector_float_R_const_float_const_float_vector_int_R_const_float_const_int(const std::vector<cv::RotatedRect>* bboxes, const std::vector<float>* scores, const float score_threshold, const float nms_threshold, std::vector<int>* indices, const float eta, const int top_k, Result_void* ocvrs_return) {
		try {
			cv::dnn::NMSBoxes(*bboxes, *scores, score_threshold, nms_threshold, *indices, eta, top_k);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_blobFromImage_const__InputArrayR_const__OutputArrayR_double_const_SizeR_const_ScalarR_bool_bool_int(const cv::_InputArray* image, const cv::_OutputArray* blob, double scalefactor, const cv::Size* size, const cv::Scalar* mean, bool swapRB, bool crop, int ddepth, Result_void* ocvrs_return) {
		try {
			cv::dnn::blobFromImage(*image, *blob, scalefactor, *size, *mean, swapRB, crop, ddepth);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_blobFromImage_const__InputArrayR_double_const_SizeR_const_ScalarR_bool_bool_int(const cv::_InputArray* image, double scalefactor, const cv::Size* size, const cv::Scalar* mean, bool swapRB, bool crop, int ddepth, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::blobFromImage(*image, scalefactor, *size, *mean, swapRB, crop, ddepth);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_blobFromImages_const__InputArrayR_const__OutputArrayR_double_Size_const_ScalarR_bool_bool_int(const cv::_InputArray* images, const cv::_OutputArray* blob, double scalefactor, cv::Size* size, const cv::Scalar* mean, bool swapRB, bool crop, int ddepth, Result_void* ocvrs_return) {
		try {
			cv::dnn::blobFromImages(*images, *blob, scalefactor, *size, *mean, swapRB, crop, ddepth);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_blobFromImages_const__InputArrayR_double_Size_const_ScalarR_bool_bool_int(const cv::_InputArray* images, double scalefactor, cv::Size* size, const cv::Scalar* mean, bool swapRB, bool crop, int ddepth, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::blobFromImages(*images, scalefactor, *size, *mean, swapRB, crop, ddepth);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_concat_const_MatShapeR_const_MatShapeR(const cv::dnn::MatShape* a, const cv::dnn::MatShape* b, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::concat(*a, *b);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_enableModelDiagnostics_bool(bool isDiagnosticsMode, Result_void* ocvrs_return) {
		try {
			cv::dnn::enableModelDiagnostics(isDiagnosticsMode);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_getAvailableTargets_Backend(cv::dnn::Backend be, Result<std::vector<cv::dnn::Target>*>* ocvrs_return) {
		try {
			std::vector<cv::dnn::Target> ret = cv::dnn::getAvailableTargets(be);
			Ok(new std::vector<cv::dnn::Target>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::dnn::Target>*>))
	}
	
	void cv_dnn_getInferenceEngineBackendType(Result<void*>* ocvrs_return) {
		try {
			cv::String ret = cv::dnn::getInferenceEngineBackendType();
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_getInferenceEngineCPUType(Result<void*>* ocvrs_return) {
		try {
			cv::String ret = cv::dnn::getInferenceEngineCPUType();
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_getInferenceEngineVPUType(Result<void*>* ocvrs_return) {
		try {
			cv::String ret = cv::dnn::getInferenceEngineVPUType();
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_getPlane_const_MatR_int_int(const cv::Mat* m, int n, int cn, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::getPlane(*m, n, cn);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_imagesFromBlob_const_MatR_const__OutputArrayR(const cv::Mat* blob_, const cv::_OutputArray* images_, Result_void* ocvrs_return) {
		try {
			cv::dnn::imagesFromBlob(*blob_, *images_);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_print_const_MatShapeR_const_StringR(const cv::dnn::MatShape* shape, const char* name, Result_void* ocvrs_return) {
		try {
			cv::dnn::print(*shape, std::string(name));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_readNetFromCaffe_const_StringR_const_StringR(const char* prototxt, const char* caffeModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromCaffe(std::string(prototxt), std::string(caffeModel));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromCaffe_const_charX_size_t_const_charX_size_t(const char* bufferProto, size_t lenProto, const char* bufferModel, size_t lenModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromCaffe(bufferProto, lenProto, bufferModel, lenModel);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromCaffe_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const std::vector<unsigned char>* bufferProto, const std::vector<unsigned char>* bufferModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromCaffe(*bufferProto, *bufferModel);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromDarknet_const_StringR_const_StringR(const char* cfgFile, const char* darknetModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromDarknet(std::string(cfgFile), std::string(darknetModel));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromDarknet_const_charX_size_t_const_charX_size_t(const char* bufferCfg, size_t lenCfg, const char* bufferModel, size_t lenModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromDarknet(bufferCfg, lenCfg, bufferModel, lenModel);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromDarknet_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const std::vector<unsigned char>* bufferCfg, const std::vector<unsigned char>* bufferModel, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromDarknet(*bufferCfg, *bufferModel);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromModelOptimizer_const_StringR_const_StringR(const char* xml, const char* bin, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromModelOptimizer(std::string(xml), std::string(bin));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromModelOptimizer_const_unsigned_charX_size_t_const_unsigned_charX_size_t(const unsigned char* bufferModelConfigPtr, size_t bufferModelConfigSize, const unsigned char* bufferWeightsPtr, size_t bufferWeightsSize, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromModelOptimizer(bufferModelConfigPtr, bufferModelConfigSize, bufferWeightsPtr, bufferWeightsSize);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromModelOptimizer_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const std::vector<unsigned char>* bufferModelConfig, const std::vector<unsigned char>* bufferWeights, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromModelOptimizer(*bufferModelConfig, *bufferWeights);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromONNX_const_StringR(const char* onnxFile, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromONNX(std::string(onnxFile));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromONNX_const_charX_size_t(const char* buffer, size_t sizeBuffer, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromONNX(buffer, sizeBuffer);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromONNX_const_vector_unsigned_char_R(const std::vector<unsigned char>* buffer, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromONNX(*buffer);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromTensorflow_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromTensorflow(std::string(model), std::string(config));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromTensorflow_const_charX_size_t_const_charX_size_t(const char* bufferModel, size_t lenModel, const char* bufferConfig, size_t lenConfig, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromTensorflow(bufferModel, lenModel, bufferConfig, lenConfig);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromTensorflow_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const std::vector<unsigned char>* bufferModel, const std::vector<unsigned char>* bufferConfig, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromTensorflow(*bufferModel, *bufferConfig);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNetFromTorch_const_StringR_bool_bool(const char* model, bool isBinary, bool evaluate, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNetFromTorch(std::string(model), isBinary, evaluate);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNet_const_StringR_const_StringR_const_StringR(const char* model, const char* config, const char* framework, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNet(std::string(model), std::string(config), std::string(framework));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readNet_const_StringR_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const char* framework, const std::vector<unsigned char>* bufferModel, const std::vector<unsigned char>* bufferConfig, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::readNet(std::string(framework), *bufferModel, *bufferConfig);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_readTensorFromONNX_const_StringR(const char* path, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::readTensorFromONNX(std::string(path));
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_readTorchBlob_const_StringR_bool(const char* filename, bool isBinary, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::readTorchBlob(std::string(filename), isBinary);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_releaseHDDLPlugin(Result_void* ocvrs_return) {
		try {
			cv::dnn::releaseHDDLPlugin();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_resetMyriadDevice(Result_void* ocvrs_return) {
		try {
			cv::dnn::resetMyriadDevice();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_setInferenceEngineBackendType_const_StringR(const char* newBackendType, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = cv::dnn::setInferenceEngineBackendType(std::string(newBackendType));
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_shape_const_MatR(const cv::Mat* mat, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::shape(*mat);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_shape_const_MatSizeR(const cv::MatSize* sz, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::shape(*sz);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_shape_const_UMatR(const cv::UMat* mat, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::shape(*mat);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_shape_const_intX_const_int(const int* dims, const int n, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::shape(dims, n);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_shape_int_int_int_int(int a0, int a1, int a2, int a3, Result<cv::dnn::MatShape*>* ocvrs_return) {
		try {
			cv::dnn::MatShape ret = cv::dnn::shape(a0, a1, a2, a3);
			Ok(new cv::dnn::MatShape(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::MatShape*>))
	}
	
	void cv_dnn_shrinkCaffeModel_const_StringR_const_StringR_const_vector_String_R(const char* src, const char* dst, const std::vector<cv::String>* layersTypes, Result_void* ocvrs_return) {
		try {
			cv::dnn::shrinkCaffeModel(std::string(src), std::string(dst), *layersTypes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_slice_const_MatR_const__RangeR(const cv::Mat* m, const cv::dnn::_Range* r0, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::slice(*m, *r0);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_slice_const_MatR_const__RangeR_const__RangeR(const cv::Mat* m, const cv::dnn::_Range* r0, const cv::dnn::_Range* r1, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::slice(*m, *r0, *r1);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_slice_const_MatR_const__RangeR_const__RangeR_const__RangeR(const cv::Mat* m, const cv::dnn::_Range* r0, const cv::dnn::_Range* r1, const cv::dnn::_Range* r2, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::slice(*m, *r0, *r1, *r2);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_slice_const_MatR_const__RangeR_const__RangeR_const__RangeR_const__RangeR(const cv::Mat* m, const cv::dnn::_Range* r0, const cv::dnn::_Range* r1, const cv::dnn::_Range* r2, const cv::dnn::_Range* r3, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::dnn::slice(*m, *r0, *r1, *r2, *r3);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_softNMSBoxes_const_vector_Rect_R_const_vector_float_R_vector_float_R_const_float_const_float_vector_int_R_size_t_const_float_SoftNMSMethod(const std::vector<cv::Rect>* bboxes, const std::vector<float>* scores, std::vector<float>* updated_scores, const float score_threshold, const float nms_threshold, std::vector<int>* indices, size_t top_k, const float sigma, cv::dnn::SoftNMSMethod method, Result_void* ocvrs_return) {
		try {
			cv::dnn::softNMSBoxes(*bboxes, *scores, *updated_scores, score_threshold, nms_threshold, *indices, top_k, sigma, method);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_toString_const_MatShapeR_const_StringR(const cv::dnn::MatShape* shape, const char* name, Result<void*>* ocvrs_return) {
		try {
			std::string ret = cv::dnn::toString(*shape, std::string(name));
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_total_const_MatShapeR_int_int(const cv::dnn::MatShape* shape, int start, int end, Result<int>* ocvrs_return) {
		try {
			int ret = cv::dnn::total(*shape, start, end);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_writeTextGraph_const_StringR_const_StringR(const char* model, const char* output, Result_void* ocvrs_return) {
		try {
			cv::dnn::writeTextGraph(std::string(model), std::string(output));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::dnn::ActivationLayer* cv_AbsLayer_to_ActivationLayer(cv::dnn::AbsLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AbsLayer_to_Algorithm(cv::dnn::AbsLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AbsLayer_to_Layer(cv::dnn::AbsLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AbsLayer_delete(cv::dnn::AbsLayer* instance) {
		delete instance;
	}
	void cv_dnn_AbsLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AbsLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AbsLayer> ret = cv::dnn::AbsLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AbsLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AbsLayer>*>))
	}
	
	cv::Algorithm* cv_AccumLayer_to_Algorithm(cv::dnn::AccumLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AccumLayer_to_Layer(cv::dnn::AccumLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AccumLayer_delete(cv::dnn::AccumLayer* instance) {
		delete instance;
	}
	void cv_dnn_AccumLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AccumLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AccumLayer> ret = cv::dnn::AccumLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AccumLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AccumLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AcosLayer_to_ActivationLayer(cv::dnn::AcosLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AcosLayer_to_Algorithm(cv::dnn::AcosLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AcosLayer_to_Layer(cv::dnn::AcosLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AcosLayer_delete(cv::dnn::AcosLayer* instance) {
		delete instance;
	}
	void cv_dnn_AcosLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AcosLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AcosLayer> ret = cv::dnn::AcosLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AcosLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AcosLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AcoshLayer_to_ActivationLayer(cv::dnn::AcoshLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AcoshLayer_to_Algorithm(cv::dnn::AcoshLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AcoshLayer_to_Layer(cv::dnn::AcoshLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AcoshLayer_delete(cv::dnn::AcoshLayer* instance) {
		delete instance;
	}
	void cv_dnn_AcoshLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AcoshLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AcoshLayer> ret = cv::dnn::AcoshLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AcoshLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AcoshLayer>*>))
	}
	
	cv::dnn::AbsLayer* cv_ActivationLayer_to_AbsLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AbsLayer*>(instance);
	}
	
	cv::dnn::AcosLayer* cv_ActivationLayer_to_AcosLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AcosLayer*>(instance);
	}
	
	cv::dnn::AcoshLayer* cv_ActivationLayer_to_AcoshLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AcoshLayer*>(instance);
	}
	
	cv::dnn::ActivationLayerInt8* cv_ActivationLayer_to_ActivationLayerInt8(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayerInt8*>(instance);
	}
	
	cv::dnn::AsinLayer* cv_ActivationLayer_to_AsinLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AsinLayer*>(instance);
	}
	
	cv::dnn::AsinhLayer* cv_ActivationLayer_to_AsinhLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AsinhLayer*>(instance);
	}
	
	cv::dnn::AtanLayer* cv_ActivationLayer_to_AtanLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AtanLayer*>(instance);
	}
	
	cv::dnn::AtanhLayer* cv_ActivationLayer_to_AtanhLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::AtanhLayer*>(instance);
	}
	
	cv::dnn::BNLLLayer* cv_ActivationLayer_to_BNLLLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::BNLLLayer*>(instance);
	}
	
	cv::dnn::BatchNormLayer* cv_ActivationLayer_to_BatchNormLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::BatchNormLayer*>(instance);
	}
	
	cv::dnn::CeilLayer* cv_ActivationLayer_to_CeilLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::CeilLayer*>(instance);
	}
	
	cv::dnn::CeluLayer* cv_ActivationLayer_to_CeluLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::CeluLayer*>(instance);
	}
	
	cv::dnn::ChannelsPReLULayer* cv_ActivationLayer_to_ChannelsPReLULayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ChannelsPReLULayer*>(instance);
	}
	
	cv::dnn::CosLayer* cv_ActivationLayer_to_CosLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::CosLayer*>(instance);
	}
	
	cv::dnn::CoshLayer* cv_ActivationLayer_to_CoshLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::CoshLayer*>(instance);
	}
	
	cv::dnn::ELULayer* cv_ActivationLayer_to_ELULayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ELULayer*>(instance);
	}
	
	cv::dnn::ErfLayer* cv_ActivationLayer_to_ErfLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ErfLayer*>(instance);
	}
	
	cv::dnn::ExpLayer* cv_ActivationLayer_to_ExpLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ExpLayer*>(instance);
	}
	
	cv::dnn::FloorLayer* cv_ActivationLayer_to_FloorLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::FloorLayer*>(instance);
	}
	
	cv::dnn::HardSigmoidLayer* cv_ActivationLayer_to_HardSigmoidLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::HardSigmoidLayer*>(instance);
	}
	
	cv::dnn::HardSwishLayer* cv_ActivationLayer_to_HardSwishLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::HardSwishLayer*>(instance);
	}
	
	cv::dnn::LogLayer* cv_ActivationLayer_to_LogLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::LogLayer*>(instance);
	}
	
	cv::dnn::MishLayer* cv_ActivationLayer_to_MishLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::MishLayer*>(instance);
	}
	
	cv::dnn::NotLayer* cv_ActivationLayer_to_NotLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::NotLayer*>(instance);
	}
	
	cv::dnn::PowerLayer* cv_ActivationLayer_to_PowerLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::PowerLayer*>(instance);
	}
	
	cv::dnn::ReLU6Layer* cv_ActivationLayer_to_ReLU6Layer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ReLU6Layer*>(instance);
	}
	
	cv::dnn::ReLULayer* cv_ActivationLayer_to_ReLULayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ReLULayer*>(instance);
	}
	
	cv::dnn::RoundLayer* cv_ActivationLayer_to_RoundLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::RoundLayer*>(instance);
	}
	
	cv::dnn::SeluLayer* cv_ActivationLayer_to_SeluLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SeluLayer*>(instance);
	}
	
	cv::dnn::SigmoidLayer* cv_ActivationLayer_to_SigmoidLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SigmoidLayer*>(instance);
	}
	
	cv::dnn::SinLayer* cv_ActivationLayer_to_SinLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SinLayer*>(instance);
	}
	
	cv::dnn::SinhLayer* cv_ActivationLayer_to_SinhLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SinhLayer*>(instance);
	}
	
	cv::dnn::SoftplusLayer* cv_ActivationLayer_to_SoftplusLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SoftplusLayer*>(instance);
	}
	
	cv::dnn::SoftsignLayer* cv_ActivationLayer_to_SoftsignLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SoftsignLayer*>(instance);
	}
	
	cv::dnn::SqrtLayer* cv_ActivationLayer_to_SqrtLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SqrtLayer*>(instance);
	}
	
	cv::dnn::SwishLayer* cv_ActivationLayer_to_SwishLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::SwishLayer*>(instance);
	}
	
	cv::dnn::TanHLayer* cv_ActivationLayer_to_TanHLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::TanHLayer*>(instance);
	}
	
	cv::dnn::TanLayer* cv_ActivationLayer_to_TanLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::TanLayer*>(instance);
	}
	
	cv::dnn::ThresholdedReluLayer* cv_ActivationLayer_to_ThresholdedReluLayer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::ThresholdedReluLayer*>(instance);
	}
	
	cv::Algorithm* cv_ActivationLayer_to_Algorithm(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ActivationLayer_to_Layer(cv::dnn::ActivationLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ActivationLayer_delete(cv::dnn::ActivationLayer* instance) {
		delete instance;
	}
	void cv_dnn_ActivationLayer_forwardSlice_const_const_floatX_floatX_int_size_t_int_int(const cv::dnn::ActivationLayer* instance, const float* src, float* dst, int len, size_t outPlaneSize, int cn0, int cn1, Result_void* ocvrs_return) {
		try {
			instance->forwardSlice(src, dst, len, outPlaneSize, cn0, cn1);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_ActivationLayer_forwardSlice_const_const_intX_const_intX_intX_int_size_t_int_int(const cv::dnn::ActivationLayer* instance, const int* src, const int* lut, int* dst, int len, size_t outPlaneSize, int cn0, int cn1, Result_void* ocvrs_return) {
		try {
			instance->forwardSlice(src, lut, dst, len, outPlaneSize, cn0, cn1);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::dnn::ActivationLayer* cv_ActivationLayerInt8_to_ActivationLayer(cv::dnn::ActivationLayerInt8* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ActivationLayerInt8_to_Algorithm(cv::dnn::ActivationLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ActivationLayerInt8_to_Layer(cv::dnn::ActivationLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ActivationLayerInt8_delete(cv::dnn::ActivationLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_ActivationLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ActivationLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ActivationLayerInt8> ret = cv::dnn::ActivationLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::ActivationLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ActivationLayerInt8>*>))
	}
	
	cv::Algorithm* cv_ArgLayer_to_Algorithm(cv::dnn::ArgLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ArgLayer_to_Layer(cv::dnn::ArgLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ArgLayer_delete(cv::dnn::ArgLayer* instance) {
		delete instance;
	}
	void cv_dnn_ArgLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ArgLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ArgLayer> ret = cv::dnn::ArgLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ArgLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ArgLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AsinLayer_to_ActivationLayer(cv::dnn::AsinLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AsinLayer_to_Algorithm(cv::dnn::AsinLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AsinLayer_to_Layer(cv::dnn::AsinLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AsinLayer_delete(cv::dnn::AsinLayer* instance) {
		delete instance;
	}
	void cv_dnn_AsinLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AsinLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AsinLayer> ret = cv::dnn::AsinLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AsinLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AsinLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AsinhLayer_to_ActivationLayer(cv::dnn::AsinhLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AsinhLayer_to_Algorithm(cv::dnn::AsinhLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AsinhLayer_to_Layer(cv::dnn::AsinhLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AsinhLayer_delete(cv::dnn::AsinhLayer* instance) {
		delete instance;
	}
	void cv_dnn_AsinhLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AsinhLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AsinhLayer> ret = cv::dnn::AsinhLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AsinhLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AsinhLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AtanLayer_to_ActivationLayer(cv::dnn::AtanLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AtanLayer_to_Algorithm(cv::dnn::AtanLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AtanLayer_to_Layer(cv::dnn::AtanLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AtanLayer_delete(cv::dnn::AtanLayer* instance) {
		delete instance;
	}
	void cv_dnn_AtanLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AtanLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AtanLayer> ret = cv::dnn::AtanLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AtanLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AtanLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_AtanhLayer_to_ActivationLayer(cv::dnn::AtanhLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_AtanhLayer_to_Algorithm(cv::dnn::AtanhLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_AtanhLayer_to_Layer(cv::dnn::AtanhLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_AtanhLayer_delete(cv::dnn::AtanhLayer* instance) {
		delete instance;
	}
	void cv_dnn_AtanhLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::AtanhLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::AtanhLayer> ret = cv::dnn::AtanhLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::AtanhLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::AtanhLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_BNLLLayer_to_ActivationLayer(cv::dnn::BNLLLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_BNLLLayer_to_Algorithm(cv::dnn::BNLLLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_BNLLLayer_to_Layer(cv::dnn::BNLLLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_BNLLLayer_delete(cv::dnn::BNLLLayer* instance) {
		delete instance;
	}
	void cv_dnn_BNLLLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BNLLLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BNLLLayer> ret = cv::dnn::BNLLLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::BNLLLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BNLLLayer>*>))
	}
	
	int cv_dnn_BackendNode_getPropBackendId_const(const cv::dnn::BackendNode* instance) {
			int ret = instance->backendId;
			return ret;
	}
	
	void cv_dnn_BackendNode_setPropBackendId_int(cv::dnn::BackendNode* instance, int val) {
			instance->backendId = val;
	}
	
	void cv_BackendNode_delete(cv::dnn::BackendNode* instance) {
		delete instance;
	}
	int cv_dnn_BackendWrapper_getPropBackendId_const(const cv::dnn::BackendWrapper* instance) {
			int ret = instance->backendId;
			return ret;
	}
	
	void cv_dnn_BackendWrapper_setPropBackendId_int(cv::dnn::BackendWrapper* instance, int val) {
			instance->backendId = val;
	}
	
	int cv_dnn_BackendWrapper_getPropTargetId_const(const cv::dnn::BackendWrapper* instance) {
			int ret = instance->targetId;
			return ret;
	}
	
	void cv_dnn_BackendWrapper_setPropTargetId_int(cv::dnn::BackendWrapper* instance, int val) {
			instance->targetId = val;
	}
	
	void cv_dnn_BackendWrapper_copyToHost(cv::dnn::BackendWrapper* instance, Result_void* ocvrs_return) {
		try {
			instance->copyToHost();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_BackendWrapper_setHostDirty(cv::dnn::BackendWrapper* instance, Result_void* ocvrs_return) {
		try {
			instance->setHostDirty();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_BaseConvolutionLayer_getPropKernel_const(const cv::dnn::BaseConvolutionLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->kernel;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropKernel_Size(cv::dnn::BaseConvolutionLayer* instance, cv::Size* val) {
			instance->kernel = *val;
	}
	
	void cv_dnn_BaseConvolutionLayer_getPropStride_const(const cv::dnn::BaseConvolutionLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->stride;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropStride_Size(cv::dnn::BaseConvolutionLayer* instance, cv::Size* val) {
			instance->stride = *val;
	}
	
	void cv_dnn_BaseConvolutionLayer_getPropPad_const(const cv::dnn::BaseConvolutionLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->pad;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropPad_Size(cv::dnn::BaseConvolutionLayer* instance, cv::Size* val) {
			instance->pad = *val;
	}
	
	void cv_dnn_BaseConvolutionLayer_getPropDilation_const(const cv::dnn::BaseConvolutionLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->dilation;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropDilation_Size(cv::dnn::BaseConvolutionLayer* instance, cv::Size* val) {
			instance->dilation = *val;
	}
	
	void cv_dnn_BaseConvolutionLayer_getPropAdjustPad_const(const cv::dnn::BaseConvolutionLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->adjustPad;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropAdjustPad_Size(cv::dnn::BaseConvolutionLayer* instance, cv::Size* val) {
			instance->adjustPad = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropAdjust_pads_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->adjust_pads;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropAdjust_pads_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->adjust_pads = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropKernel_size_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->kernel_size;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropKernel_size_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->kernel_size = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropStrides_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->strides;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropStrides_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->strides = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropDilations_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->dilations;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropDilations_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->dilations = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropPads_begin_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->pads_begin;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropPads_begin_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->pads_begin = *val;
	}
	
	std::vector<size_t>* cv_dnn_BaseConvolutionLayer_getPropPads_end_const(const cv::dnn::BaseConvolutionLayer* instance) {
			std::vector<size_t> ret = instance->pads_end;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropPads_end_vector_size_t_(cv::dnn::BaseConvolutionLayer* instance, std::vector<size_t>* val) {
			instance->pads_end = *val;
	}
	
	void* cv_dnn_BaseConvolutionLayer_getPropPadMode_const(const cv::dnn::BaseConvolutionLayer* instance) {
			cv::String ret = instance->padMode;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropPadMode_String(cv::dnn::BaseConvolutionLayer* instance, char* val) {
			instance->padMode = std::string(val);
	}
	
	int cv_dnn_BaseConvolutionLayer_getPropNumOutput_const(const cv::dnn::BaseConvolutionLayer* instance) {
			int ret = instance->numOutput;
			return ret;
	}
	
	void cv_dnn_BaseConvolutionLayer_setPropNumOutput_int(cv::dnn::BaseConvolutionLayer* instance, int val) {
			instance->numOutput = val;
	}
	
	cv::Algorithm* cv_BaseConvolutionLayer_to_Algorithm(cv::dnn::BaseConvolutionLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_BaseConvolutionLayer_to_Layer(cv::dnn::BaseConvolutionLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_BaseConvolutionLayer_delete(cv::dnn::BaseConvolutionLayer* instance) {
		delete instance;
	}
	bool cv_dnn_BatchNormLayer_getPropHasWeights_const(const cv::dnn::BatchNormLayer* instance) {
			bool ret = instance->hasWeights;
			return ret;
	}
	
	void cv_dnn_BatchNormLayer_setPropHasWeights_bool(cv::dnn::BatchNormLayer* instance, bool val) {
			instance->hasWeights = val;
	}
	
	bool cv_dnn_BatchNormLayer_getPropHasBias_const(const cv::dnn::BatchNormLayer* instance) {
			bool ret = instance->hasBias;
			return ret;
	}
	
	void cv_dnn_BatchNormLayer_setPropHasBias_bool(cv::dnn::BatchNormLayer* instance, bool val) {
			instance->hasBias = val;
	}
	
	float cv_dnn_BatchNormLayer_getPropEpsilon_const(const cv::dnn::BatchNormLayer* instance) {
			float ret = instance->epsilon;
			return ret;
	}
	
	void cv_dnn_BatchNormLayer_setPropEpsilon_float(cv::dnn::BatchNormLayer* instance, float val) {
			instance->epsilon = val;
	}
	
	cv::dnn::ActivationLayer* cv_BatchNormLayer_to_ActivationLayer(cv::dnn::BatchNormLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_BatchNormLayer_to_Algorithm(cv::dnn::BatchNormLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_BatchNormLayer_to_Layer(cv::dnn::BatchNormLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_BatchNormLayer_delete(cv::dnn::BatchNormLayer* instance) {
		delete instance;
	}
	void cv_dnn_BatchNormLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BatchNormLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BatchNormLayer> ret = cv::dnn::BatchNormLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::BatchNormLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BatchNormLayer>*>))
	}
	
	float cv_dnn_BatchNormLayerInt8_getPropInput_sc_const(const cv::dnn::BatchNormLayerInt8* instance) {
			float ret = instance->input_sc;
			return ret;
	}
	
	void cv_dnn_BatchNormLayerInt8_setPropInput_sc_float(cv::dnn::BatchNormLayerInt8* instance, float val) {
			instance->input_sc = val;
	}
	
	float cv_dnn_BatchNormLayerInt8_getPropOutput_sc_const(const cv::dnn::BatchNormLayerInt8* instance) {
			float ret = instance->output_sc;
			return ret;
	}
	
	void cv_dnn_BatchNormLayerInt8_setPropOutput_sc_float(cv::dnn::BatchNormLayerInt8* instance, float val) {
			instance->output_sc = val;
	}
	
	int cv_dnn_BatchNormLayerInt8_getPropInput_zp_const(const cv::dnn::BatchNormLayerInt8* instance) {
			int ret = instance->input_zp;
			return ret;
	}
	
	void cv_dnn_BatchNormLayerInt8_setPropInput_zp_int(cv::dnn::BatchNormLayerInt8* instance, int val) {
			instance->input_zp = val;
	}
	
	int cv_dnn_BatchNormLayerInt8_getPropOutput_zp_const(const cv::dnn::BatchNormLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_BatchNormLayerInt8_setPropOutput_zp_int(cv::dnn::BatchNormLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	cv::dnn::ActivationLayer* cv_BatchNormLayerInt8_to_ActivationLayer(cv::dnn::BatchNormLayerInt8* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_BatchNormLayerInt8_to_Algorithm(cv::dnn::BatchNormLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::BatchNormLayer* cv_BatchNormLayerInt8_to_BatchNormLayer(cv::dnn::BatchNormLayerInt8* instance) {
		return dynamic_cast<cv::dnn::BatchNormLayer*>(instance);
	}
	
	cv::dnn::Layer* cv_BatchNormLayerInt8_to_Layer(cv::dnn::BatchNormLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_BatchNormLayerInt8_delete(cv::dnn::BatchNormLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_BatchNormLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BatchNormLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BatchNormLayerInt8> ret = cv::dnn::BatchNormLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::BatchNormLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BatchNormLayerInt8>*>))
	}
	
	cv::Algorithm* cv_BlankLayer_to_Algorithm(cv::dnn::BlankLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_BlankLayer_to_Layer(cv::dnn::BlankLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_BlankLayer_delete(cv::dnn::BlankLayer* instance) {
		delete instance;
	}
	void cv_dnn_BlankLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::BlankLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_CeilLayer_to_ActivationLayer(cv::dnn::CeilLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_CeilLayer_to_Algorithm(cv::dnn::CeilLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CeilLayer_to_Layer(cv::dnn::CeilLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CeilLayer_delete(cv::dnn::CeilLayer* instance) {
		delete instance;
	}
	void cv_dnn_CeilLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CeilLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CeilLayer> ret = cv::dnn::CeilLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CeilLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CeilLayer>*>))
	}
	
	float cv_dnn_CeluLayer_getPropAlpha_const(const cv::dnn::CeluLayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_CeluLayer_setPropAlpha_float(cv::dnn::CeluLayer* instance, float val) {
			instance->alpha = val;
	}
	
	cv::dnn::ActivationLayer* cv_CeluLayer_to_ActivationLayer(cv::dnn::CeluLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_CeluLayer_to_Algorithm(cv::dnn::CeluLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CeluLayer_to_Layer(cv::dnn::CeluLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CeluLayer_delete(cv::dnn::CeluLayer* instance) {
		delete instance;
	}
	void cv_dnn_CeluLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CeluLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CeluLayer> ret = cv::dnn::CeluLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CeluLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CeluLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_ChannelsPReLULayer_to_ActivationLayer(cv::dnn::ChannelsPReLULayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ChannelsPReLULayer_to_Algorithm(cv::dnn::ChannelsPReLULayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ChannelsPReLULayer_to_Layer(cv::dnn::ChannelsPReLULayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ChannelsPReLULayer_delete(cv::dnn::ChannelsPReLULayer* instance) {
		delete instance;
	}
	void cv_dnn_ChannelsPReLULayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::ChannelsPReLULayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::dnn::Model* cv_ClassificationModel_to_Model(cv::dnn::ClassificationModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_ClassificationModel_delete(cv::dnn::ClassificationModel* instance) {
		delete instance;
	}
	void cv_dnn_ClassificationModel_ClassificationModel_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::ClassificationModel*>* ocvrs_return) {
		try {
			cv::dnn::ClassificationModel* ret = new cv::dnn::ClassificationModel(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::ClassificationModel*>))
	}
	
	void cv_dnn_ClassificationModel_ClassificationModel_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::ClassificationModel*>* ocvrs_return) {
		try {
			cv::dnn::ClassificationModel* ret = new cv::dnn::ClassificationModel(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::ClassificationModel*>))
	}
	
	void cv_dnn_ClassificationModel_classify_const__InputArrayR_intR_floatR(cv::dnn::ClassificationModel* instance, const cv::_InputArray* frame, int* classId, float* conf, Result_void* ocvrs_return) {
		try {
			instance->classify(*frame, *classId, *conf);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::Algorithm* cv_CompareLayer_to_Algorithm(cv::dnn::CompareLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CompareLayer_to_Layer(cv::dnn::CompareLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CompareLayer_delete(cv::dnn::CompareLayer* instance) {
		delete instance;
	}
	void cv_dnn_CompareLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::CompareLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	int cv_dnn_ConcatLayer_getPropAxis_const(const cv::dnn::ConcatLayer* instance) {
			int ret = instance->axis;
			return ret;
	}
	
	void cv_dnn_ConcatLayer_setPropAxis_int(cv::dnn::ConcatLayer* instance, int val) {
			instance->axis = val;
	}
	
	bool cv_dnn_ConcatLayer_getPropPadding_const(const cv::dnn::ConcatLayer* instance) {
			bool ret = instance->padding;
			return ret;
	}
	
	void cv_dnn_ConcatLayer_setPropPadding_bool(cv::dnn::ConcatLayer* instance, bool val) {
			instance->padding = val;
	}
	
	int cv_dnn_ConcatLayer_getPropPaddingValue_const(const cv::dnn::ConcatLayer* instance) {
			int ret = instance->paddingValue;
			return ret;
	}
	
	void cv_dnn_ConcatLayer_setPropPaddingValue_int(cv::dnn::ConcatLayer* instance, int val) {
			instance->paddingValue = val;
	}
	
	cv::Algorithm* cv_ConcatLayer_to_Algorithm(cv::dnn::ConcatLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ConcatLayer_to_Layer(cv::dnn::ConcatLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ConcatLayer_delete(cv::dnn::ConcatLayer* instance) {
		delete instance;
	}
	void cv_dnn_ConcatLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ConcatLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ConcatLayer> ret = cv::dnn::ConcatLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ConcatLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ConcatLayer>*>))
	}
	
	cv::Algorithm* cv_ConstLayer_to_Algorithm(cv::dnn::ConstLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ConstLayer_to_Layer(cv::dnn::ConstLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ConstLayer_delete(cv::dnn::ConstLayer* instance) {
		delete instance;
	}
	void cv_dnn_ConstLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::ConstLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::Algorithm* cv_ConvolutionLayer_to_Algorithm(cv::dnn::ConvolutionLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::BaseConvolutionLayer* cv_ConvolutionLayer_to_BaseConvolutionLayer(cv::dnn::ConvolutionLayer* instance) {
		return dynamic_cast<cv::dnn::BaseConvolutionLayer*>(instance);
	}
	
	cv::dnn::Layer* cv_ConvolutionLayer_to_Layer(cv::dnn::ConvolutionLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ConvolutionLayer_delete(cv::dnn::ConvolutionLayer* instance) {
		delete instance;
	}
	void cv_dnn_ConvolutionLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BaseConvolutionLayer> ret = cv::dnn::ConvolutionLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::BaseConvolutionLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>))
	}
	
	int cv_dnn_ConvolutionLayerInt8_getPropInput_zp_const(const cv::dnn::ConvolutionLayerInt8* instance) {
			int ret = instance->input_zp;
			return ret;
	}
	
	void cv_dnn_ConvolutionLayerInt8_setPropInput_zp_int(cv::dnn::ConvolutionLayerInt8* instance, int val) {
			instance->input_zp = val;
	}
	
	int cv_dnn_ConvolutionLayerInt8_getPropOutput_zp_const(const cv::dnn::ConvolutionLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_ConvolutionLayerInt8_setPropOutput_zp_int(cv::dnn::ConvolutionLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	float cv_dnn_ConvolutionLayerInt8_getPropOutput_sc_const(const cv::dnn::ConvolutionLayerInt8* instance) {
			float ret = instance->output_sc;
			return ret;
	}
	
	void cv_dnn_ConvolutionLayerInt8_setPropOutput_sc_float(cv::dnn::ConvolutionLayerInt8* instance, float val) {
			instance->output_sc = val;
	}
	
	cv::Algorithm* cv_ConvolutionLayerInt8_to_Algorithm(cv::dnn::ConvolutionLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::BaseConvolutionLayer* cv_ConvolutionLayerInt8_to_BaseConvolutionLayer(cv::dnn::ConvolutionLayerInt8* instance) {
		return dynamic_cast<cv::dnn::BaseConvolutionLayer*>(instance);
	}
	
	cv::dnn::Layer* cv_ConvolutionLayerInt8_to_Layer(cv::dnn::ConvolutionLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ConvolutionLayerInt8_delete(cv::dnn::ConvolutionLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_ConvolutionLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BaseConvolutionLayer> ret = cv::dnn::ConvolutionLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::BaseConvolutionLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>))
	}
	
	cv::Algorithm* cv_CorrelationLayer_to_Algorithm(cv::dnn::CorrelationLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CorrelationLayer_to_Layer(cv::dnn::CorrelationLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CorrelationLayer_delete(cv::dnn::CorrelationLayer* instance) {
		delete instance;
	}
	void cv_dnn_CorrelationLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CorrelationLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CorrelationLayer> ret = cv::dnn::CorrelationLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CorrelationLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CorrelationLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_CosLayer_to_ActivationLayer(cv::dnn::CosLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_CosLayer_to_Algorithm(cv::dnn::CosLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CosLayer_to_Layer(cv::dnn::CosLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CosLayer_delete(cv::dnn::CosLayer* instance) {
		delete instance;
	}
	void cv_dnn_CosLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CosLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CosLayer> ret = cv::dnn::CosLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CosLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CosLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_CoshLayer_to_ActivationLayer(cv::dnn::CoshLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_CoshLayer_to_Algorithm(cv::dnn::CoshLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CoshLayer_to_Layer(cv::dnn::CoshLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CoshLayer_delete(cv::dnn::CoshLayer* instance) {
		delete instance;
	}
	void cv_dnn_CoshLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CoshLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CoshLayer> ret = cv::dnn::CoshLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CoshLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CoshLayer>*>))
	}
	
	cv::Algorithm* cv_CropAndResizeLayer_to_Algorithm(cv::dnn::CropAndResizeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CropAndResizeLayer_to_Layer(cv::dnn::CropAndResizeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CropAndResizeLayer_delete(cv::dnn::CropAndResizeLayer* instance) {
		delete instance;
	}
	void cv_dnn_CropAndResizeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::CropAndResizeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::Algorithm* cv_CropLayer_to_Algorithm(cv::dnn::CropLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CropLayer_to_Layer(cv::dnn::CropLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CropLayer_delete(cv::dnn::CropLayer* instance) {
		delete instance;
	}
	void cv_dnn_CropLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::CropLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	int cv_dnn_CumSumLayer_getPropExclusive_const(const cv::dnn::CumSumLayer* instance) {
			int ret = instance->exclusive;
			return ret;
	}
	
	void cv_dnn_CumSumLayer_setPropExclusive_int(cv::dnn::CumSumLayer* instance, int val) {
			instance->exclusive = val;
	}
	
	int cv_dnn_CumSumLayer_getPropReverse_const(const cv::dnn::CumSumLayer* instance) {
			int ret = instance->reverse;
			return ret;
	}
	
	void cv_dnn_CumSumLayer_setPropReverse_int(cv::dnn::CumSumLayer* instance, int val) {
			instance->reverse = val;
	}
	
	cv::Algorithm* cv_CumSumLayer_to_Algorithm(cv::dnn::CumSumLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_CumSumLayer_to_Layer(cv::dnn::CumSumLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_CumSumLayer_delete(cv::dnn::CumSumLayer* instance) {
		delete instance;
	}
	void cv_dnn_CumSumLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::CumSumLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::CumSumLayer> ret = cv::dnn::CumSumLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::CumSumLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::CumSumLayer>*>))
	}
	
	cv::Algorithm* cv_DataAugmentationLayer_to_Algorithm(cv::dnn::DataAugmentationLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_DataAugmentationLayer_to_Layer(cv::dnn::DataAugmentationLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_DataAugmentationLayer_delete(cv::dnn::DataAugmentationLayer* instance) {
		delete instance;
	}
	void cv_dnn_DataAugmentationLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::DataAugmentationLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::DataAugmentationLayer> ret = cv::dnn::DataAugmentationLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::DataAugmentationLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::DataAugmentationLayer>*>))
	}
	
	cv::Algorithm* cv_DeconvolutionLayer_to_Algorithm(cv::dnn::DeconvolutionLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::BaseConvolutionLayer* cv_DeconvolutionLayer_to_BaseConvolutionLayer(cv::dnn::DeconvolutionLayer* instance) {
		return dynamic_cast<cv::dnn::BaseConvolutionLayer*>(instance);
	}
	
	cv::dnn::Layer* cv_DeconvolutionLayer_to_Layer(cv::dnn::DeconvolutionLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_DeconvolutionLayer_delete(cv::dnn::DeconvolutionLayer* instance) {
		delete instance;
	}
	void cv_dnn_DeconvolutionLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BaseConvolutionLayer> ret = cv::dnn::DeconvolutionLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::BaseConvolutionLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BaseConvolutionLayer>*>))
	}
	
	float cv_dnn_DequantizeLayer_getPropScale_const(const cv::dnn::DequantizeLayer* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_dnn_DequantizeLayer_setPropScale_float(cv::dnn::DequantizeLayer* instance, float val) {
			instance->scale = val;
	}
	
	int cv_dnn_DequantizeLayer_getPropZeropoint_const(const cv::dnn::DequantizeLayer* instance) {
			int ret = instance->zeropoint;
			return ret;
	}
	
	void cv_dnn_DequantizeLayer_setPropZeropoint_int(cv::dnn::DequantizeLayer* instance, int val) {
			instance->zeropoint = val;
	}
	
	cv::Algorithm* cv_DequantizeLayer_to_Algorithm(cv::dnn::DequantizeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_DequantizeLayer_to_Layer(cv::dnn::DequantizeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_DequantizeLayer_delete(cv::dnn::DequantizeLayer* instance) {
		delete instance;
	}
	void cv_dnn_DequantizeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::DequantizeLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::DequantizeLayer> ret = cv::dnn::DequantizeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::DequantizeLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::DequantizeLayer>*>))
	}
	
	cv::dnn::Model* cv_DetectionModel_to_Model(cv::dnn::DetectionModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_DetectionModel_delete(cv::dnn::DetectionModel* instance) {
		delete instance;
	}
	void cv_dnn_DetectionModel_DetectionModel_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::DetectionModel*>* ocvrs_return) {
		try {
			cv::dnn::DetectionModel* ret = new cv::dnn::DetectionModel(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DetectionModel*>))
	}
	
	void cv_dnn_DetectionModel_DetectionModel_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::DetectionModel*>* ocvrs_return) {
		try {
			cv::dnn::DetectionModel* ret = new cv::dnn::DetectionModel(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DetectionModel*>))
	}
	
	void cv_dnn_DetectionModel_DetectionModel(Result<cv::dnn::DetectionModel*>* ocvrs_return) {
		try {
			cv::dnn::DetectionModel* ret = new cv::dnn::DetectionModel();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DetectionModel*>))
	}
	
	void cv_dnn_DetectionModel_setNmsAcrossClasses_bool(cv::dnn::DetectionModel* instance, bool value, Result<cv::dnn::DetectionModel*>* ocvrs_return) {
		try {
			cv::dnn::DetectionModel ret = instance->setNmsAcrossClasses(value);
			Ok(new cv::dnn::DetectionModel(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DetectionModel*>))
	}
	
	void cv_dnn_DetectionModel_getNmsAcrossClasses(cv::dnn::DetectionModel* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getNmsAcrossClasses();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_DetectionModel_detect_const__InputArrayR_vector_int_R_vector_float_R_vector_Rect_R_float_float(cv::dnn::DetectionModel* instance, const cv::_InputArray* frame, std::vector<int>* classIds, std::vector<float>* confidences, std::vector<cv::Rect>* boxes, float confThreshold, float nmsThreshold, Result_void* ocvrs_return) {
		try {
			instance->detect(*frame, *classIds, *confidences, *boxes, confThreshold, nmsThreshold);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::Algorithm* cv_DetectionOutputLayer_to_Algorithm(cv::dnn::DetectionOutputLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_DetectionOutputLayer_to_Layer(cv::dnn::DetectionOutputLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_DetectionOutputLayer_delete(cv::dnn::DetectionOutputLayer* instance) {
		delete instance;
	}
	void cv_dnn_DetectionOutputLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::DetectionOutputLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::DetectionOutputLayer> ret = cv::dnn::DetectionOutputLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::DetectionOutputLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::DetectionOutputLayer>*>))
	}
	
	void cv_Dict_delete(cv::dnn::Dict* instance) {
		delete instance;
	}
	void cv_dnn_Dict_has_const_const_StringR(const cv::dnn::Dict* instance, const char* key, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->has(std::string(key));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Dict_ptr_const_StringR(cv::dnn::Dict* instance, const char* key, Result<cv::dnn::DictValue**>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = instance->ptr(std::string(key));
			Ok(new cv::dnn::DictValue*(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue**>))
	}
	
	void cv_dnn_Dict_ptr_const_const_StringR(const cv::dnn::Dict* instance, const char* key, Result<const cv::dnn::DictValue**>* ocvrs_return) {
		try {
			const cv::dnn::DictValue* ret = instance->ptr(std::string(key));
			Ok(new const cv::dnn::DictValue*(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<const cv::dnn::DictValue**>))
	}
	
	void cv_dnn_Dict_get_const_const_StringR(const cv::dnn::Dict* instance, const char* key, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			const cv::dnn::DictValue ret = instance->get(std::string(key));
			Ok(new const cv::dnn::DictValue(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_Dict_set_cv_String_const_StringR_const_StringR(cv::dnn::Dict* instance, const char* key, const char* value, Result<void*>* ocvrs_return) {
		try {
			const cv::String ret = instance->set<cv::String>(std::string(key), std::string(value));
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_Dict_set_cv_dnn_DictValue_const_StringR_const_DictValueR(cv::dnn::Dict* instance, const char* key, const cv::dnn::DictValue* value, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			const cv::dnn::DictValue ret = instance->set<cv::dnn::DictValue>(std::string(key), *value);
			Ok(new const cv::dnn::DictValue(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_Dict_set_double_const_StringR_const_doubleR(cv::dnn::Dict* instance, const char* key, const double* value, Result<double>* ocvrs_return) {
		try {
			const double ret = instance->set<double>(std::string(key), *value);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_dnn_Dict_set_int64_t_const_StringR_const_int64_tR(cv::dnn::Dict* instance, const char* key, const int64_t* value, Result<int64_t>* ocvrs_return) {
		try {
			const int64_t ret = instance->set<int64_t>(std::string(key), *value);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Dict_erase_const_StringR(cv::dnn::Dict* instance, const char* key, Result_void* ocvrs_return) {
		try {
			instance->erase(std::string(key));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_DictValue_delete(cv::dnn::DictValue* instance) {
		delete instance;
	}
	void cv_dnn_DictValue_DictValue_const_DictValueR(const cv::dnn::DictValue* r, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(*r);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_bool(bool i, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(i);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_int64_t(int64_t i, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(i);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_int(int i, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(i);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_unsigned_int(unsigned int p, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(p);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_double(double p, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(p);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_DictValue_const_charX(const char* s, Result<cv::dnn::DictValue*>* ocvrs_return) {
		try {
			cv::dnn::DictValue* ret = new cv::dnn::DictValue(s);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::DictValue*>))
	}
	
	void cv_dnn_DictValue_get_cv_String_const_int(const cv::dnn::DictValue* instance, int idx, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = instance->get<cv::String>(idx);
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_DictValue_get_double_const_int(const cv::dnn::DictValue* instance, int idx, Result<double>* ocvrs_return) {
		try {
			double ret = instance->get<double>(idx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_dnn_DictValue_get_int_const_int(const cv::dnn::DictValue* instance, int idx, Result<int>* ocvrs_return) {
		try {
			int ret = instance->get<int>(idx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_DictValue_get_int64_t_const_int(const cv::dnn::DictValue* instance, int idx, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->get<int64_t>(idx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_DictValue_size_const(const cv::dnn::DictValue* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->size();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_DictValue_isInt_const(const cv::dnn::DictValue* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isInt();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_DictValue_isString_const(const cv::dnn::DictValue* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isString();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_DictValue_isReal_const(const cv::dnn::DictValue* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isReal();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_DictValue_getIntValue_const_int(const cv::dnn::DictValue* instance, int idx, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getIntValue(idx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_DictValue_getRealValue_const_int(const cv::dnn::DictValue* instance, int idx, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRealValue(idx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_dnn_DictValue_getStringValue_const_int(const cv::dnn::DictValue* instance, int idx, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = instance->getStringValue(idx);
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	float cv_dnn_ELULayer_getPropAlpha_const(const cv::dnn::ELULayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_ELULayer_setPropAlpha_float(cv::dnn::ELULayer* instance, float val) {
			instance->alpha = val;
	}
	
	cv::dnn::ActivationLayer* cv_ELULayer_to_ActivationLayer(cv::dnn::ELULayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ELULayer_to_Algorithm(cv::dnn::ELULayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ELULayer_to_Layer(cv::dnn::ELULayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ELULayer_delete(cv::dnn::ELULayer* instance) {
		delete instance;
	}
	void cv_dnn_ELULayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ELULayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ELULayer> ret = cv::dnn::ELULayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ELULayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ELULayer>*>))
	}
	
	cv::Algorithm* cv_EltwiseLayer_to_Algorithm(cv::dnn::EltwiseLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_EltwiseLayer_to_Layer(cv::dnn::EltwiseLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_EltwiseLayer_delete(cv::dnn::EltwiseLayer* instance) {
		delete instance;
	}
	void cv_dnn_EltwiseLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::EltwiseLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::EltwiseLayer> ret = cv::dnn::EltwiseLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::EltwiseLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::EltwiseLayer>*>))
	}
	
	cv::Algorithm* cv_EltwiseLayerInt8_to_Algorithm(cv::dnn::EltwiseLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_EltwiseLayerInt8_to_Layer(cv::dnn::EltwiseLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_EltwiseLayerInt8_delete(cv::dnn::EltwiseLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_EltwiseLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::EltwiseLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::EltwiseLayerInt8> ret = cv::dnn::EltwiseLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::EltwiseLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::EltwiseLayerInt8>*>))
	}
	
	cv::dnn::ActivationLayer* cv_ErfLayer_to_ActivationLayer(cv::dnn::ErfLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ErfLayer_to_Algorithm(cv::dnn::ErfLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ErfLayer_to_Layer(cv::dnn::ErfLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ErfLayer_delete(cv::dnn::ErfLayer* instance) {
		delete instance;
	}
	void cv_dnn_ErfLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ErfLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ErfLayer> ret = cv::dnn::ErfLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ErfLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ErfLayer>*>))
	}
	
	float cv_dnn_ExpLayer_getPropBase_const(const cv::dnn::ExpLayer* instance) {
			float ret = instance->base;
			return ret;
	}
	
	void cv_dnn_ExpLayer_setPropBase_float(cv::dnn::ExpLayer* instance, float val) {
			instance->base = val;
	}
	
	float cv_dnn_ExpLayer_getPropScale_const(const cv::dnn::ExpLayer* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_dnn_ExpLayer_setPropScale_float(cv::dnn::ExpLayer* instance, float val) {
			instance->scale = val;
	}
	
	float cv_dnn_ExpLayer_getPropShift_const(const cv::dnn::ExpLayer* instance) {
			float ret = instance->shift;
			return ret;
	}
	
	void cv_dnn_ExpLayer_setPropShift_float(cv::dnn::ExpLayer* instance, float val) {
			instance->shift = val;
	}
	
	cv::dnn::ActivationLayer* cv_ExpLayer_to_ActivationLayer(cv::dnn::ExpLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ExpLayer_to_Algorithm(cv::dnn::ExpLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ExpLayer_to_Layer(cv::dnn::ExpLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ExpLayer_delete(cv::dnn::ExpLayer* instance) {
		delete instance;
	}
	void cv_dnn_ExpLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ExpLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ExpLayer> ret = cv::dnn::ExpLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ExpLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ExpLayer>*>))
	}
	
	cv::Algorithm* cv_FlattenLayer_to_Algorithm(cv::dnn::FlattenLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_FlattenLayer_to_Layer(cv::dnn::FlattenLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_FlattenLayer_delete(cv::dnn::FlattenLayer* instance) {
		delete instance;
	}
	void cv_dnn_FlattenLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::FlattenLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::FlattenLayer> ret = cv::dnn::FlattenLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::FlattenLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::FlattenLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_FloorLayer_to_ActivationLayer(cv::dnn::FloorLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_FloorLayer_to_Algorithm(cv::dnn::FloorLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_FloorLayer_to_Layer(cv::dnn::FloorLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_FloorLayer_delete(cv::dnn::FloorLayer* instance) {
		delete instance;
	}
	void cv_dnn_FloorLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::FloorLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::FloorLayer> ret = cv::dnn::FloorLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::FloorLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::FloorLayer>*>))
	}
	
	cv::Algorithm* cv_FlowWarpLayer_to_Algorithm(cv::dnn::FlowWarpLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_FlowWarpLayer_to_Layer(cv::dnn::FlowWarpLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_FlowWarpLayer_delete(cv::dnn::FlowWarpLayer* instance) {
		delete instance;
	}
	void cv_dnn_FlowWarpLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::FlowWarpLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::FlowWarpLayer> ret = cv::dnn::FlowWarpLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::FlowWarpLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::FlowWarpLayer>*>))
	}
	
	cv::Algorithm* cv_GRULayer_to_Algorithm(cv::dnn::GRULayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_GRULayer_to_Layer(cv::dnn::GRULayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_GRULayer_delete(cv::dnn::GRULayer* instance) {
		delete instance;
	}
	void cv_dnn_GRULayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::GRULayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::GRULayer> ret = cv::dnn::GRULayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::GRULayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::GRULayer>*>))
	}
	
	float cv_dnn_HardSigmoidLayer_getPropAlpha_const(const cv::dnn::HardSigmoidLayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_HardSigmoidLayer_setPropAlpha_float(cv::dnn::HardSigmoidLayer* instance, float val) {
			instance->alpha = val;
	}
	
	float cv_dnn_HardSigmoidLayer_getPropBeta_const(const cv::dnn::HardSigmoidLayer* instance) {
			float ret = instance->beta;
			return ret;
	}
	
	void cv_dnn_HardSigmoidLayer_setPropBeta_float(cv::dnn::HardSigmoidLayer* instance, float val) {
			instance->beta = val;
	}
	
	cv::dnn::ActivationLayer* cv_HardSigmoidLayer_to_ActivationLayer(cv::dnn::HardSigmoidLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_HardSigmoidLayer_to_Algorithm(cv::dnn::HardSigmoidLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_HardSigmoidLayer_to_Layer(cv::dnn::HardSigmoidLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_HardSigmoidLayer_delete(cv::dnn::HardSigmoidLayer* instance) {
		delete instance;
	}
	void cv_dnn_HardSigmoidLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::HardSigmoidLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::HardSigmoidLayer> ret = cv::dnn::HardSigmoidLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::HardSigmoidLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::HardSigmoidLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_HardSwishLayer_to_ActivationLayer(cv::dnn::HardSwishLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_HardSwishLayer_to_Algorithm(cv::dnn::HardSwishLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_HardSwishLayer_to_Layer(cv::dnn::HardSwishLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_HardSwishLayer_delete(cv::dnn::HardSwishLayer* instance) {
		delete instance;
	}
	void cv_dnn_HardSwishLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::HardSwishLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::HardSwishLayer> ret = cv::dnn::HardSwishLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::HardSwishLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::HardSwishLayer>*>))
	}
	
	int cv_dnn_InnerProductLayer_getPropAxis_const(const cv::dnn::InnerProductLayer* instance) {
			int ret = instance->axis;
			return ret;
	}
	
	void cv_dnn_InnerProductLayer_setPropAxis_int(cv::dnn::InnerProductLayer* instance, int val) {
			instance->axis = val;
	}
	
	cv::Algorithm* cv_InnerProductLayer_to_Algorithm(cv::dnn::InnerProductLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_InnerProductLayer_to_Layer(cv::dnn::InnerProductLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_InnerProductLayer_delete(cv::dnn::InnerProductLayer* instance) {
		delete instance;
	}
	void cv_dnn_InnerProductLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::InnerProductLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::InnerProductLayer> ret = cv::dnn::InnerProductLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::InnerProductLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::InnerProductLayer>*>))
	}
	
	int cv_dnn_InnerProductLayerInt8_getPropOutput_zp_const(const cv::dnn::InnerProductLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_InnerProductLayerInt8_setPropOutput_zp_int(cv::dnn::InnerProductLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	cv::Algorithm* cv_InnerProductLayerInt8_to_Algorithm(cv::dnn::InnerProductLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::InnerProductLayer* cv_InnerProductLayerInt8_to_InnerProductLayer(cv::dnn::InnerProductLayerInt8* instance) {
		return dynamic_cast<cv::dnn::InnerProductLayer*>(instance);
	}
	
	cv::dnn::Layer* cv_InnerProductLayerInt8_to_Layer(cv::dnn::InnerProductLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_InnerProductLayerInt8_delete(cv::dnn::InnerProductLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_InnerProductLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::InnerProductLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::InnerProductLayerInt8> ret = cv::dnn::InnerProductLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::InnerProductLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::InnerProductLayerInt8>*>))
	}
	
	cv::Algorithm* cv_InterpLayer_to_Algorithm(cv::dnn::InterpLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_InterpLayer_to_Layer(cv::dnn::InterpLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_InterpLayer_delete(cv::dnn::InterpLayer* instance) {
		delete instance;
	}
	void cv_dnn_InterpLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::InterpLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::dnn::Model* cv_KeypointsModel_to_Model(cv::dnn::KeypointsModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_KeypointsModel_delete(cv::dnn::KeypointsModel* instance) {
		delete instance;
	}
	void cv_dnn_KeypointsModel_KeypointsModel_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::KeypointsModel*>* ocvrs_return) {
		try {
			cv::dnn::KeypointsModel* ret = new cv::dnn::KeypointsModel(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::KeypointsModel*>))
	}
	
	void cv_dnn_KeypointsModel_KeypointsModel_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::KeypointsModel*>* ocvrs_return) {
		try {
			cv::dnn::KeypointsModel* ret = new cv::dnn::KeypointsModel(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::KeypointsModel*>))
	}
	
	void cv_dnn_KeypointsModel_estimate_const__InputArrayR_float(cv::dnn::KeypointsModel* instance, const cv::_InputArray* frame, float thresh, Result<std::vector<cv::Point2f>*>* ocvrs_return) {
		try {
			std::vector<cv::Point2f> ret = instance->estimate(*frame, thresh);
			Ok(new std::vector<cv::Point2f>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Point2f>*>))
	}
	
	int cv_dnn_LRNLayer_getPropType_const(const cv::dnn::LRNLayer* instance) {
			int ret = instance->type;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropType_int(cv::dnn::LRNLayer* instance, int val) {
			instance->type = val;
	}
	
	int cv_dnn_LRNLayer_getPropSize_const(const cv::dnn::LRNLayer* instance) {
			int ret = instance->size;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropSize_int(cv::dnn::LRNLayer* instance, int val) {
			instance->size = val;
	}
	
	float cv_dnn_LRNLayer_getPropAlpha_const(const cv::dnn::LRNLayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropAlpha_float(cv::dnn::LRNLayer* instance, float val) {
			instance->alpha = val;
	}
	
	float cv_dnn_LRNLayer_getPropBeta_const(const cv::dnn::LRNLayer* instance) {
			float ret = instance->beta;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropBeta_float(cv::dnn::LRNLayer* instance, float val) {
			instance->beta = val;
	}
	
	float cv_dnn_LRNLayer_getPropBias_const(const cv::dnn::LRNLayer* instance) {
			float ret = instance->bias;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropBias_float(cv::dnn::LRNLayer* instance, float val) {
			instance->bias = val;
	}
	
	bool cv_dnn_LRNLayer_getPropNormBySize_const(const cv::dnn::LRNLayer* instance) {
			bool ret = instance->normBySize;
			return ret;
	}
	
	void cv_dnn_LRNLayer_setPropNormBySize_bool(cv::dnn::LRNLayer* instance, bool val) {
			instance->normBySize = val;
	}
	
	cv::Algorithm* cv_LRNLayer_to_Algorithm(cv::dnn::LRNLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_LRNLayer_to_Layer(cv::dnn::LRNLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_LRNLayer_delete(cv::dnn::LRNLayer* instance) {
		delete instance;
	}
	void cv_dnn_LRNLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::LRNLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::LRNLayer> ret = cv::dnn::LRNLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::LRNLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::LRNLayer>*>))
	}
	
	void cv_dnn_LSTMLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::LSTMLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::LSTMLayer> ret = cv::dnn::LSTMLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::LSTMLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::LSTMLayer>*>))
	}
	
	void cv_dnn_LSTMLayer_setWeights_const_MatR_const_MatR_const_MatR(cv::dnn::LSTMLayer* instance, const cv::Mat* Wh, const cv::Mat* Wx, const cv::Mat* b, Result_void* ocvrs_return) {
		try {
			instance->setWeights(*Wh, *Wx, *b);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LSTMLayer_setOutShape_const_MatShapeR(cv::dnn::LSTMLayer* instance, const cv::dnn::MatShape* outTailShape, Result_void* ocvrs_return) {
		try {
			instance->setOutShape(*outTailShape);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LSTMLayer_setUseTimstampsDim_bool(cv::dnn::LSTMLayer* instance, bool use, Result_void* ocvrs_return) {
		try {
			instance->setUseTimstampsDim(use);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LSTMLayer_setProduceCellOutput_bool(cv::dnn::LSTMLayer* instance, bool produce, Result_void* ocvrs_return) {
		try {
			instance->setProduceCellOutput(produce);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LSTMLayer_inputNameToIndex_String(cv::dnn::LSTMLayer* instance, char* inputName, Result<int>* ocvrs_return) {
		try {
			int ret = instance->inputNameToIndex(std::string(inputName));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_LSTMLayer_outputNameToIndex_const_StringR(cv::dnn::LSTMLayer* instance, const char* outputName, Result<int>* ocvrs_return) {
		try {
			int ret = instance->outputNameToIndex(std::string(outputName));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	std::vector<cv::Mat>* cv_dnn_Layer_getPropBlobs_const(const cv::dnn::Layer* instance) {
			std::vector<cv::Mat> ret = instance->blobs;
			return new std::vector<cv::Mat>(ret);
	}
	
	void cv_dnn_Layer_setPropBlobs_vector_Mat_(cv::dnn::Layer* instance, std::vector<cv::Mat>* val) {
			instance->blobs = *val;
	}
	
	void* cv_dnn_Layer_getPropName_const(const cv::dnn::Layer* instance) {
			cv::String ret = instance->name;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_Layer_setPropName_String(cv::dnn::Layer* instance, char* val) {
			instance->name = std::string(val);
	}
	
	void* cv_dnn_Layer_getPropType_const(const cv::dnn::Layer* instance) {
			cv::String ret = instance->type;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_Layer_setPropType_String(cv::dnn::Layer* instance, char* val) {
			instance->type = std::string(val);
	}
	
	int cv_dnn_Layer_getPropPreferableTarget_const(const cv::dnn::Layer* instance) {
			int ret = instance->preferableTarget;
			return ret;
	}
	
	void cv_dnn_Layer_setPropPreferableTarget_int(cv::dnn::Layer* instance, int val) {
			instance->preferableTarget = val;
	}
	
	cv::dnn::AccumLayer* cv_Layer_to_AccumLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::AccumLayer*>(instance);
	}
	
	cv::dnn::ActivationLayer* cv_Layer_to_ActivationLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::dnn::ArgLayer* cv_Layer_to_ArgLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ArgLayer*>(instance);
	}
	
	cv::dnn::BaseConvolutionLayer* cv_Layer_to_BaseConvolutionLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::BaseConvolutionLayer*>(instance);
	}
	
	cv::dnn::BlankLayer* cv_Layer_to_BlankLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::BlankLayer*>(instance);
	}
	
	cv::dnn::CompareLayer* cv_Layer_to_CompareLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::CompareLayer*>(instance);
	}
	
	cv::dnn::ConcatLayer* cv_Layer_to_ConcatLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ConcatLayer*>(instance);
	}
	
	cv::dnn::ConstLayer* cv_Layer_to_ConstLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ConstLayer*>(instance);
	}
	
	cv::dnn::CorrelationLayer* cv_Layer_to_CorrelationLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::CorrelationLayer*>(instance);
	}
	
	cv::dnn::CropAndResizeLayer* cv_Layer_to_CropAndResizeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::CropAndResizeLayer*>(instance);
	}
	
	cv::dnn::CropLayer* cv_Layer_to_CropLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::CropLayer*>(instance);
	}
	
	cv::dnn::CumSumLayer* cv_Layer_to_CumSumLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::CumSumLayer*>(instance);
	}
	
	cv::dnn::DataAugmentationLayer* cv_Layer_to_DataAugmentationLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::DataAugmentationLayer*>(instance);
	}
	
	cv::dnn::DequantizeLayer* cv_Layer_to_DequantizeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::DequantizeLayer*>(instance);
	}
	
	cv::dnn::DetectionOutputLayer* cv_Layer_to_DetectionOutputLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::DetectionOutputLayer*>(instance);
	}
	
	cv::dnn::EltwiseLayer* cv_Layer_to_EltwiseLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::EltwiseLayer*>(instance);
	}
	
	cv::dnn::EltwiseLayerInt8* cv_Layer_to_EltwiseLayerInt8(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::EltwiseLayerInt8*>(instance);
	}
	
	cv::dnn::FlattenLayer* cv_Layer_to_FlattenLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::FlattenLayer*>(instance);
	}
	
	cv::dnn::FlowWarpLayer* cv_Layer_to_FlowWarpLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::FlowWarpLayer*>(instance);
	}
	
	cv::dnn::GRULayer* cv_Layer_to_GRULayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::GRULayer*>(instance);
	}
	
	cv::dnn::InnerProductLayer* cv_Layer_to_InnerProductLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::InnerProductLayer*>(instance);
	}
	
	cv::dnn::InterpLayer* cv_Layer_to_InterpLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::InterpLayer*>(instance);
	}
	
	cv::dnn::LRNLayer* cv_Layer_to_LRNLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::LRNLayer*>(instance);
	}
	
	cv::dnn::MVNLayer* cv_Layer_to_MVNLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::MVNLayer*>(instance);
	}
	
	cv::dnn::MaxUnpoolLayer* cv_Layer_to_MaxUnpoolLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::MaxUnpoolLayer*>(instance);
	}
	
	cv::dnn::NormalizeBBoxLayer* cv_Layer_to_NormalizeBBoxLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::NormalizeBBoxLayer*>(instance);
	}
	
	cv::dnn::PaddingLayer* cv_Layer_to_PaddingLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::PaddingLayer*>(instance);
	}
	
	cv::dnn::PermuteLayer* cv_Layer_to_PermuteLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::PermuteLayer*>(instance);
	}
	
	cv::dnn::PoolingLayer* cv_Layer_to_PoolingLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::PoolingLayer*>(instance);
	}
	
	cv::dnn::PriorBoxLayer* cv_Layer_to_PriorBoxLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::PriorBoxLayer*>(instance);
	}
	
	cv::dnn::ProposalLayer* cv_Layer_to_ProposalLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ProposalLayer*>(instance);
	}
	
	cv::dnn::QuantizeLayer* cv_Layer_to_QuantizeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::QuantizeLayer*>(instance);
	}
	
	cv::dnn::RegionLayer* cv_Layer_to_RegionLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::RegionLayer*>(instance);
	}
	
	cv::dnn::ReorgLayer* cv_Layer_to_ReorgLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ReorgLayer*>(instance);
	}
	
	cv::dnn::RequantizeLayer* cv_Layer_to_RequantizeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::RequantizeLayer*>(instance);
	}
	
	cv::dnn::ReshapeLayer* cv_Layer_to_ReshapeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ReshapeLayer*>(instance);
	}
	
	cv::dnn::ResizeLayer* cv_Layer_to_ResizeLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ResizeLayer*>(instance);
	}
	
	cv::dnn::ScaleLayer* cv_Layer_to_ScaleLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ScaleLayer*>(instance);
	}
	
	cv::dnn::ShiftLayer* cv_Layer_to_ShiftLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ShiftLayer*>(instance);
	}
	
	cv::dnn::ShiftLayerInt8* cv_Layer_to_ShiftLayerInt8(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ShiftLayerInt8*>(instance);
	}
	
	cv::dnn::ShuffleChannelLayer* cv_Layer_to_ShuffleChannelLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::ShuffleChannelLayer*>(instance);
	}
	
	cv::dnn::SliceLayer* cv_Layer_to_SliceLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::SliceLayer*>(instance);
	}
	
	cv::dnn::SoftmaxLayer* cv_Layer_to_SoftmaxLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::SoftmaxLayer*>(instance);
	}
	
	cv::dnn::SplitLayer* cv_Layer_to_SplitLayer(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::dnn::SplitLayer*>(instance);
	}
	
	cv::Algorithm* cv_Layer_to_Algorithm(cv::dnn::Layer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	void cv_Layer_delete(cv::dnn::Layer* instance) {
		delete instance;
	}
	void cv_dnn_Layer_finalize_const__InputArrayR_const__OutputArrayR(cv::dnn::Layer* instance, const cv::_InputArray* inputs, const cv::_OutputArray* outputs, Result_void* ocvrs_return) {
		try {
			instance->finalize(*inputs, *outputs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_forward_vector_MatX_R_vector_Mat_R_vector_Mat_R(cv::dnn::Layer* instance, std::vector<cv::Mat*>* input, std::vector<cv::Mat>* output, std::vector<cv::Mat>* internals, Result_void* ocvrs_return) {
		try {
			instance->forward(*input, *output, *internals);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_forward_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::dnn::Layer* instance, const cv::_InputArray* inputs, const cv::_OutputArray* outputs, const cv::_OutputArray* internals, Result_void* ocvrs_return) {
		try {
			instance->forward(*inputs, *outputs, *internals);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_tryQuantize_const_vector_vector_float__R_const_vector_vector_int__R_LayerParamsR(cv::dnn::Layer* instance, const std::vector<std::vector<float>>* scales, const std::vector<std::vector<int>>* zeropoints, cv::dnn::LayerParams* params, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->tryQuantize(*scales, *zeropoints, *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_forward_fallback_const__InputArrayR_const__OutputArrayR_const__OutputArrayR(cv::dnn::Layer* instance, const cv::_InputArray* inputs, const cv::_OutputArray* outputs, const cv::_OutputArray* internals, Result_void* ocvrs_return) {
		try {
			instance->forward_fallback(*inputs, *outputs, *internals);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_finalize_const_vector_Mat_R_vector_Mat_R(cv::dnn::Layer* instance, const std::vector<cv::Mat>* inputs, std::vector<cv::Mat>* outputs, Result_void* ocvrs_return) {
		try {
			instance->finalize(*inputs, *outputs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_finalize_const_vector_Mat_R(cv::dnn::Layer* instance, const std::vector<cv::Mat>* inputs, Result<std::vector<cv::Mat>*>* ocvrs_return) {
		try {
			std::vector<cv::Mat> ret = instance->finalize(*inputs);
			Ok(new std::vector<cv::Mat>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Mat>*>))
	}
	
	void cv_dnn_Layer_run_const_vector_Mat_R_vector_Mat_R_vector_Mat_R(cv::dnn::Layer* instance, const std::vector<cv::Mat>* inputs, std::vector<cv::Mat>* outputs, std::vector<cv::Mat>* internals, Result_void* ocvrs_return) {
		try {
			instance->run(*inputs, *outputs, *internals);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_inputNameToIndex_String(cv::dnn::Layer* instance, char* inputName, Result<int>* ocvrs_return) {
		try {
			int ret = instance->inputNameToIndex(std::string(inputName));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Layer_outputNameToIndex_const_StringR(cv::dnn::Layer* instance, const char* outputName, Result<int>* ocvrs_return) {
		try {
			int ret = instance->outputNameToIndex(std::string(outputName));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Layer_supportBackend_int(cv::dnn::Layer* instance, int backendId, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->supportBackend(backendId);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_initHalide_const_vector_Ptr_BackendWrapper__R(cv::dnn::Layer* instance, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initHalide(*inputs);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_initInfEngine_const_vector_Ptr_BackendWrapper__R(cv::dnn::Layer* instance, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initInfEngine(*inputs);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_initNgraph_const_vector_Ptr_BackendWrapper__R_const_vector_Ptr_BackendNode__R(cv::dnn::Layer* instance, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, const std::vector<cv::Ptr<cv::dnn::BackendNode>>* nodes, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initNgraph(*inputs, *nodes);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_initVkCom_const_vector_Ptr_BackendWrapper__R(cv::dnn::Layer* instance, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initVkCom(*inputs);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_initWebnn_const_vector_Ptr_BackendWrapper__R_const_vector_Ptr_BackendNode__R(cv::dnn::Layer* instance, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, const std::vector<cv::Ptr<cv::dnn::BackendNode>>* nodes, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initWebnn(*inputs, *nodes);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_initCUDA_voidX_const_vector_Ptr_BackendWrapper__R_const_vector_Ptr_BackendWrapper__R(cv::dnn::Layer* instance, void* context, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* inputs, const std::vector<cv::Ptr<cv::dnn::BackendWrapper>>* outputs, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->initCUDA(context, *inputs, *outputs);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_applyHalideScheduler_const_Ptr_BackendNode_R_const_vector_MatX_R_const_vector_Mat_R_int(const cv::dnn::Layer* instance, cv::Ptr<cv::dnn::BackendNode>* node, const std::vector<cv::Mat*>* inputs, const std::vector<cv::Mat>* outputs, int targetId, Result_void* ocvrs_return) {
		try {
			instance->applyHalideScheduler(*node, *inputs, *outputs, targetId);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_tryAttach_const_Ptr_BackendNode_R(cv::dnn::Layer* instance, const cv::Ptr<cv::dnn::BackendNode>* node, Result<cv::Ptr<cv::dnn::BackendNode>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::BackendNode> ret = instance->tryAttach(*node);
			Ok(new cv::Ptr<cv::dnn::BackendNode>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::BackendNode>*>))
	}
	
	void cv_dnn_Layer_setActivation_const_Ptr_ActivationLayer_R(cv::dnn::Layer* instance, const cv::Ptr<cv::dnn::ActivationLayer>* layer, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->setActivation(*layer);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_tryFuse_Ptr_Layer_R(cv::dnn::Layer* instance, cv::Ptr<cv::dnn::Layer>* top, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->tryFuse(*top);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_getScaleShift_const_MatR_MatR(const cv::dnn::Layer* instance, cv::Mat* scale, cv::Mat* shift, Result_void* ocvrs_return) {
		try {
			instance->getScaleShift(*scale, *shift);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_getScaleZeropoint_const_floatR_intR(const cv::dnn::Layer* instance, float* scale, int* zeropoint, Result_void* ocvrs_return) {
		try {
			instance->getScaleZeropoint(*scale, *zeropoint);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_unsetAttached(cv::dnn::Layer* instance, Result_void* ocvrs_return) {
		try {
			instance->unsetAttached();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Layer_getMemoryShapes_const_const_vector_MatShape_R_const_int_vector_MatShape_R_vector_MatShape_R(const cv::dnn::Layer* instance, const std::vector<cv::dnn::MatShape>* inputs, const int requiredOutputs, std::vector<cv::dnn::MatShape>* outputs, std::vector<cv::dnn::MatShape>* internals, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getMemoryShapes(*inputs, requiredOutputs, *outputs, *internals);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_getFLOPS_const_const_vector_MatShape_R_const_vector_MatShape_R(const cv::dnn::Layer* instance, const std::vector<cv::dnn::MatShape>* inputs, const std::vector<cv::dnn::MatShape>* outputs, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getFLOPS(*inputs, *outputs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Layer_updateMemoryShapes_const_vector_MatShape_R(cv::dnn::Layer* instance, const std::vector<cv::dnn::MatShape>* inputs, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->updateMemoryShapes(*inputs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Layer_Layer(Result<cv::dnn::Layer*>* ocvrs_return) {
		try {
			cv::dnn::Layer* ret = new cv::dnn::Layer();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Layer*>))
	}
	
	void cv_dnn_Layer_Layer_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::dnn::Layer*>* ocvrs_return) {
		try {
			cv::dnn::Layer* ret = new cv::dnn::Layer(*params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Layer*>))
	}
	
	void cv_dnn_Layer_setParamsFrom_const_LayerParamsR(cv::dnn::Layer* instance, const cv::dnn::LayerParams* params, Result_void* ocvrs_return) {
		try {
			instance->setParamsFrom(*params);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_LayerFactory_delete(cv::dnn::LayerFactory* instance) {
		delete instance;
	}
	void cv_dnn_LayerFactory_registerLayer_const_StringR_Constructor(const char* type, cv::dnn::LayerFactory::Constructor constructor, Result_void* ocvrs_return) {
		try {
			cv::dnn::LayerFactory::registerLayer(std::string(type), constructor);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LayerFactory_unregisterLayer_const_StringR(const char* type, Result_void* ocvrs_return) {
		try {
			cv::dnn::LayerFactory::unregisterLayer(std::string(type));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_LayerFactory_createLayerInstance_const_StringR_LayerParamsR(const char* type, cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::LayerFactory::createLayerInstance(std::string(type), *params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	std::vector<cv::Mat>* cv_dnn_LayerParams_getPropBlobs_const(const cv::dnn::LayerParams* instance) {
			std::vector<cv::Mat> ret = instance->blobs;
			return new std::vector<cv::Mat>(ret);
	}
	
	void cv_dnn_LayerParams_setPropBlobs_vector_Mat_(cv::dnn::LayerParams* instance, std::vector<cv::Mat>* val) {
			instance->blobs = *val;
	}
	
	void* cv_dnn_LayerParams_getPropName_const(const cv::dnn::LayerParams* instance) {
			cv::String ret = instance->name;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_LayerParams_setPropName_String(cv::dnn::LayerParams* instance, char* val) {
			instance->name = std::string(val);
	}
	
	void* cv_dnn_LayerParams_getPropType_const(const cv::dnn::LayerParams* instance) {
			cv::String ret = instance->type;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_LayerParams_setPropType_String(cv::dnn::LayerParams* instance, char* val) {
			instance->type = std::string(val);
	}
	
	cv::dnn::Dict* cv_LayerParams_to_Dict(cv::dnn::LayerParams* instance) {
		return dynamic_cast<cv::dnn::Dict*>(instance);
	}
	
	void cv_LayerParams_delete(cv::dnn::LayerParams* instance) {
		delete instance;
	}
	cv::dnn::ActivationLayer* cv_LogLayer_to_ActivationLayer(cv::dnn::LogLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_LogLayer_to_Algorithm(cv::dnn::LogLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_LogLayer_to_Layer(cv::dnn::LogLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_LogLayer_delete(cv::dnn::LogLayer* instance) {
		delete instance;
	}
	void cv_dnn_LogLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::LogLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::LogLayer> ret = cv::dnn::LogLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::LogLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::LogLayer>*>))
	}
	
	float cv_dnn_MVNLayer_getPropEps_const(const cv::dnn::MVNLayer* instance) {
			float ret = instance->eps;
			return ret;
	}
	
	void cv_dnn_MVNLayer_setPropEps_float(cv::dnn::MVNLayer* instance, float val) {
			instance->eps = val;
	}
	
	bool cv_dnn_MVNLayer_getPropNormVariance_const(const cv::dnn::MVNLayer* instance) {
			bool ret = instance->normVariance;
			return ret;
	}
	
	void cv_dnn_MVNLayer_setPropNormVariance_bool(cv::dnn::MVNLayer* instance, bool val) {
			instance->normVariance = val;
	}
	
	bool cv_dnn_MVNLayer_getPropAcrossChannels_const(const cv::dnn::MVNLayer* instance) {
			bool ret = instance->acrossChannels;
			return ret;
	}
	
	void cv_dnn_MVNLayer_setPropAcrossChannels_bool(cv::dnn::MVNLayer* instance, bool val) {
			instance->acrossChannels = val;
	}
	
	cv::Algorithm* cv_MVNLayer_to_Algorithm(cv::dnn::MVNLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_MVNLayer_to_Layer(cv::dnn::MVNLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_MVNLayer_delete(cv::dnn::MVNLayer* instance) {
		delete instance;
	}
	void cv_dnn_MVNLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::MVNLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::MVNLayer> ret = cv::dnn::MVNLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::MVNLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::MVNLayer>*>))
	}
	
	void cv_dnn_MaxUnpoolLayer_getPropPoolKernel_const(const cv::dnn::MaxUnpoolLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->poolKernel;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_MaxUnpoolLayer_setPropPoolKernel_Size(cv::dnn::MaxUnpoolLayer* instance, cv::Size* val) {
			instance->poolKernel = *val;
	}
	
	void cv_dnn_MaxUnpoolLayer_getPropPoolPad_const(const cv::dnn::MaxUnpoolLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->poolPad;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_MaxUnpoolLayer_setPropPoolPad_Size(cv::dnn::MaxUnpoolLayer* instance, cv::Size* val) {
			instance->poolPad = *val;
	}
	
	void cv_dnn_MaxUnpoolLayer_getPropPoolStride_const(const cv::dnn::MaxUnpoolLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->poolStride;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_MaxUnpoolLayer_setPropPoolStride_Size(cv::dnn::MaxUnpoolLayer* instance, cv::Size* val) {
			instance->poolStride = *val;
	}
	
	cv::Algorithm* cv_MaxUnpoolLayer_to_Algorithm(cv::dnn::MaxUnpoolLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_MaxUnpoolLayer_to_Layer(cv::dnn::MaxUnpoolLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_MaxUnpoolLayer_delete(cv::dnn::MaxUnpoolLayer* instance) {
		delete instance;
	}
	void cv_dnn_MaxUnpoolLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::MaxUnpoolLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::MaxUnpoolLayer> ret = cv::dnn::MaxUnpoolLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::MaxUnpoolLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::MaxUnpoolLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_MishLayer_to_ActivationLayer(cv::dnn::MishLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_MishLayer_to_Algorithm(cv::dnn::MishLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_MishLayer_to_Layer(cv::dnn::MishLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_MishLayer_delete(cv::dnn::MishLayer* instance) {
		delete instance;
	}
	void cv_dnn_MishLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::MishLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::MishLayer> ret = cv::dnn::MishLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::MishLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::MishLayer>*>))
	}
	
	void cv_Model_delete(cv::dnn::Model* instance) {
		delete instance;
	}
	void cv_dnn_Model_Model(Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model* ret = new cv::dnn::Model();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	cv::dnn::Model* cv_dnn_Model_Model_const_ModelR(const cv::dnn::Model* unnamed) {
			cv::dnn::Model* ret = new cv::dnn::Model(*unnamed);
			return ret;
	}
	
	cv::dnn::Model* cv_dnn_Model_Model_ModelR(cv::dnn::Model* unnamed) {
			cv::dnn::Model* ret = new cv::dnn::Model(*unnamed);
			return ret;
	}
	
	void cv_dnn_Model_Model_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model* ret = new cv::dnn::Model(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_Model_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model* ret = new cv::dnn::Model(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputSize_const_SizeR(cv::dnn::Model* instance, const cv::Size* size, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputSize(*size);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputSize_int_int(cv::dnn::Model* instance, int width, int height, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputSize(width, height);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputMean_const_ScalarR(cv::dnn::Model* instance, const cv::Scalar* mean, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputMean(*mean);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputScale_double(cv::dnn::Model* instance, double scale, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputScale(scale);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputCrop_bool(cv::dnn::Model* instance, bool crop, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputCrop(crop);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputSwapRB_bool(cv::dnn::Model* instance, bool swapRB, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setInputSwapRB(swapRB);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setInputParams_double_const_SizeR_const_ScalarR_bool_bool(cv::dnn::Model* instance, double scale, const cv::Size* size, const cv::Scalar* mean, bool swapRB, bool crop, Result_void* ocvrs_return) {
		try {
			instance->setInputParams(scale, *size, *mean, swapRB, crop);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Model_predict_const_const__InputArrayR_const__OutputArrayR(const cv::dnn::Model* instance, const cv::_InputArray* frame, const cv::_OutputArray* outs, Result_void* ocvrs_return) {
		try {
			instance->predict(*frame, *outs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Model_setPreferableBackend_Backend(cv::dnn::Model* instance, cv::dnn::Backend backendId, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setPreferableBackend(backendId);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_setPreferableTarget_Target(cv::dnn::Model* instance, cv::dnn::Target targetId, Result<cv::dnn::Model*>* ocvrs_return) {
		try {
			cv::dnn::Model ret = instance->setPreferableTarget(targetId);
			Ok(new cv::dnn::Model(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Model*>))
	}
	
	void cv_dnn_Model_getNetwork__const(const cv::dnn::Model* instance, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = instance->getNetwork_();
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Model_getNetwork_(cv::dnn::Model* instance, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = instance->getNetwork_();
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_Net_delete(cv::dnn::Net* instance) {
		delete instance;
	}
	void cv_dnn_Net_Net(Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net* ret = new cv::dnn::Net();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Net_readFromModelOptimizer_const_StringR_const_StringR(const char* xml, const char* bin, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::Net::readFromModelOptimizer(std::string(xml), std::string(bin));
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Net_readFromModelOptimizer_const_vector_unsigned_char_R_const_vector_unsigned_char_R(const std::vector<unsigned char>* bufferModelConfig, const std::vector<unsigned char>* bufferWeights, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::Net::readFromModelOptimizer(*bufferModelConfig, *bufferWeights);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Net_readFromModelOptimizer_const_unsigned_charX_size_t_const_unsigned_charX_size_t(const unsigned char* bufferModelConfigPtr, size_t bufferModelConfigSize, const unsigned char* bufferWeightsPtr, size_t bufferWeightsSize, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = cv::dnn::Net::readFromModelOptimizer(bufferModelConfigPtr, bufferModelConfigSize, bufferWeightsPtr, bufferWeightsSize);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Net_empty_const(const cv::dnn::Net* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->empty();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_dnn_Net_dump(cv::dnn::Net* instance, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = instance->dump();
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_Net_dumpToFile_const_StringR(cv::dnn::Net* instance, const char* path, Result_void* ocvrs_return) {
		try {
			instance->dumpToFile(std::string(path));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_addLayer_const_StringR_const_StringR_const_intR_LayerParamsR(cv::dnn::Net* instance, const char* name, const char* type, const int* dtype, cv::dnn::LayerParams* params, Result<int>* ocvrs_return) {
		try {
			int ret = instance->addLayer(std::string(name), std::string(type), *dtype, *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_addLayer_const_StringR_const_StringR_LayerParamsR(cv::dnn::Net* instance, const char* name, const char* type, cv::dnn::LayerParams* params, Result<int>* ocvrs_return) {
		try {
			int ret = instance->addLayer(std::string(name), std::string(type), *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_addLayerToPrev_const_StringR_const_StringR_const_intR_LayerParamsR(cv::dnn::Net* instance, const char* name, const char* type, const int* dtype, cv::dnn::LayerParams* params, Result<int>* ocvrs_return) {
		try {
			int ret = instance->addLayerToPrev(std::string(name), std::string(type), *dtype, *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_addLayerToPrev_const_StringR_const_StringR_LayerParamsR(cv::dnn::Net* instance, const char* name, const char* type, cv::dnn::LayerParams* params, Result<int>* ocvrs_return) {
		try {
			int ret = instance->addLayerToPrev(std::string(name), std::string(type), *params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_getLayerId_const_StringR(cv::dnn::Net* instance, const char* layer, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getLayerId(std::string(layer));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_getLayerNames_const(const cv::dnn::Net* instance, Result<std::vector<cv::String>*>* ocvrs_return) {
		try {
			std::vector<cv::String> ret = instance->getLayerNames();
			Ok(new std::vector<cv::String>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::String>*>))
	}
	
	void cv_dnn_Net_getLayer_LayerId(cv::dnn::Net* instance, cv::dnn::Net::LayerId* layerId, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = instance->getLayer(*layerId);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	void cv_dnn_Net_getLayerInputs_LayerId(cv::dnn::Net* instance, cv::dnn::Net::LayerId* layerId, Result<std::vector<cv::Ptr<cv::dnn::Layer>>*>* ocvrs_return) {
		try {
			std::vector<cv::Ptr<cv::dnn::Layer>> ret = instance->getLayerInputs(*layerId);
			Ok(new std::vector<cv::Ptr<cv::dnn::Layer>>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Ptr<cv::dnn::Layer>>*>))
	}
	
	void cv_dnn_Net_connect_String_String(cv::dnn::Net* instance, char* outPin, char* inpPin, Result_void* ocvrs_return) {
		try {
			instance->connect(std::string(outPin), std::string(inpPin));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_connect_int_int_int_int(cv::dnn::Net* instance, int outLayerId, int outNum, int inpLayerId, int inpNum, Result_void* ocvrs_return) {
		try {
			instance->connect(outLayerId, outNum, inpLayerId, inpNum);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setInputsNames_const_vector_String_R(cv::dnn::Net* instance, const std::vector<cv::String>* inputBlobNames, Result_void* ocvrs_return) {
		try {
			instance->setInputsNames(*inputBlobNames);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setInputShape_const_StringR_const_MatShapeR(cv::dnn::Net* instance, const char* inputName, const cv::dnn::MatShape* shape, Result_void* ocvrs_return) {
		try {
			instance->setInputShape(std::string(inputName), *shape);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_forward_const_StringR(cv::dnn::Net* instance, const char* outputName, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->forward(std::string(outputName));
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_Net_forwardAsync_const_StringR(cv::dnn::Net* instance, const char* outputName, Result<cv::AsyncArray*>* ocvrs_return) {
		try {
			cv::AsyncArray ret = instance->forwardAsync(std::string(outputName));
			Ok(new cv::AsyncArray(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::AsyncArray*>))
	}
	
	void cv_dnn_Net_forward_const__OutputArrayR_const_StringR(cv::dnn::Net* instance, const cv::_OutputArray* outputBlobs, const char* outputName, Result_void* ocvrs_return) {
		try {
			instance->forward(*outputBlobs, std::string(outputName));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_forward_const__OutputArrayR_const_vector_String_R(cv::dnn::Net* instance, const cv::_OutputArray* outputBlobs, const std::vector<cv::String>* outBlobNames, Result_void* ocvrs_return) {
		try {
			instance->forward(*outputBlobs, *outBlobNames);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_forward_vector_vector_Mat__R_const_vector_String_R(cv::dnn::Net* instance, std::vector<std::vector<cv::Mat>>* outputBlobs, const std::vector<cv::String>* outBlobNames, Result_void* ocvrs_return) {
		try {
			instance->forward(*outputBlobs, *outBlobNames);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_quantize_const__InputArrayR_int_int(cv::dnn::Net* instance, const cv::_InputArray* calibData, int inputsDtype, int outputsDtype, Result<cv::dnn::Net*>* ocvrs_return) {
		try {
			cv::dnn::Net ret = instance->quantize(*calibData, inputsDtype, outputsDtype);
			Ok(new cv::dnn::Net(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::Net*>))
	}
	
	void cv_dnn_Net_getInputDetails_const_vector_float_R_vector_int_R(const cv::dnn::Net* instance, std::vector<float>* scales, std::vector<int>* zeropoints, Result_void* ocvrs_return) {
		try {
			instance->getInputDetails(*scales, *zeropoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getOutputDetails_const_vector_float_R_vector_int_R(const cv::dnn::Net* instance, std::vector<float>* scales, std::vector<int>* zeropoints, Result_void* ocvrs_return) {
		try {
			instance->getOutputDetails(*scales, *zeropoints);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setHalideScheduler_const_StringR(cv::dnn::Net* instance, const char* scheduler, Result_void* ocvrs_return) {
		try {
			instance->setHalideScheduler(std::string(scheduler));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setPreferableBackend_int(cv::dnn::Net* instance, int backendId, Result_void* ocvrs_return) {
		try {
			instance->setPreferableBackend(backendId);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setPreferableTarget_int(cv::dnn::Net* instance, int targetId, Result_void* ocvrs_return) {
		try {
			instance->setPreferableTarget(targetId);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setInput_const__InputArrayR_const_StringR_double_const_ScalarR(cv::dnn::Net* instance, const cv::_InputArray* blob, const char* name, double scalefactor, const cv::Scalar* mean, Result_void* ocvrs_return) {
		try {
			instance->setInput(*blob, std::string(name), scalefactor, *mean);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_setParam_LayerId_int_const_MatR(cv::dnn::Net* instance, cv::dnn::Net::LayerId* layer, int numParam, const cv::Mat* blob, Result_void* ocvrs_return) {
		try {
			instance->setParam(*layer, numParam, *blob);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getParam_LayerId_int(cv::dnn::Net* instance, cv::dnn::Net::LayerId* layer, int numParam, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getParam(*layer, numParam);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_dnn_Net_getUnconnectedOutLayers_const(const cv::dnn::Net* instance, Result<std::vector<int>*>* ocvrs_return) {
		try {
			std::vector<int> ret = instance->getUnconnectedOutLayers();
			Ok(new std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_dnn_Net_getUnconnectedOutLayersNames_const(const cv::dnn::Net* instance, Result<std::vector<cv::String>*>* ocvrs_return) {
		try {
			std::vector<cv::String> ret = instance->getUnconnectedOutLayersNames();
			Ok(new std::vector<cv::String>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::String>*>))
	}
	
	void cv_dnn_Net_getLayersShapes_const_const_vector_MatShape_R_vector_int_R_vector_vector_MatShape__R_vector_vector_MatShape__R(const cv::dnn::Net* instance, const std::vector<cv::dnn::MatShape>* netInputShapes, std::vector<int>* layersIds, std::vector<std::vector<cv::dnn::MatShape>>* inLayersShapes, std::vector<std::vector<cv::dnn::MatShape>>* outLayersShapes, Result_void* ocvrs_return) {
		try {
			instance->getLayersShapes(*netInputShapes, *layersIds, *inLayersShapes, *outLayersShapes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getLayersShapes_const_const_MatShapeR_vector_int_R_vector_vector_MatShape__R_vector_vector_MatShape__R(const cv::dnn::Net* instance, const cv::dnn::MatShape* netInputShape, std::vector<int>* layersIds, std::vector<std::vector<cv::dnn::MatShape>>* inLayersShapes, std::vector<std::vector<cv::dnn::MatShape>>* outLayersShapes, Result_void* ocvrs_return) {
		try {
			instance->getLayersShapes(*netInputShape, *layersIds, *inLayersShapes, *outLayersShapes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getLayerShapes_const_const_MatShapeR_const_int_vector_MatShape_R_vector_MatShape_R(const cv::dnn::Net* instance, const cv::dnn::MatShape* netInputShape, const int layerId, std::vector<cv::dnn::MatShape>* inLayerShapes, std::vector<cv::dnn::MatShape>* outLayerShapes, Result_void* ocvrs_return) {
		try {
			instance->getLayerShapes(*netInputShape, layerId, *inLayerShapes, *outLayerShapes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getLayerShapes_const_const_vector_MatShape_R_const_int_vector_MatShape_R_vector_MatShape_R(const cv::dnn::Net* instance, const std::vector<cv::dnn::MatShape>* netInputShapes, const int layerId, std::vector<cv::dnn::MatShape>* inLayerShapes, std::vector<cv::dnn::MatShape>* outLayerShapes, Result_void* ocvrs_return) {
		try {
			instance->getLayerShapes(*netInputShapes, layerId, *inLayerShapes, *outLayerShapes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getFLOPS_const_const_vector_MatShape_R(const cv::dnn::Net* instance, const std::vector<cv::dnn::MatShape>* netInputShapes, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getFLOPS(*netInputShapes);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Net_getFLOPS_const_const_MatShapeR(const cv::dnn::Net* instance, const cv::dnn::MatShape* netInputShape, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getFLOPS(*netInputShape);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Net_getFLOPS_const_const_int_const_vector_MatShape_R(const cv::dnn::Net* instance, const int layerId, const std::vector<cv::dnn::MatShape>* netInputShapes, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getFLOPS(layerId, *netInputShapes);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Net_getFLOPS_const_const_int_const_MatShapeR(const cv::dnn::Net* instance, const int layerId, const cv::dnn::MatShape* netInputShape, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getFLOPS(layerId, *netInputShape);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	void cv_dnn_Net_getLayerTypes_const_vector_String_R(const cv::dnn::Net* instance, std::vector<cv::String>* layersTypes, Result_void* ocvrs_return) {
		try {
			instance->getLayerTypes(*layersTypes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getLayersCount_const_const_StringR(const cv::dnn::Net* instance, const char* layerType, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getLayersCount(std::string(layerType));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_vector_MatShape_R_size_tR_size_tR(const cv::dnn::Net* instance, const std::vector<cv::dnn::MatShape>* netInputShapes, size_t* weights, size_t* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(*netInputShapes, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_MatShapeR_size_tR_size_tR(const cv::dnn::Net* instance, const cv::dnn::MatShape* netInputShape, size_t* weights, size_t* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(*netInputShape, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_int_const_vector_MatShape_R_size_tR_size_tR(const cv::dnn::Net* instance, const int layerId, const std::vector<cv::dnn::MatShape>* netInputShapes, size_t* weights, size_t* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(layerId, *netInputShapes, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_int_const_MatShapeR_size_tR_size_tR(const cv::dnn::Net* instance, const int layerId, const cv::dnn::MatShape* netInputShape, size_t* weights, size_t* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(layerId, *netInputShape, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_vector_MatShape_R_vector_int_R_vector_size_t_R_vector_size_t_R(const cv::dnn::Net* instance, const std::vector<cv::dnn::MatShape>* netInputShapes, std::vector<int>* layerIds, std::vector<size_t>* weights, std::vector<size_t>* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(*netInputShapes, *layerIds, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getMemoryConsumption_const_const_MatShapeR_vector_int_R_vector_size_t_R_vector_size_t_R(const cv::dnn::Net* instance, const cv::dnn::MatShape* netInputShape, std::vector<int>* layerIds, std::vector<size_t>* weights, std::vector<size_t>* blobs, Result_void* ocvrs_return) {
		try {
			instance->getMemoryConsumption(*netInputShape, *layerIds, *weights, *blobs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_enableFusion_bool(cv::dnn::Net* instance, bool fusion, Result_void* ocvrs_return) {
		try {
			instance->enableFusion(fusion);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_Net_getPerfProfile_vector_double_R(cv::dnn::Net* instance, std::vector<double>* timings, Result<int64_t>* ocvrs_return) {
		try {
			int64_t ret = instance->getPerfProfile(*timings);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int64_t>))
	}
	
	float cv_dnn_NormalizeBBoxLayer_getPropPnorm_const(const cv::dnn::NormalizeBBoxLayer* instance) {
			float ret = instance->pnorm;
			return ret;
	}
	
	void cv_dnn_NormalizeBBoxLayer_setPropPnorm_float(cv::dnn::NormalizeBBoxLayer* instance, float val) {
			instance->pnorm = val;
	}
	
	float cv_dnn_NormalizeBBoxLayer_getPropEpsilon_const(const cv::dnn::NormalizeBBoxLayer* instance) {
			float ret = instance->epsilon;
			return ret;
	}
	
	void cv_dnn_NormalizeBBoxLayer_setPropEpsilon_float(cv::dnn::NormalizeBBoxLayer* instance, float val) {
			instance->epsilon = val;
	}
	
	bool cv_dnn_NormalizeBBoxLayer_getPropAcrossSpatial_const(const cv::dnn::NormalizeBBoxLayer* instance) {
			bool ret = instance->acrossSpatial;
			return ret;
	}
	
	void cv_dnn_NormalizeBBoxLayer_setPropAcrossSpatial_bool(cv::dnn::NormalizeBBoxLayer* instance, bool val) {
			instance->acrossSpatial = val;
	}
	
	cv::Algorithm* cv_NormalizeBBoxLayer_to_Algorithm(cv::dnn::NormalizeBBoxLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_NormalizeBBoxLayer_to_Layer(cv::dnn::NormalizeBBoxLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_NormalizeBBoxLayer_delete(cv::dnn::NormalizeBBoxLayer* instance) {
		delete instance;
	}
	void cv_dnn_NormalizeBBoxLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::NormalizeBBoxLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::NormalizeBBoxLayer> ret = cv::dnn::NormalizeBBoxLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::NormalizeBBoxLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::NormalizeBBoxLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_NotLayer_to_ActivationLayer(cv::dnn::NotLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_NotLayer_to_Algorithm(cv::dnn::NotLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_NotLayer_to_Layer(cv::dnn::NotLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_NotLayer_delete(cv::dnn::NotLayer* instance) {
		delete instance;
	}
	void cv_dnn_NotLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::NotLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::NotLayer> ret = cv::dnn::NotLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::NotLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::NotLayer>*>))
	}
	
	cv::Algorithm* cv_PaddingLayer_to_Algorithm(cv::dnn::PaddingLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PaddingLayer_to_Layer(cv::dnn::PaddingLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_PaddingLayer_delete(cv::dnn::PaddingLayer* instance) {
		delete instance;
	}
	void cv_dnn_PaddingLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PaddingLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PaddingLayer> ret = cv::dnn::PaddingLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::PaddingLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PaddingLayer>*>))
	}
	
	cv::Algorithm* cv_PermuteLayer_to_Algorithm(cv::dnn::PermuteLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PermuteLayer_to_Layer(cv::dnn::PermuteLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_PermuteLayer_delete(cv::dnn::PermuteLayer* instance) {
		delete instance;
	}
	void cv_dnn_PermuteLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PermuteLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PermuteLayer> ret = cv::dnn::PermuteLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::PermuteLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PermuteLayer>*>))
	}
	
	int cv_dnn_PoolingLayer_getPropType_const(const cv::dnn::PoolingLayer* instance) {
			int ret = instance->type;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropType_int(cv::dnn::PoolingLayer* instance, int val) {
			instance->type = val;
	}
	
	std::vector<size_t>* cv_dnn_PoolingLayer_getPropKernel_size_const(const cv::dnn::PoolingLayer* instance) {
			std::vector<size_t> ret = instance->kernel_size;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_PoolingLayer_setPropKernel_size_vector_size_t_(cv::dnn::PoolingLayer* instance, std::vector<size_t>* val) {
			instance->kernel_size = *val;
	}
	
	std::vector<size_t>* cv_dnn_PoolingLayer_getPropStrides_const(const cv::dnn::PoolingLayer* instance) {
			std::vector<size_t> ret = instance->strides;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_PoolingLayer_setPropStrides_vector_size_t_(cv::dnn::PoolingLayer* instance, std::vector<size_t>* val) {
			instance->strides = *val;
	}
	
	std::vector<size_t>* cv_dnn_PoolingLayer_getPropPads_begin_const(const cv::dnn::PoolingLayer* instance) {
			std::vector<size_t> ret = instance->pads_begin;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_PoolingLayer_setPropPads_begin_vector_size_t_(cv::dnn::PoolingLayer* instance, std::vector<size_t>* val) {
			instance->pads_begin = *val;
	}
	
	std::vector<size_t>* cv_dnn_PoolingLayer_getPropPads_end_const(const cv::dnn::PoolingLayer* instance) {
			std::vector<size_t> ret = instance->pads_end;
			return new std::vector<size_t>(ret);
	}
	
	void cv_dnn_PoolingLayer_setPropPads_end_vector_size_t_(cv::dnn::PoolingLayer* instance, std::vector<size_t>* val) {
			instance->pads_end = *val;
	}
	
	bool cv_dnn_PoolingLayer_getPropGlobalPooling_const(const cv::dnn::PoolingLayer* instance) {
			bool ret = instance->globalPooling;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropGlobalPooling_bool(cv::dnn::PoolingLayer* instance, bool val) {
			instance->globalPooling = val;
	}
	
	std::vector<bool>* cv_dnn_PoolingLayer_getPropIsGlobalPooling_const(const cv::dnn::PoolingLayer* instance) {
			std::vector<bool> ret = instance->isGlobalPooling;
			return new std::vector<bool>(ret);
	}
	
	void cv_dnn_PoolingLayer_setPropIsGlobalPooling_vector_bool_(cv::dnn::PoolingLayer* instance, std::vector<bool>* val) {
			instance->isGlobalPooling = *val;
	}
	
	bool cv_dnn_PoolingLayer_getPropComputeMaxIdx_const(const cv::dnn::PoolingLayer* instance) {
			bool ret = instance->computeMaxIdx;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropComputeMaxIdx_bool(cv::dnn::PoolingLayer* instance, bool val) {
			instance->computeMaxIdx = val;
	}
	
	void* cv_dnn_PoolingLayer_getPropPadMode_const(const cv::dnn::PoolingLayer* instance) {
			cv::String ret = instance->padMode;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_PoolingLayer_setPropPadMode_String(cv::dnn::PoolingLayer* instance, char* val) {
			instance->padMode = std::string(val);
	}
	
	bool cv_dnn_PoolingLayer_getPropCeilMode_const(const cv::dnn::PoolingLayer* instance) {
			bool ret = instance->ceilMode;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropCeilMode_bool(cv::dnn::PoolingLayer* instance, bool val) {
			instance->ceilMode = val;
	}
	
	bool cv_dnn_PoolingLayer_getPropAvePoolPaddedArea_const(const cv::dnn::PoolingLayer* instance) {
			bool ret = instance->avePoolPaddedArea;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropAvePoolPaddedArea_bool(cv::dnn::PoolingLayer* instance, bool val) {
			instance->avePoolPaddedArea = val;
	}
	
	void cv_dnn_PoolingLayer_getPropPooledSize_const(const cv::dnn::PoolingLayer* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->pooledSize;
			*ocvrs_return = ret;
	}
	
	void cv_dnn_PoolingLayer_setPropPooledSize_Size(cv::dnn::PoolingLayer* instance, cv::Size* val) {
			instance->pooledSize = *val;
	}
	
	float cv_dnn_PoolingLayer_getPropSpatialScale_const(const cv::dnn::PoolingLayer* instance) {
			float ret = instance->spatialScale;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropSpatialScale_float(cv::dnn::PoolingLayer* instance, float val) {
			instance->spatialScale = val;
	}
	
	int cv_dnn_PoolingLayer_getPropPsRoiOutChannels_const(const cv::dnn::PoolingLayer* instance) {
			int ret = instance->psRoiOutChannels;
			return ret;
	}
	
	void cv_dnn_PoolingLayer_setPropPsRoiOutChannels_int(cv::dnn::PoolingLayer* instance, int val) {
			instance->psRoiOutChannels = val;
	}
	
	cv::Algorithm* cv_PoolingLayer_to_Algorithm(cv::dnn::PoolingLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PoolingLayer_to_Layer(cv::dnn::PoolingLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_PoolingLayer_delete(cv::dnn::PoolingLayer* instance) {
		delete instance;
	}
	void cv_dnn_PoolingLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PoolingLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PoolingLayer> ret = cv::dnn::PoolingLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::PoolingLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PoolingLayer>*>))
	}
	
	int cv_dnn_PoolingLayerInt8_getPropInput_zp_const(const cv::dnn::PoolingLayerInt8* instance) {
			int ret = instance->input_zp;
			return ret;
	}
	
	void cv_dnn_PoolingLayerInt8_setPropInput_zp_int(cv::dnn::PoolingLayerInt8* instance, int val) {
			instance->input_zp = val;
	}
	
	int cv_dnn_PoolingLayerInt8_getPropOutput_zp_const(const cv::dnn::PoolingLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_PoolingLayerInt8_setPropOutput_zp_int(cv::dnn::PoolingLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	cv::Algorithm* cv_PoolingLayerInt8_to_Algorithm(cv::dnn::PoolingLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PoolingLayerInt8_to_Layer(cv::dnn::PoolingLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	cv::dnn::PoolingLayer* cv_PoolingLayerInt8_to_PoolingLayer(cv::dnn::PoolingLayerInt8* instance) {
		return dynamic_cast<cv::dnn::PoolingLayer*>(instance);
	}
	
	void cv_PoolingLayerInt8_delete(cv::dnn::PoolingLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_PoolingLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PoolingLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PoolingLayerInt8> ret = cv::dnn::PoolingLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::PoolingLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PoolingLayerInt8>*>))
	}
	
	float cv_dnn_PowerLayer_getPropPower_const(const cv::dnn::PowerLayer* instance) {
			float ret = instance->power;
			return ret;
	}
	
	void cv_dnn_PowerLayer_setPropPower_float(cv::dnn::PowerLayer* instance, float val) {
			instance->power = val;
	}
	
	float cv_dnn_PowerLayer_getPropScale_const(const cv::dnn::PowerLayer* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_dnn_PowerLayer_setPropScale_float(cv::dnn::PowerLayer* instance, float val) {
			instance->scale = val;
	}
	
	float cv_dnn_PowerLayer_getPropShift_const(const cv::dnn::PowerLayer* instance) {
			float ret = instance->shift;
			return ret;
	}
	
	void cv_dnn_PowerLayer_setPropShift_float(cv::dnn::PowerLayer* instance, float val) {
			instance->shift = val;
	}
	
	cv::dnn::ActivationLayer* cv_PowerLayer_to_ActivationLayer(cv::dnn::PowerLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_PowerLayer_to_Algorithm(cv::dnn::PowerLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PowerLayer_to_Layer(cv::dnn::PowerLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_PowerLayer_delete(cv::dnn::PowerLayer* instance) {
		delete instance;
	}
	void cv_dnn_PowerLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PowerLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PowerLayer> ret = cv::dnn::PowerLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::PowerLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PowerLayer>*>))
	}
	
	cv::Algorithm* cv_PriorBoxLayer_to_Algorithm(cv::dnn::PriorBoxLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_PriorBoxLayer_to_Layer(cv::dnn::PriorBoxLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_PriorBoxLayer_delete(cv::dnn::PriorBoxLayer* instance) {
		delete instance;
	}
	void cv_dnn_PriorBoxLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::PriorBoxLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::PriorBoxLayer> ret = cv::dnn::PriorBoxLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::PriorBoxLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::PriorBoxLayer>*>))
	}
	
	cv::Algorithm* cv_ProposalLayer_to_Algorithm(cv::dnn::ProposalLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ProposalLayer_to_Layer(cv::dnn::ProposalLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ProposalLayer_delete(cv::dnn::ProposalLayer* instance) {
		delete instance;
	}
	void cv_dnn_ProposalLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ProposalLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ProposalLayer> ret = cv::dnn::ProposalLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ProposalLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ProposalLayer>*>))
	}
	
	float cv_dnn_QuantizeLayer_getPropScale_const(const cv::dnn::QuantizeLayer* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_dnn_QuantizeLayer_setPropScale_float(cv::dnn::QuantizeLayer* instance, float val) {
			instance->scale = val;
	}
	
	int cv_dnn_QuantizeLayer_getPropZeropoint_const(const cv::dnn::QuantizeLayer* instance) {
			int ret = instance->zeropoint;
			return ret;
	}
	
	void cv_dnn_QuantizeLayer_setPropZeropoint_int(cv::dnn::QuantizeLayer* instance, int val) {
			instance->zeropoint = val;
	}
	
	cv::Algorithm* cv_QuantizeLayer_to_Algorithm(cv::dnn::QuantizeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_QuantizeLayer_to_Layer(cv::dnn::QuantizeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_QuantizeLayer_delete(cv::dnn::QuantizeLayer* instance) {
		delete instance;
	}
	void cv_dnn_QuantizeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::QuantizeLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::QuantizeLayer> ret = cv::dnn::QuantizeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::QuantizeLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::QuantizeLayer>*>))
	}
	
	void cv_dnn_RNNLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::RNNLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::RNNLayer> ret = cv::dnn::RNNLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::RNNLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::RNNLayer>*>))
	}
	
	void cv_dnn_RNNLayer_setWeights_const_MatR_const_MatR_const_MatR_const_MatR_const_MatR(cv::dnn::RNNLayer* instance, const cv::Mat* Wxh, const cv::Mat* bh, const cv::Mat* Whh, const cv::Mat* Who, const cv::Mat* bo, Result_void* ocvrs_return) {
		try {
			instance->setWeights(*Wxh, *bh, *Whh, *Who, *bo);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_RNNLayer_setProduceHiddenOutput_bool(cv::dnn::RNNLayer* instance, bool produce, Result_void* ocvrs_return) {
		try {
			instance->setProduceHiddenOutput(produce);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	float cv_dnn_ReLU6Layer_getPropMinValue_const(const cv::dnn::ReLU6Layer* instance) {
			float ret = instance->minValue;
			return ret;
	}
	
	void cv_dnn_ReLU6Layer_setPropMinValue_float(cv::dnn::ReLU6Layer* instance, float val) {
			instance->minValue = val;
	}
	
	float cv_dnn_ReLU6Layer_getPropMaxValue_const(const cv::dnn::ReLU6Layer* instance) {
			float ret = instance->maxValue;
			return ret;
	}
	
	void cv_dnn_ReLU6Layer_setPropMaxValue_float(cv::dnn::ReLU6Layer* instance, float val) {
			instance->maxValue = val;
	}
	
	cv::dnn::ActivationLayer* cv_ReLU6Layer_to_ActivationLayer(cv::dnn::ReLU6Layer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ReLU6Layer_to_Algorithm(cv::dnn::ReLU6Layer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ReLU6Layer_to_Layer(cv::dnn::ReLU6Layer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ReLU6Layer_delete(cv::dnn::ReLU6Layer* instance) {
		delete instance;
	}
	void cv_dnn_ReLU6Layer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ReLU6Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ReLU6Layer> ret = cv::dnn::ReLU6Layer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ReLU6Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ReLU6Layer>*>))
	}
	
	float cv_dnn_ReLULayer_getPropNegativeSlope_const(const cv::dnn::ReLULayer* instance) {
			float ret = instance->negativeSlope;
			return ret;
	}
	
	void cv_dnn_ReLULayer_setPropNegativeSlope_float(cv::dnn::ReLULayer* instance, float val) {
			instance->negativeSlope = val;
	}
	
	cv::dnn::ActivationLayer* cv_ReLULayer_to_ActivationLayer(cv::dnn::ReLULayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ReLULayer_to_Algorithm(cv::dnn::ReLULayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ReLULayer_to_Layer(cv::dnn::ReLULayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ReLULayer_delete(cv::dnn::ReLULayer* instance) {
		delete instance;
	}
	void cv_dnn_ReLULayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ReLULayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ReLULayer> ret = cv::dnn::ReLULayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ReLULayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ReLULayer>*>))
	}
	
	float cv_dnn_RegionLayer_getPropNmsThreshold_const(const cv::dnn::RegionLayer* instance) {
			float ret = instance->nmsThreshold;
			return ret;
	}
	
	void cv_dnn_RegionLayer_setPropNmsThreshold_float(cv::dnn::RegionLayer* instance, float val) {
			instance->nmsThreshold = val;
	}
	
	cv::Algorithm* cv_RegionLayer_to_Algorithm(cv::dnn::RegionLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_RegionLayer_to_Layer(cv::dnn::RegionLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_RegionLayer_delete(cv::dnn::RegionLayer* instance) {
		delete instance;
	}
	void cv_dnn_RegionLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::RegionLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::RegionLayer> ret = cv::dnn::RegionLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::RegionLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::RegionLayer>*>))
	}
	
	cv::Algorithm* cv_ReorgLayer_to_Algorithm(cv::dnn::ReorgLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ReorgLayer_to_Layer(cv::dnn::ReorgLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ReorgLayer_delete(cv::dnn::ReorgLayer* instance) {
		delete instance;
	}
	void cv_dnn_ReorgLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ReorgLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ReorgLayer> ret = cv::dnn::ReorgLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ReorgLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ReorgLayer>*>))
	}
	
	float cv_dnn_RequantizeLayer_getPropScale_const(const cv::dnn::RequantizeLayer* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_dnn_RequantizeLayer_setPropScale_float(cv::dnn::RequantizeLayer* instance, float val) {
			instance->scale = val;
	}
	
	float cv_dnn_RequantizeLayer_getPropShift_const(const cv::dnn::RequantizeLayer* instance) {
			float ret = instance->shift;
			return ret;
	}
	
	void cv_dnn_RequantizeLayer_setPropShift_float(cv::dnn::RequantizeLayer* instance, float val) {
			instance->shift = val;
	}
	
	cv::Algorithm* cv_RequantizeLayer_to_Algorithm(cv::dnn::RequantizeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_RequantizeLayer_to_Layer(cv::dnn::RequantizeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_RequantizeLayer_delete(cv::dnn::RequantizeLayer* instance) {
		delete instance;
	}
	void cv_dnn_RequantizeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::RequantizeLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::RequantizeLayer> ret = cv::dnn::RequantizeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::RequantizeLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::RequantizeLayer>*>))
	}
	
	cv::dnn::MatShape* cv_dnn_ReshapeLayer_getPropNewShapeDesc_const(const cv::dnn::ReshapeLayer* instance) {
			cv::dnn::MatShape ret = instance->newShapeDesc;
			return new cv::dnn::MatShape(ret);
	}
	
	void cv_dnn_ReshapeLayer_setPropNewShapeDesc_MatShape(cv::dnn::ReshapeLayer* instance, cv::dnn::MatShape* val) {
			instance->newShapeDesc = *val;
	}
	
	cv::Range* cv_dnn_ReshapeLayer_getPropNewShapeRange_const(const cv::dnn::ReshapeLayer* instance) {
			cv::Range ret = instance->newShapeRange;
			return new cv::Range(ret);
	}
	
	void cv_dnn_ReshapeLayer_setPropNewShapeRange_Range(cv::dnn::ReshapeLayer* instance, cv::Range* val) {
			instance->newShapeRange = *val;
	}
	
	cv::Algorithm* cv_ReshapeLayer_to_Algorithm(cv::dnn::ReshapeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ReshapeLayer_to_Layer(cv::dnn::ReshapeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ReshapeLayer_delete(cv::dnn::ReshapeLayer* instance) {
		delete instance;
	}
	void cv_dnn_ReshapeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ReshapeLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ReshapeLayer> ret = cv::dnn::ReshapeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ReshapeLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ReshapeLayer>*>))
	}
	
	cv::Algorithm* cv_ResizeLayer_to_Algorithm(cv::dnn::ResizeLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ResizeLayer_to_Layer(cv::dnn::ResizeLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ResizeLayer_delete(cv::dnn::ResizeLayer* instance) {
		delete instance;
	}
	void cv_dnn_ResizeLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ResizeLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ResizeLayer> ret = cv::dnn::ResizeLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ResizeLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ResizeLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_RoundLayer_to_ActivationLayer(cv::dnn::RoundLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_RoundLayer_to_Algorithm(cv::dnn::RoundLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_RoundLayer_to_Layer(cv::dnn::RoundLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_RoundLayer_delete(cv::dnn::RoundLayer* instance) {
		delete instance;
	}
	void cv_dnn_RoundLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::RoundLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::RoundLayer> ret = cv::dnn::RoundLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::RoundLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::RoundLayer>*>))
	}
	
	bool cv_dnn_ScaleLayer_getPropHasBias_const(const cv::dnn::ScaleLayer* instance) {
			bool ret = instance->hasBias;
			return ret;
	}
	
	void cv_dnn_ScaleLayer_setPropHasBias_bool(cv::dnn::ScaleLayer* instance, bool val) {
			instance->hasBias = val;
	}
	
	int cv_dnn_ScaleLayer_getPropAxis_const(const cv::dnn::ScaleLayer* instance) {
			int ret = instance->axis;
			return ret;
	}
	
	void cv_dnn_ScaleLayer_setPropAxis_int(cv::dnn::ScaleLayer* instance, int val) {
			instance->axis = val;
	}
	
	void* cv_dnn_ScaleLayer_getPropMode_const(const cv::dnn::ScaleLayer* instance) {
			cv::String ret = instance->mode;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_dnn_ScaleLayer_setPropMode_String(cv::dnn::ScaleLayer* instance, char* val) {
			instance->mode = std::string(val);
	}
	
	cv::Algorithm* cv_ScaleLayer_to_Algorithm(cv::dnn::ScaleLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ScaleLayer_to_Layer(cv::dnn::ScaleLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ScaleLayer_delete(cv::dnn::ScaleLayer* instance) {
		delete instance;
	}
	void cv_dnn_ScaleLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ScaleLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ScaleLayer> ret = cv::dnn::ScaleLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ScaleLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ScaleLayer>*>))
	}
	
	float cv_dnn_ScaleLayerInt8_getPropOutput_sc_const(const cv::dnn::ScaleLayerInt8* instance) {
			float ret = instance->output_sc;
			return ret;
	}
	
	void cv_dnn_ScaleLayerInt8_setPropOutput_sc_float(cv::dnn::ScaleLayerInt8* instance, float val) {
			instance->output_sc = val;
	}
	
	int cv_dnn_ScaleLayerInt8_getPropOutput_zp_const(const cv::dnn::ScaleLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_ScaleLayerInt8_setPropOutput_zp_int(cv::dnn::ScaleLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	cv::Algorithm* cv_ScaleLayerInt8_to_Algorithm(cv::dnn::ScaleLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ScaleLayerInt8_to_Layer(cv::dnn::ScaleLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	cv::dnn::ScaleLayer* cv_ScaleLayerInt8_to_ScaleLayer(cv::dnn::ScaleLayerInt8* instance) {
		return dynamic_cast<cv::dnn::ScaleLayer*>(instance);
	}
	
	void cv_ScaleLayerInt8_delete(cv::dnn::ScaleLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_ScaleLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ScaleLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ScaleLayerInt8> ret = cv::dnn::ScaleLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::ScaleLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ScaleLayerInt8>*>))
	}
	
	cv::dnn::Model* cv_SegmentationModel_to_Model(cv::dnn::SegmentationModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_SegmentationModel_delete(cv::dnn::SegmentationModel* instance) {
		delete instance;
	}
	void cv_dnn_SegmentationModel_SegmentationModel_const_StringR_const_StringR(const char* model, const char* config, Result<cv::dnn::SegmentationModel*>* ocvrs_return) {
		try {
			cv::dnn::SegmentationModel* ret = new cv::dnn::SegmentationModel(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::SegmentationModel*>))
	}
	
	void cv_dnn_SegmentationModel_SegmentationModel_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::SegmentationModel*>* ocvrs_return) {
		try {
			cv::dnn::SegmentationModel* ret = new cv::dnn::SegmentationModel(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::SegmentationModel*>))
	}
	
	void cv_dnn_SegmentationModel_segment_const__InputArrayR_const__OutputArrayR(cv::dnn::SegmentationModel* instance, const cv::_InputArray* frame, const cv::_OutputArray* mask, Result_void* ocvrs_return) {
		try {
			instance->segment(*frame, *mask);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	float cv_dnn_SeluLayer_getPropAlpha_const(const cv::dnn::SeluLayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_SeluLayer_setPropAlpha_float(cv::dnn::SeluLayer* instance, float val) {
			instance->alpha = val;
	}
	
	float cv_dnn_SeluLayer_getPropGamma_const(const cv::dnn::SeluLayer* instance) {
			float ret = instance->gamma;
			return ret;
	}
	
	void cv_dnn_SeluLayer_setPropGamma_float(cv::dnn::SeluLayer* instance, float val) {
			instance->gamma = val;
	}
	
	cv::dnn::ActivationLayer* cv_SeluLayer_to_ActivationLayer(cv::dnn::SeluLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SeluLayer_to_Algorithm(cv::dnn::SeluLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SeluLayer_to_Layer(cv::dnn::SeluLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SeluLayer_delete(cv::dnn::SeluLayer* instance) {
		delete instance;
	}
	void cv_dnn_SeluLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SeluLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SeluLayer> ret = cv::dnn::SeluLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SeluLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SeluLayer>*>))
	}
	
	cv::Algorithm* cv_ShiftLayer_to_Algorithm(cv::dnn::ShiftLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ShiftLayer_to_Layer(cv::dnn::ShiftLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ShiftLayer_delete(cv::dnn::ShiftLayer* instance) {
		delete instance;
	}
	void cv_dnn_ShiftLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::ShiftLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::Algorithm* cv_ShiftLayerInt8_to_Algorithm(cv::dnn::ShiftLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ShiftLayerInt8_to_Layer(cv::dnn::ShiftLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ShiftLayerInt8_delete(cv::dnn::ShiftLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_ShiftLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::ShiftLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	int cv_dnn_ShuffleChannelLayer_getPropGroup_const(const cv::dnn::ShuffleChannelLayer* instance) {
			int ret = instance->group;
			return ret;
	}
	
	void cv_dnn_ShuffleChannelLayer_setPropGroup_int(cv::dnn::ShuffleChannelLayer* instance, int val) {
			instance->group = val;
	}
	
	cv::Algorithm* cv_ShuffleChannelLayer_to_Algorithm(cv::dnn::ShuffleChannelLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ShuffleChannelLayer_to_Layer(cv::dnn::ShuffleChannelLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ShuffleChannelLayer_delete(cv::dnn::ShuffleChannelLayer* instance) {
		delete instance;
	}
	void cv_dnn_ShuffleChannelLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::Layer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::Layer> ret = cv::dnn::ShuffleChannelLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::Layer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::Layer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SigmoidLayer_to_ActivationLayer(cv::dnn::SigmoidLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SigmoidLayer_to_Algorithm(cv::dnn::SigmoidLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SigmoidLayer_to_Layer(cv::dnn::SigmoidLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SigmoidLayer_delete(cv::dnn::SigmoidLayer* instance) {
		delete instance;
	}
	void cv_dnn_SigmoidLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SigmoidLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SigmoidLayer> ret = cv::dnn::SigmoidLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SigmoidLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SigmoidLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SinLayer_to_ActivationLayer(cv::dnn::SinLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SinLayer_to_Algorithm(cv::dnn::SinLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SinLayer_to_Layer(cv::dnn::SinLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SinLayer_delete(cv::dnn::SinLayer* instance) {
		delete instance;
	}
	void cv_dnn_SinLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SinLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SinLayer> ret = cv::dnn::SinLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SinLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SinLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SinhLayer_to_ActivationLayer(cv::dnn::SinhLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SinhLayer_to_Algorithm(cv::dnn::SinhLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SinhLayer_to_Layer(cv::dnn::SinhLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SinhLayer_delete(cv::dnn::SinhLayer* instance) {
		delete instance;
	}
	void cv_dnn_SinhLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SinhLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SinhLayer> ret = cv::dnn::SinhLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SinhLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SinhLayer>*>))
	}
	
	std::vector<std::vector<cv::Range>>* cv_dnn_SliceLayer_getPropSliceRanges_const(const cv::dnn::SliceLayer* instance) {
			std::vector<std::vector<cv::Range>> ret = instance->sliceRanges;
			return new std::vector<std::vector<cv::Range>>(ret);
	}
	
	void cv_dnn_SliceLayer_setPropSliceRanges_vector_vector_Range__(cv::dnn::SliceLayer* instance, std::vector<std::vector<cv::Range>>* val) {
			instance->sliceRanges = *val;
	}
	
	std::vector<std::vector<int>>* cv_dnn_SliceLayer_getPropSliceSteps_const(const cv::dnn::SliceLayer* instance) {
			std::vector<std::vector<int>> ret = instance->sliceSteps;
			return new std::vector<std::vector<int>>(ret);
	}
	
	void cv_dnn_SliceLayer_setPropSliceSteps_vector_vector_int__(cv::dnn::SliceLayer* instance, std::vector<std::vector<int>>* val) {
			instance->sliceSteps = *val;
	}
	
	int cv_dnn_SliceLayer_getPropAxis_const(const cv::dnn::SliceLayer* instance) {
			int ret = instance->axis;
			return ret;
	}
	
	void cv_dnn_SliceLayer_setPropAxis_int(cv::dnn::SliceLayer* instance, int val) {
			instance->axis = val;
	}
	
	int cv_dnn_SliceLayer_getPropNum_split_const(const cv::dnn::SliceLayer* instance) {
			int ret = instance->num_split;
			return ret;
	}
	
	void cv_dnn_SliceLayer_setPropNum_split_int(cv::dnn::SliceLayer* instance, int val) {
			instance->num_split = val;
	}
	
	cv::Algorithm* cv_SliceLayer_to_Algorithm(cv::dnn::SliceLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SliceLayer_to_Layer(cv::dnn::SliceLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SliceLayer_delete(cv::dnn::SliceLayer* instance) {
		delete instance;
	}
	void cv_dnn_SliceLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SliceLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SliceLayer> ret = cv::dnn::SliceLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SliceLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SliceLayer>*>))
	}
	
	bool cv_dnn_SoftmaxLayer_getPropLogSoftMax_const(const cv::dnn::SoftmaxLayer* instance) {
			bool ret = instance->logSoftMax;
			return ret;
	}
	
	void cv_dnn_SoftmaxLayer_setPropLogSoftMax_bool(cv::dnn::SoftmaxLayer* instance, bool val) {
			instance->logSoftMax = val;
	}
	
	cv::Algorithm* cv_SoftmaxLayer_to_Algorithm(cv::dnn::SoftmaxLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SoftmaxLayer_to_Layer(cv::dnn::SoftmaxLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SoftmaxLayer_delete(cv::dnn::SoftmaxLayer* instance) {
		delete instance;
	}
	void cv_dnn_SoftmaxLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SoftmaxLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SoftmaxLayer> ret = cv::dnn::SoftmaxLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SoftmaxLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SoftmaxLayer>*>))
	}
	
	float cv_dnn_SoftmaxLayerInt8_getPropOutput_sc_const(const cv::dnn::SoftmaxLayerInt8* instance) {
			float ret = instance->output_sc;
			return ret;
	}
	
	void cv_dnn_SoftmaxLayerInt8_setPropOutput_sc_float(cv::dnn::SoftmaxLayerInt8* instance, float val) {
			instance->output_sc = val;
	}
	
	int cv_dnn_SoftmaxLayerInt8_getPropOutput_zp_const(const cv::dnn::SoftmaxLayerInt8* instance) {
			int ret = instance->output_zp;
			return ret;
	}
	
	void cv_dnn_SoftmaxLayerInt8_setPropOutput_zp_int(cv::dnn::SoftmaxLayerInt8* instance, int val) {
			instance->output_zp = val;
	}
	
	cv::Algorithm* cv_SoftmaxLayerInt8_to_Algorithm(cv::dnn::SoftmaxLayerInt8* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SoftmaxLayerInt8_to_Layer(cv::dnn::SoftmaxLayerInt8* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	cv::dnn::SoftmaxLayer* cv_SoftmaxLayerInt8_to_SoftmaxLayer(cv::dnn::SoftmaxLayerInt8* instance) {
		return dynamic_cast<cv::dnn::SoftmaxLayer*>(instance);
	}
	
	void cv_SoftmaxLayerInt8_delete(cv::dnn::SoftmaxLayerInt8* instance) {
		delete instance;
	}
	void cv_dnn_SoftmaxLayerInt8_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SoftmaxLayerInt8>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SoftmaxLayerInt8> ret = cv::dnn::SoftmaxLayerInt8::create(*params);
			Ok(new cv::Ptr<cv::dnn::SoftmaxLayerInt8>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SoftmaxLayerInt8>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SoftplusLayer_to_ActivationLayer(cv::dnn::SoftplusLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SoftplusLayer_to_Algorithm(cv::dnn::SoftplusLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SoftplusLayer_to_Layer(cv::dnn::SoftplusLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SoftplusLayer_delete(cv::dnn::SoftplusLayer* instance) {
		delete instance;
	}
	void cv_dnn_SoftplusLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SoftplusLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SoftplusLayer> ret = cv::dnn::SoftplusLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SoftplusLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SoftplusLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SoftsignLayer_to_ActivationLayer(cv::dnn::SoftsignLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SoftsignLayer_to_Algorithm(cv::dnn::SoftsignLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SoftsignLayer_to_Layer(cv::dnn::SoftsignLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SoftsignLayer_delete(cv::dnn::SoftsignLayer* instance) {
		delete instance;
	}
	void cv_dnn_SoftsignLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SoftsignLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SoftsignLayer> ret = cv::dnn::SoftsignLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SoftsignLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SoftsignLayer>*>))
	}
	
	int cv_dnn_SplitLayer_getPropOutputsCount_const(const cv::dnn::SplitLayer* instance) {
			int ret = instance->outputsCount;
			return ret;
	}
	
	void cv_dnn_SplitLayer_setPropOutputsCount_int(cv::dnn::SplitLayer* instance, int val) {
			instance->outputsCount = val;
	}
	
	cv::Algorithm* cv_SplitLayer_to_Algorithm(cv::dnn::SplitLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SplitLayer_to_Layer(cv::dnn::SplitLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SplitLayer_delete(cv::dnn::SplitLayer* instance) {
		delete instance;
	}
	void cv_dnn_SplitLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SplitLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SplitLayer> ret = cv::dnn::SplitLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SplitLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SplitLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SqrtLayer_to_ActivationLayer(cv::dnn::SqrtLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SqrtLayer_to_Algorithm(cv::dnn::SqrtLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SqrtLayer_to_Layer(cv::dnn::SqrtLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SqrtLayer_delete(cv::dnn::SqrtLayer* instance) {
		delete instance;
	}
	void cv_dnn_SqrtLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SqrtLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SqrtLayer> ret = cv::dnn::SqrtLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SqrtLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SqrtLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_SwishLayer_to_ActivationLayer(cv::dnn::SwishLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_SwishLayer_to_Algorithm(cv::dnn::SwishLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_SwishLayer_to_Layer(cv::dnn::SwishLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_SwishLayer_delete(cv::dnn::SwishLayer* instance) {
		delete instance;
	}
	void cv_dnn_SwishLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::SwishLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::SwishLayer> ret = cv::dnn::SwishLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::SwishLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::SwishLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_TanHLayer_to_ActivationLayer(cv::dnn::TanHLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_TanHLayer_to_Algorithm(cv::dnn::TanHLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_TanHLayer_to_Layer(cv::dnn::TanHLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_TanHLayer_delete(cv::dnn::TanHLayer* instance) {
		delete instance;
	}
	void cv_dnn_TanHLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::TanHLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::TanHLayer> ret = cv::dnn::TanHLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::TanHLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::TanHLayer>*>))
	}
	
	cv::dnn::ActivationLayer* cv_TanLayer_to_ActivationLayer(cv::dnn::TanLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_TanLayer_to_Algorithm(cv::dnn::TanLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_TanLayer_to_Layer(cv::dnn::TanLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_TanLayer_delete(cv::dnn::TanLayer* instance) {
		delete instance;
	}
	void cv_dnn_TanLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::TanLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::TanLayer> ret = cv::dnn::TanLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::TanLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::TanLayer>*>))
	}
	
	cv::dnn::Model* cv_TextDetectionModel_to_Model(cv::dnn::TextDetectionModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_TextDetectionModel_delete(cv::dnn::TextDetectionModel* instance) {
		delete instance;
	}
	void cv_dnn_TextDetectionModel_detect_const_const__InputArrayR_vector_vector_Point__R_vector_float_R(const cv::dnn::TextDetectionModel* instance, const cv::_InputArray* frame, std::vector<std::vector<cv::Point>>* detections, std::vector<float>* confidences, Result_void* ocvrs_return) {
		try {
			instance->detect(*frame, *detections, *confidences);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_TextDetectionModel_detect_const_const__InputArrayR_vector_vector_Point__R(const cv::dnn::TextDetectionModel* instance, const cv::_InputArray* frame, std::vector<std::vector<cv::Point>>* detections, Result_void* ocvrs_return) {
		try {
			instance->detect(*frame, *detections);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_TextDetectionModel_detectTextRectangles_const_const__InputArrayR_vector_RotatedRect_R_vector_float_R(const cv::dnn::TextDetectionModel* instance, const cv::_InputArray* frame, std::vector<cv::RotatedRect>* detections, std::vector<float>* confidences, Result_void* ocvrs_return) {
		try {
			instance->detectTextRectangles(*frame, *detections, *confidences);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_dnn_TextDetectionModel_detectTextRectangles_const_const__InputArrayR_vector_RotatedRect_R(const cv::dnn::TextDetectionModel* instance, const cv::_InputArray* frame, std::vector<cv::RotatedRect>* detections, Result_void* ocvrs_return) {
		try {
			instance->detectTextRectangles(*frame, *detections);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	cv::dnn::Model* cv_TextDetectionModel_DB_to_Model(cv::dnn::TextDetectionModel_DB* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	cv::dnn::TextDetectionModel* cv_TextDetectionModel_DB_to_TextDetectionModel(cv::dnn::TextDetectionModel_DB* instance) {
		return dynamic_cast<cv::dnn::TextDetectionModel*>(instance);
	}
	
	void cv_TextDetectionModel_DB_delete(cv::dnn::TextDetectionModel_DB* instance) {
		delete instance;
	}
	void cv_dnn_TextDetectionModel_DB_TextDetectionModel_DB(Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB* ret = new cv::dnn::TextDetectionModel_DB();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_TextDetectionModel_DB_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB* ret = new cv::dnn::TextDetectionModel_DB(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_TextDetectionModel_DB_const_stringR_const_stringR(const char* model, const char* config, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB* ret = new cv::dnn::TextDetectionModel_DB(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_setBinaryThreshold_float(cv::dnn::TextDetectionModel_DB* instance, float binaryThreshold, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB ret = instance->setBinaryThreshold(binaryThreshold);
			Ok(new cv::dnn::TextDetectionModel_DB(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_getBinaryThreshold_const(const cv::dnn::TextDetectionModel_DB* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getBinaryThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_dnn_TextDetectionModel_DB_setPolygonThreshold_float(cv::dnn::TextDetectionModel_DB* instance, float polygonThreshold, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB ret = instance->setPolygonThreshold(polygonThreshold);
			Ok(new cv::dnn::TextDetectionModel_DB(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_getPolygonThreshold_const(const cv::dnn::TextDetectionModel_DB* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getPolygonThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_dnn_TextDetectionModel_DB_setUnclipRatio_double(cv::dnn::TextDetectionModel_DB* instance, double unclipRatio, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB ret = instance->setUnclipRatio(unclipRatio);
			Ok(new cv::dnn::TextDetectionModel_DB(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_getUnclipRatio_const(const cv::dnn::TextDetectionModel_DB* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getUnclipRatio();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_dnn_TextDetectionModel_DB_setMaxCandidates_int(cv::dnn::TextDetectionModel_DB* instance, int maxCandidates, Result<cv::dnn::TextDetectionModel_DB*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_DB ret = instance->setMaxCandidates(maxCandidates);
			Ok(new cv::dnn::TextDetectionModel_DB(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_DB*>))
	}
	
	void cv_dnn_TextDetectionModel_DB_getMaxCandidates_const(const cv::dnn::TextDetectionModel_DB* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxCandidates();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	cv::dnn::Model* cv_TextDetectionModel_EAST_to_Model(cv::dnn::TextDetectionModel_EAST* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	cv::dnn::TextDetectionModel* cv_TextDetectionModel_EAST_to_TextDetectionModel(cv::dnn::TextDetectionModel_EAST* instance) {
		return dynamic_cast<cv::dnn::TextDetectionModel*>(instance);
	}
	
	void cv_TextDetectionModel_EAST_delete(cv::dnn::TextDetectionModel_EAST* instance) {
		delete instance;
	}
	void cv_dnn_TextDetectionModel_EAST_TextDetectionModel_EAST(Result<cv::dnn::TextDetectionModel_EAST*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_EAST* ret = new cv::dnn::TextDetectionModel_EAST();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_EAST*>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_TextDetectionModel_EAST_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::TextDetectionModel_EAST*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_EAST* ret = new cv::dnn::TextDetectionModel_EAST(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_EAST*>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_TextDetectionModel_EAST_const_stringR_const_stringR(const char* model, const char* config, Result<cv::dnn::TextDetectionModel_EAST*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_EAST* ret = new cv::dnn::TextDetectionModel_EAST(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_EAST*>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_setConfidenceThreshold_float(cv::dnn::TextDetectionModel_EAST* instance, float confThreshold, Result<cv::dnn::TextDetectionModel_EAST*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_EAST ret = instance->setConfidenceThreshold(confThreshold);
			Ok(new cv::dnn::TextDetectionModel_EAST(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_EAST*>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_getConfidenceThreshold_const(const cv::dnn::TextDetectionModel_EAST* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getConfidenceThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_setNMSThreshold_float(cv::dnn::TextDetectionModel_EAST* instance, float nmsThreshold, Result<cv::dnn::TextDetectionModel_EAST*>* ocvrs_return) {
		try {
			cv::dnn::TextDetectionModel_EAST ret = instance->setNMSThreshold(nmsThreshold);
			Ok(new cv::dnn::TextDetectionModel_EAST(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextDetectionModel_EAST*>))
	}
	
	void cv_dnn_TextDetectionModel_EAST_getNMSThreshold_const(const cv::dnn::TextDetectionModel_EAST* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getNMSThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	cv::dnn::Model* cv_TextRecognitionModel_to_Model(cv::dnn::TextRecognitionModel* instance) {
		return dynamic_cast<cv::dnn::Model*>(instance);
	}
	
	void cv_TextRecognitionModel_delete(cv::dnn::TextRecognitionModel* instance) {
		delete instance;
	}
	void cv_dnn_TextRecognitionModel_TextRecognitionModel(Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel* ret = new cv::dnn::TextRecognitionModel();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_TextRecognitionModel_const_NetR(const cv::dnn::Net* network, Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel* ret = new cv::dnn::TextRecognitionModel(*network);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_TextRecognitionModel_const_stringR_const_stringR(const char* model, const char* config, Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel* ret = new cv::dnn::TextRecognitionModel(std::string(model), std::string(config));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_setDecodeType_const_stringR(cv::dnn::TextRecognitionModel* instance, const char* decodeType, Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel ret = instance->setDecodeType(std::string(decodeType));
			Ok(new cv::dnn::TextRecognitionModel(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_getDecodeType_const(const cv::dnn::TextRecognitionModel* instance, Result<void*>* ocvrs_return) {
		try {
			const std::string ret = instance->getDecodeType();
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_TextRecognitionModel_setDecodeOptsCTCPrefixBeamSearch_int_int(cv::dnn::TextRecognitionModel* instance, int beamSize, int vocPruneSize, Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel ret = instance->setDecodeOptsCTCPrefixBeamSearch(beamSize, vocPruneSize);
			Ok(new cv::dnn::TextRecognitionModel(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_setVocabulary_const_vector_string_R(cv::dnn::TextRecognitionModel* instance, const std::vector<std::string>* vocabulary, Result<cv::dnn::TextRecognitionModel*>* ocvrs_return) {
		try {
			cv::dnn::TextRecognitionModel ret = instance->setVocabulary(*vocabulary);
			Ok(new cv::dnn::TextRecognitionModel(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::TextRecognitionModel*>))
	}
	
	void cv_dnn_TextRecognitionModel_getVocabulary_const(const cv::dnn::TextRecognitionModel* instance, Result<std::vector<std::string>*>* ocvrs_return) {
		try {
			const std::vector<std::string> ret = instance->getVocabulary();
			Ok(new const std::vector<std::string>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<std::string>*>))
	}
	
	void cv_dnn_TextRecognitionModel_recognize_const_const__InputArrayR(const cv::dnn::TextRecognitionModel* instance, const cv::_InputArray* frame, Result<void*>* ocvrs_return) {
		try {
			std::string ret = instance->recognize(*frame);
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_dnn_TextRecognitionModel_recognize_const_const__InputArrayR_const__InputArrayR_vector_string_R(const cv::dnn::TextRecognitionModel* instance, const cv::_InputArray* frame, const cv::_InputArray* roiRects, std::vector<std::string>* results, Result_void* ocvrs_return) {
		try {
			instance->recognize(*frame, *roiRects, *results);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	float cv_dnn_ThresholdedReluLayer_getPropAlpha_const(const cv::dnn::ThresholdedReluLayer* instance) {
			float ret = instance->alpha;
			return ret;
	}
	
	void cv_dnn_ThresholdedReluLayer_setPropAlpha_float(cv::dnn::ThresholdedReluLayer* instance, float val) {
			instance->alpha = val;
	}
	
	cv::dnn::ActivationLayer* cv_ThresholdedReluLayer_to_ActivationLayer(cv::dnn::ThresholdedReluLayer* instance) {
		return dynamic_cast<cv::dnn::ActivationLayer*>(instance);
	}
	
	cv::Algorithm* cv_ThresholdedReluLayer_to_Algorithm(cv::dnn::ThresholdedReluLayer* instance) {
		return dynamic_cast<cv::Algorithm*>(instance);
	}
	
	cv::dnn::Layer* cv_ThresholdedReluLayer_to_Layer(cv::dnn::ThresholdedReluLayer* instance) {
		return dynamic_cast<cv::dnn::Layer*>(instance);
	}
	
	void cv_ThresholdedReluLayer_delete(cv::dnn::ThresholdedReluLayer* instance) {
		delete instance;
	}
	void cv_dnn_ThresholdedReluLayer_create_const_LayerParamsR(const cv::dnn::LayerParams* params, Result<cv::Ptr<cv::dnn::ThresholdedReluLayer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::dnn::ThresholdedReluLayer> ret = cv::dnn::ThresholdedReluLayer::create(*params);
			Ok(new cv::Ptr<cv::dnn::ThresholdedReluLayer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::dnn::ThresholdedReluLayer>*>))
	}
	
	cv::Range* cv__Range_to_Range(cv::dnn::_Range* instance) {
		return dynamic_cast<cv::Range*>(instance);
	}
	
	void cv__Range_delete(cv::dnn::_Range* instance) {
		delete instance;
	}
	void cv_dnn__Range__Range_const_RangeR(const cv::Range* r, Result<cv::dnn::_Range*>* ocvrs_return) {
		try {
			cv::dnn::_Range* ret = new cv::dnn::_Range(*r);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::_Range*>))
	}
	
	void cv_dnn__Range__Range_int_int(int start_, int size_, Result<cv::dnn::_Range*>* ocvrs_return) {
		try {
			cv::dnn::_Range* ret = new cv::dnn::_Range(start_, size_);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::dnn::_Range*>))
	}
	
}
