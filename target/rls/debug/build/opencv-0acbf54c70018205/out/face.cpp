#include "face.hpp"
#include "face_types.hpp"

extern "C" {
	void cv_face_createFacemarkAAM(Result<cv::Ptr<cv::face::Facemark>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::Facemark> ret = cv::face::createFacemarkAAM();
			Ok(new cv::Ptr<cv::face::Facemark>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::Facemark>*>))
	}
	
	void cv_face_createFacemarkKazemi(Result<cv::Ptr<cv::face::Facemark>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::Facemark> ret = cv::face::createFacemarkKazemi();
			Ok(new cv::Ptr<cv::face::Facemark>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::Facemark>*>))
	}
	
	void cv_face_createFacemarkLBF(Result<cv::Ptr<cv::face::Facemark>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::Facemark> ret = cv::face::createFacemarkLBF();
			Ok(new cv::Ptr<cv::face::Facemark>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::Facemark>*>))
	}
	
	void cv_face_drawFacemarks_const__InputOutputArrayR_const__InputArrayR_Scalar(const cv::_InputOutputArray* image, const cv::_InputArray* points, cv::Scalar* color, Result_void* ocvrs_return) {
		try {
			cv::face::drawFacemarks(*image, *points, *color);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_getFacesHAAR_const__InputArrayR_const__OutputArrayR_const_StringR(const cv::_InputArray* image, const cv::_OutputArray* faces, const char* face_cascade_name, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::getFacesHAAR(*image, *faces, std::string(face_cascade_name));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_getFaces_const__InputArrayR_const__OutputArrayR_CParamsX(const cv::_InputArray* image, const cv::_OutputArray* faces, cv::face::CParams* params, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::getFaces(*image, *faces, params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_loadDatasetList_String_String_vector_String_R_vector_String_R(char* imageList, char* annotationList, std::vector<cv::String>* images, std::vector<cv::String>* annotations, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::loadDatasetList(std::string(imageList), std::string(annotationList), *images, *annotations);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_loadFacePoints_String_const__OutputArrayR_float(char* filename, const cv::_OutputArray* points, float offset, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::loadFacePoints(std::string(filename), *points, offset);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_loadTrainingData_String_String_vector_String_R_const__OutputArrayR_float(char* imageList, char* groundTruth, std::vector<cv::String>* images, const cv::_OutputArray* facePoints, float offset, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::loadTrainingData(std::string(imageList), std::string(groundTruth), *images, *facePoints, offset);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_loadTrainingData_String_vector_String_R_const__OutputArrayR_char_float(char* filename, std::vector<cv::String>* images, const cv::_OutputArray* facePoints, char delim, float offset, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::loadTrainingData(std::string(filename), *images, *facePoints, delim, offset);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_loadTrainingData_vector_String__vector_vector_Point2f__R_vector_String_R(std::vector<cv::String>* filename, std::vector<std::vector<cv::Point2f>>* trainlandmarks, std::vector<cv::String>* trainimages, Result<bool>* ocvrs_return) {
		try {
			bool ret = cv::face::loadTrainingData(*filename, *trainlandmarks, *trainimages);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_BIF_getNumBands_const(const cv::face::BIF* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNumBands();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_BIF_getNumRotations_const(const cv::face::BIF* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNumRotations();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_BIF_compute_const_const__InputArrayR_const__OutputArrayR(const cv::face::BIF* instance, const cv::_InputArray* image, const cv::_OutputArray* features, Result_void* ocvrs_return) {
		try {
			instance->compute(*image, *features);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_BIF_create_int_int(int num_bands, int num_rotations, Result<cv::Ptr<cv::face::BIF>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::BIF> ret = cv::face::BIF::create(num_bands, num_rotations);
			Ok(new cv::Ptr<cv::face::BIF>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::BIF>*>))
	}
	
	void cv_face_BasicFaceRecognizer_getNumComponents_const(const cv::face::BasicFaceRecognizer* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNumComponents();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_BasicFaceRecognizer_setNumComponents_int(cv::face::BasicFaceRecognizer* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setNumComponents(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_BasicFaceRecognizer_getThreshold_const(const cv::face::BasicFaceRecognizer* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_face_BasicFaceRecognizer_setThreshold_double(cv::face::BasicFaceRecognizer* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setThreshold(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_BasicFaceRecognizer_getProjections_const(const cv::face::BasicFaceRecognizer* instance, Result<std::vector<cv::Mat>*>* ocvrs_return) {
		try {
			std::vector<cv::Mat> ret = instance->getProjections();
			Ok(new std::vector<cv::Mat>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Mat>*>))
	}
	
	void cv_face_BasicFaceRecognizer_getLabels_const(const cv::face::BasicFaceRecognizer* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getLabels();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_face_BasicFaceRecognizer_getEigenValues_const(const cv::face::BasicFaceRecognizer* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getEigenValues();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_face_BasicFaceRecognizer_getEigenVectors_const(const cv::face::BasicFaceRecognizer* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getEigenVectors();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_face_BasicFaceRecognizer_getMean_const(const cv::face::BasicFaceRecognizer* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getMean();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_face_BasicFaceRecognizer_read_const_FileNodeR(cv::face::BasicFaceRecognizer* instance, const cv::FileNode* fn, Result_void* ocvrs_return) {
		try {
			instance->read(*fn);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_BasicFaceRecognizer_write_const_FileStorageR(const cv::face::BasicFaceRecognizer* instance, cv::FileStorage* fs, Result_void* ocvrs_return) {
		try {
			instance->write(*fs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_BasicFaceRecognizer_empty_const(const cv::face::BasicFaceRecognizer* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->empty();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void* cv_face_CParams_getPropCascade_const(const cv::face::CParams* instance) {
			cv::String ret = instance->cascade;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_face_CParams_setPropCascade_String(cv::face::CParams* instance, char* val) {
			instance->cascade = std::string(val);
	}
	
	double cv_face_CParams_getPropScaleFactor_const(const cv::face::CParams* instance) {
			double ret = instance->scaleFactor;
			return ret;
	}
	
	void cv_face_CParams_setPropScaleFactor_double(cv::face::CParams* instance, double val) {
			instance->scaleFactor = val;
	}
	
	int cv_face_CParams_getPropMinNeighbors_const(const cv::face::CParams* instance) {
			int ret = instance->minNeighbors;
			return ret;
	}
	
	void cv_face_CParams_setPropMinNeighbors_int(cv::face::CParams* instance, int val) {
			instance->minNeighbors = val;
	}
	
	void cv_face_CParams_getPropMinSize_const(const cv::face::CParams* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->minSize;
			*ocvrs_return = ret;
	}
	
	void cv_face_CParams_setPropMinSize_Size(cv::face::CParams* instance, cv::Size* val) {
			instance->minSize = *val;
	}
	
	void cv_face_CParams_getPropMaxSize_const(const cv::face::CParams* instance, cv::Size* ocvrs_return) {
			cv::Size ret = instance->maxSize;
			*ocvrs_return = ret;
	}
	
	void cv_face_CParams_setPropMaxSize_Size(cv::face::CParams* instance, cv::Size* val) {
			instance->maxSize = *val;
	}
	
	cv::CascadeClassifier* cv_face_CParams_getPropFace_cascade_const(const cv::face::CParams* instance) {
			cv::CascadeClassifier ret = instance->face_cascade;
			return new cv::CascadeClassifier(ret);
	}
	
	void cv_face_CParams_setPropFace_cascade_CascadeClassifier(cv::face::CParams* instance, cv::CascadeClassifier* val) {
			instance->face_cascade = *val;
	}
	
	void cv_CParams_delete(cv::face::CParams* instance) {
		delete instance;
	}
	void cv_face_CParams_CParams_String_double_int_Size_Size(char* cascade_model, double sf, int minN, cv::Size* minSz, cv::Size* maxSz, Result<cv::face::CParams*>* ocvrs_return) {
		try {
			cv::face::CParams* ret = new cv::face::CParams(std::string(cascade_model), sf, minN, *minSz, *maxSz);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::CParams*>))
	}
	
	void cv_face_EigenFaceRecognizer_create_int_double(int num_components, double threshold, Result<cv::Ptr<cv::face::EigenFaceRecognizer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::EigenFaceRecognizer> ret = cv::face::EigenFaceRecognizer::create(num_components, threshold);
			Ok(new cv::Ptr<cv::face::EigenFaceRecognizer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::EigenFaceRecognizer>*>))
	}
	
	void cv_face_FaceRecognizer_train_const__InputArrayR_const__InputArrayR(cv::face::FaceRecognizer* instance, const cv::_InputArray* src, const cv::_InputArray* labels, Result_void* ocvrs_return) {
		try {
			instance->train(*src, *labels);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_update_const__InputArrayR_const__InputArrayR(cv::face::FaceRecognizer* instance, const cv::_InputArray* src, const cv::_InputArray* labels, Result_void* ocvrs_return) {
		try {
			instance->update(*src, *labels);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_predict_const_const__InputArrayR(const cv::face::FaceRecognizer* instance, const cv::_InputArray* src, Result<int>* ocvrs_return) {
		try {
			int ret = instance->predict(*src);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_FaceRecognizer_predict_const_const__InputArrayR_intR_doubleR(const cv::face::FaceRecognizer* instance, const cv::_InputArray* src, int* label, double* confidence, Result_void* ocvrs_return) {
		try {
			instance->predict(*src, *label, *confidence);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_predict_const_const__InputArrayR_Ptr_PredictCollector_(const cv::face::FaceRecognizer* instance, const cv::_InputArray* src, cv::Ptr<cv::face::PredictCollector>* collector, Result_void* ocvrs_return) {
		try {
			instance->predict(*src, *collector);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_write_const_const_StringR(const cv::face::FaceRecognizer* instance, const char* filename, Result_void* ocvrs_return) {
		try {
			instance->write(std::string(filename));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_read_const_StringR(cv::face::FaceRecognizer* instance, const char* filename, Result_void* ocvrs_return) {
		try {
			instance->read(std::string(filename));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_write_const_FileStorageR(const cv::face::FaceRecognizer* instance, cv::FileStorage* fs, Result_void* ocvrs_return) {
		try {
			instance->write(*fs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_read_const_FileNodeR(cv::face::FaceRecognizer* instance, const cv::FileNode* fn, Result_void* ocvrs_return) {
		try {
			instance->read(*fn);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_empty_const(const cv::face::FaceRecognizer* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->empty();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FaceRecognizer_setLabelInfo_int_const_StringR(cv::face::FaceRecognizer* instance, int label, const char* strInfo, Result_void* ocvrs_return) {
		try {
			instance->setLabelInfo(label, std::string(strInfo));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FaceRecognizer_getLabelInfo_const_int(const cv::face::FaceRecognizer* instance, int label, Result<void*>* ocvrs_return) {
		try {
			cv::String ret = instance->getLabelInfo(label);
			Ok(ocvrs_create_string(ret.c_str()), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<void*>))
	}
	
	void cv_face_FaceRecognizer_getLabelsByString_const_const_StringR(const cv::face::FaceRecognizer* instance, const char* str, Result<std::vector<int>*>* ocvrs_return) {
		try {
			std::vector<int> ret = instance->getLabelsByString(std::string(str));
			Ok(new std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_face_FaceRecognizer_getThreshold_const(const cv::face::FaceRecognizer* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_face_FaceRecognizer_setThreshold_double(cv::face::FaceRecognizer* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setThreshold(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_Facemark_loadModel_String(cv::face::Facemark* instance, char* model, Result_void* ocvrs_return) {
		try {
			instance->loadModel(std::string(model));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_Facemark_fit_const__InputArrayR_const__InputArrayR_const__OutputArrayR(cv::face::Facemark* instance, const cv::_InputArray* image, const cv::_InputArray* faces, const cv::_OutputArray* landmarks, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->fit(*image, *faces, *landmarks);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkAAM_fitConfig_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const_vector_Config_R(cv::face::FacemarkAAM* instance, const cv::_InputArray* image, const cv::_InputArray* roi, const cv::_OutputArray* _landmarks, const std::vector<cv::face::FacemarkAAM::Config>* runtime_params, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->fitConfig(*image, *roi, *_landmarks, *runtime_params);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkAAM_create_const_ParamsR(const cv::face::FacemarkAAM::Params* parameters, Result<cv::Ptr<cv::face::FacemarkAAM>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::FacemarkAAM> ret = cv::face::FacemarkAAM::create(*parameters);
			Ok(new cv::Ptr<cv::face::FacemarkAAM>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::FacemarkAAM>*>))
	}
	
	cv::Mat* cv_face_FacemarkAAM_Config_getPropR_const(const cv::face::FacemarkAAM::Config* instance) {
			cv::Mat ret = instance->R;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Config_setPropR_Mat(cv::face::FacemarkAAM::Config* instance, cv::Mat* val) {
			instance->R = *val;
	}
	
	void cv_face_FacemarkAAM_Config_getPropT_const(const cv::face::FacemarkAAM::Config* instance, cv::Point2f* ocvrs_return) {
			cv::Point2f ret = instance->t;
			*ocvrs_return = ret;
	}
	
	void cv_face_FacemarkAAM_Config_setPropT_Point2f(cv::face::FacemarkAAM::Config* instance, cv::Point2f* val) {
			instance->t = *val;
	}
	
	float cv_face_FacemarkAAM_Config_getPropScale_const(const cv::face::FacemarkAAM::Config* instance) {
			float ret = instance->scale;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Config_setPropScale_float(cv::face::FacemarkAAM::Config* instance, float val) {
			instance->scale = val;
	}
	
	int cv_face_FacemarkAAM_Config_getPropModel_scale_idx_const(const cv::face::FacemarkAAM::Config* instance) {
			int ret = instance->model_scale_idx;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Config_setPropModel_scale_idx_int(cv::face::FacemarkAAM::Config* instance, int val) {
			instance->model_scale_idx = val;
	}
	
	void cv_FacemarkAAM_Config_delete(cv::face::FacemarkAAM::Config* instance) {
		delete instance;
	}
	void cv_face_FacemarkAAM_Config_Config_Mat_Point2f_float_int(cv::Mat* rot, cv::Point2f* trans, float scaling, int scale_id, Result<cv::face::FacemarkAAM::Config*>* ocvrs_return) {
		try {
			cv::face::FacemarkAAM::Config* ret = new cv::face::FacemarkAAM::Config(*rot, *trans, scaling, scale_id);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::FacemarkAAM::Config*>))
	}
	
	std::vector<cv::Point2f>* cv_face_FacemarkAAM_Data_getPropS0_const(const cv::face::FacemarkAAM::Data* instance) {
			std::vector<cv::Point2f> ret = instance->s0;
			return new std::vector<cv::Point2f>(ret);
	}
	
	void cv_face_FacemarkAAM_Data_setPropS0_vector_Point2f_(cv::face::FacemarkAAM::Data* instance, std::vector<cv::Point2f>* val) {
			instance->s0 = *val;
	}
	
	void cv_FacemarkAAM_Data_delete(cv::face::FacemarkAAM::Data* instance) {
		delete instance;
	}
	std::vector<float>* cv_face_FacemarkAAM_Model_getPropScales_const(const cv::face::FacemarkAAM::Model* instance) {
			std::vector<float> ret = instance->scales;
			return new std::vector<float>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropScales_vector_float_(cv::face::FacemarkAAM::Model* instance, std::vector<float>* val) {
			instance->scales = *val;
	}
	
	std::vector<cv::Vec3i>* cv_face_FacemarkAAM_Model_getPropTriangles_const(const cv::face::FacemarkAAM::Model* instance) {
			std::vector<cv::Vec3i> ret = instance->triangles;
			return new std::vector<cv::Vec3i>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropTriangles_vector_Vec3i_(cv::face::FacemarkAAM::Model* instance, std::vector<cv::Vec3i>* val) {
			instance->triangles = *val;
	}
	
	std::vector<cv::face::FacemarkAAM::Model::Texture>* cv_face_FacemarkAAM_Model_getPropTextures_const(const cv::face::FacemarkAAM::Model* instance) {
			std::vector<cv::face::FacemarkAAM::Model::Texture> ret = instance->textures;
			return new std::vector<cv::face::FacemarkAAM::Model::Texture>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropTextures_vector_Texture_(cv::face::FacemarkAAM::Model* instance, std::vector<cv::face::FacemarkAAM::Model::Texture>* val) {
			instance->textures = *val;
	}
	
	std::vector<cv::Point2f>* cv_face_FacemarkAAM_Model_getPropS0_const(const cv::face::FacemarkAAM::Model* instance) {
			std::vector<cv::Point2f> ret = instance->s0;
			return new std::vector<cv::Point2f>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropS0_vector_Point2f_(cv::face::FacemarkAAM::Model* instance, std::vector<cv::Point2f>* val) {
			instance->s0 = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_getPropS_const(const cv::face::FacemarkAAM::Model* instance) {
			cv::Mat ret = instance->S;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropS_Mat(cv::face::FacemarkAAM::Model* instance, cv::Mat* val) {
			instance->S = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_getPropQ_const(const cv::face::FacemarkAAM::Model* instance) {
			cv::Mat ret = instance->Q;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_setPropQ_Mat(cv::face::FacemarkAAM::Model* instance, cv::Mat* val) {
			instance->Q = *val;
	}
	
	void cv_FacemarkAAM_Model_delete(cv::face::FacemarkAAM::Model* instance) {
		delete instance;
	}
	int cv_face_FacemarkAAM_Model_Texture_getPropMax_m_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			int ret = instance->max_m;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropMax_m_int(cv::face::FacemarkAAM::Model::Texture* instance, int val) {
			instance->max_m = val;
	}
	
	void cv_face_FacemarkAAM_Model_Texture_getPropResolution_const(const cv::face::FacemarkAAM::Model::Texture* instance, cv::Rect* ocvrs_return) {
			cv::Rect ret = instance->resolution;
			*ocvrs_return = ret;
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropResolution_Rect(cv::face::FacemarkAAM::Model::Texture* instance, cv::Rect* val) {
			instance->resolution = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_Texture_getPropA_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			cv::Mat ret = instance->A;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropA_Mat(cv::face::FacemarkAAM::Model::Texture* instance, cv::Mat* val) {
			instance->A = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_Texture_getPropA0_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			cv::Mat ret = instance->A0;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropA0_Mat(cv::face::FacemarkAAM::Model::Texture* instance, cv::Mat* val) {
			instance->A0 = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_Texture_getPropAA_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			cv::Mat ret = instance->AA;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropAA_Mat(cv::face::FacemarkAAM::Model::Texture* instance, cv::Mat* val) {
			instance->AA = *val;
	}
	
	cv::Mat* cv_face_FacemarkAAM_Model_Texture_getPropAA0_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			cv::Mat ret = instance->AA0;
			return new cv::Mat(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropAA0_Mat(cv::face::FacemarkAAM::Model::Texture* instance, cv::Mat* val) {
			instance->AA0 = *val;
	}
	
	std::vector<std::vector<cv::Point>>* cv_face_FacemarkAAM_Model_Texture_getPropTextureIdx_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			std::vector<std::vector<cv::Point>> ret = instance->textureIdx;
			return new std::vector<std::vector<cv::Point>>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropTextureIdx_vector_vector_Point__(cv::face::FacemarkAAM::Model::Texture* instance, std::vector<std::vector<cv::Point>>* val) {
			instance->textureIdx = *val;
	}
	
	std::vector<cv::Point2f>* cv_face_FacemarkAAM_Model_Texture_getPropBase_shape_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			std::vector<cv::Point2f> ret = instance->base_shape;
			return new std::vector<cv::Point2f>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropBase_shape_vector_Point2f_(cv::face::FacemarkAAM::Model::Texture* instance, std::vector<cv::Point2f>* val) {
			instance->base_shape = *val;
	}
	
	std::vector<int>* cv_face_FacemarkAAM_Model_Texture_getPropInd1_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			std::vector<int> ret = instance->ind1;
			return new std::vector<int>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropInd1_vector_int_(cv::face::FacemarkAAM::Model::Texture* instance, std::vector<int>* val) {
			instance->ind1 = *val;
	}
	
	std::vector<int>* cv_face_FacemarkAAM_Model_Texture_getPropInd2_const(const cv::face::FacemarkAAM::Model::Texture* instance) {
			std::vector<int> ret = instance->ind2;
			return new std::vector<int>(ret);
	}
	
	void cv_face_FacemarkAAM_Model_Texture_setPropInd2_vector_int_(cv::face::FacemarkAAM::Model::Texture* instance, std::vector<int>* val) {
			instance->ind2 = *val;
	}
	
	void cv_FacemarkAAM_Model_Texture_delete(cv::face::FacemarkAAM::Model::Texture* instance) {
		delete instance;
	}
	void* cv_face_FacemarkAAM_Params_getPropModel_filename_const(const cv::face::FacemarkAAM::Params* instance) {
			std::string ret = instance->model_filename;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_face_FacemarkAAM_Params_setPropModel_filename_string(cv::face::FacemarkAAM::Params* instance, char* val) {
			instance->model_filename = std::string(val);
	}
	
	int cv_face_FacemarkAAM_Params_getPropM_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->m;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropM_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->m = val;
	}
	
	int cv_face_FacemarkAAM_Params_getPropN_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->n;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropN_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->n = val;
	}
	
	int cv_face_FacemarkAAM_Params_getPropN_iter_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->n_iter;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropN_iter_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->n_iter = val;
	}
	
	bool cv_face_FacemarkAAM_Params_getPropVerbose_const(const cv::face::FacemarkAAM::Params* instance) {
			bool ret = instance->verbose;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropVerbose_bool(cv::face::FacemarkAAM::Params* instance, bool val) {
			instance->verbose = val;
	}
	
	bool cv_face_FacemarkAAM_Params_getPropSave_model_const(const cv::face::FacemarkAAM::Params* instance) {
			bool ret = instance->save_model;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropSave_model_bool(cv::face::FacemarkAAM::Params* instance, bool val) {
			instance->save_model = val;
	}
	
	int cv_face_FacemarkAAM_Params_getPropMax_m_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->max_m;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropMax_m_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->max_m = val;
	}
	
	int cv_face_FacemarkAAM_Params_getPropMax_n_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->max_n;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropMax_n_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->max_n = val;
	}
	
	int cv_face_FacemarkAAM_Params_getPropTexture_max_m_const(const cv::face::FacemarkAAM::Params* instance) {
			int ret = instance->texture_max_m;
			return ret;
	}
	
	void cv_face_FacemarkAAM_Params_setPropTexture_max_m_int(cv::face::FacemarkAAM::Params* instance, int val) {
			instance->texture_max_m = val;
	}
	
	std::vector<float>* cv_face_FacemarkAAM_Params_getPropScales_const(const cv::face::FacemarkAAM::Params* instance) {
			std::vector<float> ret = instance->scales;
			return new std::vector<float>(ret);
	}
	
	void cv_face_FacemarkAAM_Params_setPropScales_vector_float_(cv::face::FacemarkAAM::Params* instance, std::vector<float>* val) {
			instance->scales = *val;
	}
	
	void cv_FacemarkAAM_Params_delete(cv::face::FacemarkAAM::Params* instance) {
		delete instance;
	}
	void cv_face_FacemarkAAM_Params_Params(Result<cv::face::FacemarkAAM::Params*>* ocvrs_return) {
		try {
			cv::face::FacemarkAAM::Params* ret = new cv::face::FacemarkAAM::Params();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::FacemarkAAM::Params*>))
	}
	
	void cv_face_FacemarkAAM_Params_read_const_FileNodeR(cv::face::FacemarkAAM::Params* instance, const cv::FileNode* unnamed, Result_void* ocvrs_return) {
		try {
			instance->read(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FacemarkAAM_Params_write_const_FileStorageR(const cv::face::FacemarkAAM::Params* instance, cv::FileStorage* unnamed, Result_void* ocvrs_return) {
		try {
			instance->write(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FacemarkKazemi_create_const_ParamsR(const cv::face::FacemarkKazemi::Params* parameters, Result<cv::Ptr<cv::face::FacemarkKazemi>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::FacemarkKazemi> ret = cv::face::FacemarkKazemi::create(*parameters);
			Ok(new cv::Ptr<cv::face::FacemarkKazemi>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::FacemarkKazemi>*>))
	}
	
	void cv_face_FacemarkKazemi_training_vector_Mat_R_vector_vector_Point2f__R_string_Size_string(cv::face::FacemarkKazemi* instance, std::vector<cv::Mat>* images, std::vector<std::vector<cv::Point2f>>* landmarks, char* configfile, cv::Size* scale, char* modelFilename, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->training(*images, *landmarks, std::string(configfile), *scale, std::string(modelFilename));
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkKazemi_setFaceDetector_bool__X__const_cv__InputArrayR__const_cv__OutputArrayR__voidX__voidX(cv::face::FacemarkKazemi* instance, bool (*f)(const cv::_InputArray&, const cv::_OutputArray&, void*), void* userData, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->setFaceDetector(f, userData);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkKazemi_getFaces_const__InputArrayR_const__OutputArrayR(cv::face::FacemarkKazemi* instance, const cv::_InputArray* image, const cv::_OutputArray* faces, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getFaces(*image, *faces);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropCascade_depth_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->cascade_depth;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropCascade_depth_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->cascade_depth = val;
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropTree_depth_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->tree_depth;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropTree_depth_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->tree_depth = val;
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropNum_trees_per_cascade_level_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->num_trees_per_cascade_level;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropNum_trees_per_cascade_level_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->num_trees_per_cascade_level = val;
	}
	
	float cv_face_FacemarkKazemi_Params_getPropLearning_rate_const(const cv::face::FacemarkKazemi::Params* instance) {
			float ret = instance->learning_rate;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropLearning_rate_float(cv::face::FacemarkKazemi::Params* instance, float val) {
			instance->learning_rate = val;
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropOversampling_amount_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->oversampling_amount;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropOversampling_amount_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->oversampling_amount = val;
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropNum_test_coordinates_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->num_test_coordinates;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropNum_test_coordinates_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->num_test_coordinates = val;
	}
	
	float cv_face_FacemarkKazemi_Params_getPropLambda_const(const cv::face::FacemarkKazemi::Params* instance) {
			float ret = instance->lambda;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropLambda_float(cv::face::FacemarkKazemi::Params* instance, float val) {
			instance->lambda = val;
	}
	
	unsigned long cv_face_FacemarkKazemi_Params_getPropNum_test_splits_const(const cv::face::FacemarkKazemi::Params* instance) {
			unsigned long ret = instance->num_test_splits;
			return ret;
	}
	
	void cv_face_FacemarkKazemi_Params_setPropNum_test_splits_unsigned_long(cv::face::FacemarkKazemi::Params* instance, unsigned long val) {
			instance->num_test_splits = val;
	}
	
	void* cv_face_FacemarkKazemi_Params_getPropConfigfile_const(const cv::face::FacemarkKazemi::Params* instance) {
			cv::String ret = instance->configfile;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_face_FacemarkKazemi_Params_setPropConfigfile_String(cv::face::FacemarkKazemi::Params* instance, char* val) {
			instance->configfile = std::string(val);
	}
	
	void cv_FacemarkKazemi_Params_delete(cv::face::FacemarkKazemi::Params* instance) {
		delete instance;
	}
	void cv_face_FacemarkKazemi_Params_Params(Result<cv::face::FacemarkKazemi::Params*>* ocvrs_return) {
		try {
			cv::face::FacemarkKazemi::Params* ret = new cv::face::FacemarkKazemi::Params();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::FacemarkKazemi::Params*>))
	}
	
	void cv_face_FacemarkLBF_create_const_ParamsR(const cv::face::FacemarkLBF::Params* parameters, Result<cv::Ptr<cv::face::FacemarkLBF>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::FacemarkLBF> ret = cv::face::FacemarkLBF::create(*parameters);
			Ok(new cv::Ptr<cv::face::FacemarkLBF>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::FacemarkLBF>*>))
	}
	
	double cv_face_FacemarkLBF_Params_getPropShape_offset_const(const cv::face::FacemarkLBF::Params* instance) {
			double ret = instance->shape_offset;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropShape_offset_double(cv::face::FacemarkLBF::Params* instance, double val) {
			instance->shape_offset = val;
	}
	
	void* cv_face_FacemarkLBF_Params_getPropCascade_face_const(const cv::face::FacemarkLBF::Params* instance) {
			cv::String ret = instance->cascade_face;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_face_FacemarkLBF_Params_setPropCascade_face_String(cv::face::FacemarkLBF::Params* instance, char* val) {
			instance->cascade_face = std::string(val);
	}
	
	bool cv_face_FacemarkLBF_Params_getPropVerbose_const(const cv::face::FacemarkLBF::Params* instance) {
			bool ret = instance->verbose;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropVerbose_bool(cv::face::FacemarkLBF::Params* instance, bool val) {
			instance->verbose = val;
	}
	
	int cv_face_FacemarkLBF_Params_getPropN_landmarks_const(const cv::face::FacemarkLBF::Params* instance) {
			int ret = instance->n_landmarks;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropN_landmarks_int(cv::face::FacemarkLBF::Params* instance, int val) {
			instance->n_landmarks = val;
	}
	
	int cv_face_FacemarkLBF_Params_getPropInitShape_n_const(const cv::face::FacemarkLBF::Params* instance) {
			int ret = instance->initShape_n;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropInitShape_n_int(cv::face::FacemarkLBF::Params* instance, int val) {
			instance->initShape_n = val;
	}
	
	int cv_face_FacemarkLBF_Params_getPropStages_n_const(const cv::face::FacemarkLBF::Params* instance) {
			int ret = instance->stages_n;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropStages_n_int(cv::face::FacemarkLBF::Params* instance, int val) {
			instance->stages_n = val;
	}
	
	int cv_face_FacemarkLBF_Params_getPropTree_n_const(const cv::face::FacemarkLBF::Params* instance) {
			int ret = instance->tree_n;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropTree_n_int(cv::face::FacemarkLBF::Params* instance, int val) {
			instance->tree_n = val;
	}
	
	int cv_face_FacemarkLBF_Params_getPropTree_depth_const(const cv::face::FacemarkLBF::Params* instance) {
			int ret = instance->tree_depth;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropTree_depth_int(cv::face::FacemarkLBF::Params* instance, int val) {
			instance->tree_depth = val;
	}
	
	double cv_face_FacemarkLBF_Params_getPropBagging_overlap_const(const cv::face::FacemarkLBF::Params* instance) {
			double ret = instance->bagging_overlap;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropBagging_overlap_double(cv::face::FacemarkLBF::Params* instance, double val) {
			instance->bagging_overlap = val;
	}
	
	void* cv_face_FacemarkLBF_Params_getPropModel_filename_const(const cv::face::FacemarkLBF::Params* instance) {
			std::string ret = instance->model_filename;
			return ocvrs_create_string(ret.c_str());
	}
	
	void cv_face_FacemarkLBF_Params_setPropModel_filename_string(cv::face::FacemarkLBF::Params* instance, char* val) {
			instance->model_filename = std::string(val);
	}
	
	bool cv_face_FacemarkLBF_Params_getPropSave_model_const(const cv::face::FacemarkLBF::Params* instance) {
			bool ret = instance->save_model;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropSave_model_bool(cv::face::FacemarkLBF::Params* instance, bool val) {
			instance->save_model = val;
	}
	
	unsigned int cv_face_FacemarkLBF_Params_getPropSeed_const(const cv::face::FacemarkLBF::Params* instance) {
			unsigned int ret = instance->seed;
			return ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropSeed_unsigned_int(cv::face::FacemarkLBF::Params* instance, unsigned int val) {
			instance->seed = val;
	}
	
	std::vector<int>* cv_face_FacemarkLBF_Params_getPropFeats_m_const(const cv::face::FacemarkLBF::Params* instance) {
			std::vector<int> ret = instance->feats_m;
			return new std::vector<int>(ret);
	}
	
	void cv_face_FacemarkLBF_Params_setPropFeats_m_vector_int_(cv::face::FacemarkLBF::Params* instance, std::vector<int>* val) {
			instance->feats_m = *val;
	}
	
	std::vector<double>* cv_face_FacemarkLBF_Params_getPropRadius_m_const(const cv::face::FacemarkLBF::Params* instance) {
			std::vector<double> ret = instance->radius_m;
			return new std::vector<double>(ret);
	}
	
	void cv_face_FacemarkLBF_Params_setPropRadius_m_vector_double_(cv::face::FacemarkLBF::Params* instance, std::vector<double>* val) {
			instance->radius_m = *val;
	}
	
	void cv_face_FacemarkLBF_Params_getPropDetectROI_const(const cv::face::FacemarkLBF::Params* instance, cv::Rect* ocvrs_return) {
			cv::Rect ret = instance->detectROI;
			*ocvrs_return = ret;
	}
	
	void cv_face_FacemarkLBF_Params_setPropDetectROI_Rect(cv::face::FacemarkLBF::Params* instance, cv::Rect* val) {
			instance->detectROI = *val;
	}
	
	void cv_FacemarkLBF_Params_delete(cv::face::FacemarkLBF::Params* instance) {
		delete instance;
	}
	void cv_face_FacemarkLBF_Params_Params(Result<cv::face::FacemarkLBF::Params*>* ocvrs_return) {
		try {
			cv::face::FacemarkLBF::Params* ret = new cv::face::FacemarkLBF::Params();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::FacemarkLBF::Params*>))
	}
	
	void cv_face_FacemarkLBF_Params_read_const_FileNodeR(cv::face::FacemarkLBF::Params* instance, const cv::FileNode* unnamed, Result_void* ocvrs_return) {
		try {
			instance->read(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FacemarkLBF_Params_write_const_FileStorageR(const cv::face::FacemarkLBF::Params* instance, cv::FileStorage* unnamed, Result_void* ocvrs_return) {
		try {
			instance->write(*unnamed);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FacemarkTrain_addTrainingSample_const__InputArrayR_const__InputArrayR(cv::face::FacemarkTrain* instance, const cv::_InputArray* image, const cv::_InputArray* landmarks, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->addTrainingSample(*image, *landmarks);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkTrain_training_voidX(cv::face::FacemarkTrain* instance, void* parameters, Result_void* ocvrs_return) {
		try {
			instance->training(parameters);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_FacemarkTrain_setFaceDetector_FN_FaceDetector_voidX(cv::face::FacemarkTrain* instance, cv::face::FN_FaceDetector detector, void* userData, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->setFaceDetector(detector, userData);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkTrain_getFaces_const__InputArrayR_const__OutputArrayR(cv::face::FacemarkTrain* instance, const cv::_InputArray* image, const cv::_OutputArray* faces, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getFaces(*image, *faces);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FacemarkTrain_getData_voidX(cv::face::FacemarkTrain* instance, void* items, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getData(items);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_FisherFaceRecognizer_create_int_double(int num_components, double threshold, Result<cv::Ptr<cv::face::FisherFaceRecognizer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::FisherFaceRecognizer> ret = cv::face::FisherFaceRecognizer::create(num_components, threshold);
			Ok(new cv::Ptr<cv::face::FisherFaceRecognizer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::FisherFaceRecognizer>*>))
	}
	
	void cv_face_LBPHFaceRecognizer_getGridX_const(const cv::face::LBPHFaceRecognizer* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getGridX();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_LBPHFaceRecognizer_setGridX_int(cv::face::LBPHFaceRecognizer* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setGridX(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_LBPHFaceRecognizer_getGridY_const(const cv::face::LBPHFaceRecognizer* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getGridY();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_LBPHFaceRecognizer_setGridY_int(cv::face::LBPHFaceRecognizer* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setGridY(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_LBPHFaceRecognizer_getRadius_const(const cv::face::LBPHFaceRecognizer* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getRadius();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_LBPHFaceRecognizer_setRadius_int(cv::face::LBPHFaceRecognizer* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setRadius(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_LBPHFaceRecognizer_getNeighbors_const(const cv::face::LBPHFaceRecognizer* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNeighbors();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_LBPHFaceRecognizer_setNeighbors_int(cv::face::LBPHFaceRecognizer* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setNeighbors(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_LBPHFaceRecognizer_getThreshold_const(const cv::face::LBPHFaceRecognizer* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getThreshold();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_face_LBPHFaceRecognizer_setThreshold_double(cv::face::LBPHFaceRecognizer* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setThreshold(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_LBPHFaceRecognizer_getHistograms_const(const cv::face::LBPHFaceRecognizer* instance, Result<std::vector<cv::Mat>*>* ocvrs_return) {
		try {
			std::vector<cv::Mat> ret = instance->getHistograms();
			Ok(new std::vector<cv::Mat>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::Mat>*>))
	}
	
	void cv_face_LBPHFaceRecognizer_getLabels_const(const cv::face::LBPHFaceRecognizer* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getLabels();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_face_LBPHFaceRecognizer_create_int_int_int_int_double(int radius, int neighbors, int grid_x, int grid_y, double threshold, Result<cv::Ptr<cv::face::LBPHFaceRecognizer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::LBPHFaceRecognizer> ret = cv::face::LBPHFaceRecognizer::create(radius, neighbors, grid_x, grid_y, threshold);
			Ok(new cv::Ptr<cv::face::LBPHFaceRecognizer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::LBPHFaceRecognizer>*>))
	}
	
	void cv_face_MACE_salt_const_StringR(cv::face::MACE* instance, const char* passphrase, Result_void* ocvrs_return) {
		try {
			instance->salt(std::string(passphrase));
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_MACE_train_const__InputArrayR(cv::face::MACE* instance, const cv::_InputArray* images, Result_void* ocvrs_return) {
		try {
			instance->train(*images);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_MACE_same_const_const__InputArrayR(const cv::face::MACE* instance, const cv::_InputArray* query, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->same(*query);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_MACE_load_const_StringR_const_StringR(const char* filename, const char* objname, Result<cv::Ptr<cv::face::MACE>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::MACE> ret = cv::face::MACE::load(std::string(filename), std::string(objname));
			Ok(new cv::Ptr<cv::face::MACE>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::MACE>*>))
	}
	
	void cv_face_MACE_create_int(int IMGSIZE, Result<cv::Ptr<cv::face::MACE>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::MACE> ret = cv::face::MACE::create(IMGSIZE);
			Ok(new cv::Ptr<cv::face::MACE>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::MACE>*>))
	}
	
	void cv_face_PredictCollector_init_size_t(cv::face::PredictCollector* instance, size_t size, Result_void* ocvrs_return) {
		try {
			instance->init(size);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_PredictCollector_collect_int_double(cv::face::PredictCollector* instance, int label, double dist, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->collect(label, dist);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_StandardCollector_delete(cv::face::StandardCollector* instance) {
		delete instance;
	}
	void cv_face_StandardCollector_StandardCollector_double(double threshold_, Result<cv::face::StandardCollector*>* ocvrs_return) {
		try {
			cv::face::StandardCollector* ret = new cv::face::StandardCollector(threshold_);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::StandardCollector*>))
	}
	
	void cv_face_StandardCollector_init_size_t(cv::face::StandardCollector* instance, size_t size, Result_void* ocvrs_return) {
		try {
			instance->init(size);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_face_StandardCollector_collect_int_double(cv::face::StandardCollector* instance, int label, double dist, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->collect(label, dist);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_face_StandardCollector_getMinLabel_const(const cv::face::StandardCollector* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMinLabel();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_face_StandardCollector_getMinDist_const(const cv::face::StandardCollector* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getMinDist();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_face_StandardCollector_create_double(double threshold, Result<cv::Ptr<cv::face::StandardCollector>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::face::StandardCollector> ret = cv::face::StandardCollector::create(threshold);
			Ok(new cv::Ptr<cv::face::StandardCollector>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::face::StandardCollector>*>))
	}
	
	void cv_face_StandardCollector_PredictResult_PredictResult_int_double(int label_, double distance_, Result<cv::face::StandardCollector::PredictResult>* ocvrs_return) {
		try {
			cv::face::StandardCollector::PredictResult ret(label_, distance_);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::face::StandardCollector::PredictResult>))
	}
	
}
