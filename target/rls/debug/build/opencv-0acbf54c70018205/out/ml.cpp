#include "ocvrs_common.hpp"
#include <opencv2/ml.hpp>
#include "ml_types.hpp"

extern "C" {
	void cv_ml_createConcentricSpheresTestSet_int_int_int_const__OutputArrayR_const__OutputArrayR(int nsamples, int nfeatures, int nclasses, const cv::_OutputArray* samples, const cv::_OutputArray* responses, Result_void* ocvrs_return) {
		try {
			cv::ml::createConcentricSpheresTestSet(nsamples, nfeatures, nclasses, *samples, *responses);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_randMVNormal_const__InputArrayR_const__InputArrayR_int_const__OutputArrayR(const cv::_InputArray* mean, const cv::_InputArray* cov, int nsamples, const cv::_OutputArray* samples, Result_void* ocvrs_return) {
		try {
			cv::ml::randMVNormal(*mean, *cov, nsamples, *samples);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_setTrainMethod_int_double_double(cv::ml::ANN_MLP* instance, int method, double param1, double param2, Result_void* ocvrs_return) {
		try {
			instance->setTrainMethod(method, param1, param2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getTrainMethod_const(const cv::ml::ANN_MLP* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getTrainMethod();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_ANN_MLP_setActivationFunction_int_double_double(cv::ml::ANN_MLP* instance, int type, double param1, double param2, Result_void* ocvrs_return) {
		try {
			instance->setActivationFunction(type, param1, param2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_setLayerSizes_const__InputArrayR(cv::ml::ANN_MLP* instance, const cv::_InputArray* _layer_sizes, Result_void* ocvrs_return) {
		try {
			instance->setLayerSizes(*_layer_sizes);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getLayerSizes_const(const cv::ml::ANN_MLP* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getLayerSizes();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_ANN_MLP_getTermCriteria_const(const cv::ml::ANN_MLP* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_ANN_MLP_setTermCriteria_TermCriteria(cv::ml::ANN_MLP* instance, cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getBackpropWeightScale_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getBackpropWeightScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setBackpropWeightScale_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setBackpropWeightScale(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getBackpropMomentumScale_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getBackpropMomentumScale();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setBackpropMomentumScale_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setBackpropMomentumScale(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getRpropDW0_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRpropDW0();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setRpropDW0_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setRpropDW0(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getRpropDWPlus_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRpropDWPlus();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setRpropDWPlus_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setRpropDWPlus(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getRpropDWMinus_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRpropDWMinus();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setRpropDWMinus_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setRpropDWMinus(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getRpropDWMin_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRpropDWMin();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setRpropDWMin_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setRpropDWMin(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getRpropDWMax_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRpropDWMax();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setRpropDWMax_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setRpropDWMax(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getAnnealInitialT_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getAnnealInitialT();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setAnnealInitialT_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setAnnealInitialT(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getAnnealFinalT_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getAnnealFinalT();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setAnnealFinalT_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setAnnealFinalT(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getAnnealCoolingRatio_const(const cv::ml::ANN_MLP* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getAnnealCoolingRatio();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_ANN_MLP_setAnnealCoolingRatio_double(cv::ml::ANN_MLP* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setAnnealCoolingRatio(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getAnnealItePerStep_const(const cv::ml::ANN_MLP* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getAnnealItePerStep();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_ANN_MLP_setAnnealItePerStep_int(cv::ml::ANN_MLP* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setAnnealItePerStep(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_setAnnealEnergyRNG_const_RNGR(cv::ml::ANN_MLP* instance, const cv::RNG* rng, Result_void* ocvrs_return) {
		try {
			instance->setAnnealEnergyRNG(*rng);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_ANN_MLP_getWeights_const_int(const cv::ml::ANN_MLP* instance, int layerIdx, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getWeights(layerIdx);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_ANN_MLP_create(Result<cv::Ptr<cv::ml::ANN_MLP>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::ANN_MLP> ret = cv::ml::ANN_MLP::create();
			Ok(new cv::Ptr<cv::ml::ANN_MLP>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::ANN_MLP>*>))
	}
	
	void cv_ml_ANN_MLP_load_const_StringR(const char* filepath, Result<cv::Ptr<cv::ml::ANN_MLP>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::ANN_MLP> ret = cv::ml::ANN_MLP::load(std::string(filepath));
			Ok(new cv::Ptr<cv::ml::ANN_MLP>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::ANN_MLP>*>))
	}
	
	void cv_ml_Boost_getBoostType_const(const cv::ml::Boost* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getBoostType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_Boost_setBoostType_int(cv::ml::Boost* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setBoostType(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_Boost_getWeakCount_const(const cv::ml::Boost* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getWeakCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_Boost_setWeakCount_int(cv::ml::Boost* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setWeakCount(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_Boost_getWeightTrimRate_const(const cv::ml::Boost* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getWeightTrimRate();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_Boost_setWeightTrimRate_double(cv::ml::Boost* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setWeightTrimRate(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_Boost_create(Result<cv::Ptr<cv::ml::Boost>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::Boost> ret = cv::ml::Boost::create();
			Ok(new cv::Ptr<cv::ml::Boost>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::Boost>*>))
	}
	
	void cv_ml_Boost_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::Boost>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::Boost> ret = cv::ml::Boost::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::Boost>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::Boost>*>))
	}
	
	void cv_ml_DTrees_getMaxCategories_const(const cv::ml::DTrees* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxCategories();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_DTrees_setMaxCategories_int(cv::ml::DTrees* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setMaxCategories(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getMaxDepth_const(const cv::ml::DTrees* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMaxDepth();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_DTrees_setMaxDepth_int(cv::ml::DTrees* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setMaxDepth(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getMinSampleCount_const(const cv::ml::DTrees* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMinSampleCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_DTrees_setMinSampleCount_int(cv::ml::DTrees* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setMinSampleCount(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getCVFolds_const(const cv::ml::DTrees* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getCVFolds();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_DTrees_setCVFolds_int(cv::ml::DTrees* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setCVFolds(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getUseSurrogates_const(const cv::ml::DTrees* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUseSurrogates();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_DTrees_setUseSurrogates_bool(cv::ml::DTrees* instance, bool val, Result_void* ocvrs_return) {
		try {
			instance->setUseSurrogates(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getUse1SERule_const(const cv::ml::DTrees* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getUse1SERule();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_DTrees_setUse1SERule_bool(cv::ml::DTrees* instance, bool val, Result_void* ocvrs_return) {
		try {
			instance->setUse1SERule(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getTruncatePrunedTree_const(const cv::ml::DTrees* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getTruncatePrunedTree();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_DTrees_setTruncatePrunedTree_bool(cv::ml::DTrees* instance, bool val, Result_void* ocvrs_return) {
		try {
			instance->setTruncatePrunedTree(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getRegressionAccuracy_const(const cv::ml::DTrees* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getRegressionAccuracy();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_DTrees_setRegressionAccuracy_float(cv::ml::DTrees* instance, float val, Result_void* ocvrs_return) {
		try {
			instance->setRegressionAccuracy(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getPriors_const(const cv::ml::DTrees* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getPriors();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_DTrees_setPriors_const_MatR(cv::ml::DTrees* instance, const cv::Mat* val, Result_void* ocvrs_return) {
		try {
			instance->setPriors(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_DTrees_getRoots_const(const cv::ml::DTrees* instance, Result<std::vector<int>*>* ocvrs_return) {
		try {
			const std::vector<int> ret = instance->getRoots();
			Ok(new const std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_ml_DTrees_getNodes_const(const cv::ml::DTrees* instance, Result<std::vector<cv::ml::DTrees::Node>*>* ocvrs_return) {
		try {
			const std::vector<cv::ml::DTrees::Node> ret = instance->getNodes();
			Ok(new const std::vector<cv::ml::DTrees::Node>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::ml::DTrees::Node>*>))
	}
	
	void cv_ml_DTrees_getSplits_const(const cv::ml::DTrees* instance, Result<std::vector<cv::ml::DTrees::Split>*>* ocvrs_return) {
		try {
			const std::vector<cv::ml::DTrees::Split> ret = instance->getSplits();
			Ok(new const std::vector<cv::ml::DTrees::Split>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<cv::ml::DTrees::Split>*>))
	}
	
	void cv_ml_DTrees_getSubsets_const(const cv::ml::DTrees* instance, Result<std::vector<int>*>* ocvrs_return) {
		try {
			const std::vector<int> ret = instance->getSubsets();
			Ok(new const std::vector<int>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<std::vector<int>*>))
	}
	
	void cv_ml_DTrees_create(Result<cv::Ptr<cv::ml::DTrees>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::DTrees> ret = cv::ml::DTrees::create();
			Ok(new cv::Ptr<cv::ml::DTrees>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::DTrees>*>))
	}
	
	void cv_ml_DTrees_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::DTrees>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::DTrees> ret = cv::ml::DTrees::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::DTrees>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::DTrees>*>))
	}
	
	double cv_ml_DTrees_Node_getPropValue_const(const cv::ml::DTrees::Node* instance) {
			double ret = instance->value;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropValue_double(cv::ml::DTrees::Node* instance, double val) {
			instance->value = val;
	}
	
	int cv_ml_DTrees_Node_getPropClassIdx_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->classIdx;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropClassIdx_int(cv::ml::DTrees::Node* instance, int val) {
			instance->classIdx = val;
	}
	
	int cv_ml_DTrees_Node_getPropParent_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->parent;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropParent_int(cv::ml::DTrees::Node* instance, int val) {
			instance->parent = val;
	}
	
	int cv_ml_DTrees_Node_getPropLeft_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->left;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropLeft_int(cv::ml::DTrees::Node* instance, int val) {
			instance->left = val;
	}
	
	int cv_ml_DTrees_Node_getPropRight_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->right;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropRight_int(cv::ml::DTrees::Node* instance, int val) {
			instance->right = val;
	}
	
	int cv_ml_DTrees_Node_getPropDefaultDir_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->defaultDir;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropDefaultDir_int(cv::ml::DTrees::Node* instance, int val) {
			instance->defaultDir = val;
	}
	
	int cv_ml_DTrees_Node_getPropSplit_const(const cv::ml::DTrees::Node* instance) {
			int ret = instance->split;
			return ret;
	}
	
	void cv_ml_DTrees_Node_setPropSplit_int(cv::ml::DTrees::Node* instance, int val) {
			instance->split = val;
	}
	
	void cv_DTrees_Node_delete(cv::ml::DTrees::Node* instance) {
		delete instance;
	}
	void cv_ml_DTrees_Node_Node(Result<cv::ml::DTrees::Node*>* ocvrs_return) {
		try {
			cv::ml::DTrees::Node* ret = new cv::ml::DTrees::Node();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::ml::DTrees::Node*>))
	}
	
	int cv_ml_DTrees_Split_getPropVarIdx_const(const cv::ml::DTrees::Split* instance) {
			int ret = instance->varIdx;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropVarIdx_int(cv::ml::DTrees::Split* instance, int val) {
			instance->varIdx = val;
	}
	
	bool cv_ml_DTrees_Split_getPropInversed_const(const cv::ml::DTrees::Split* instance) {
			bool ret = instance->inversed;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropInversed_bool(cv::ml::DTrees::Split* instance, bool val) {
			instance->inversed = val;
	}
	
	float cv_ml_DTrees_Split_getPropQuality_const(const cv::ml::DTrees::Split* instance) {
			float ret = instance->quality;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropQuality_float(cv::ml::DTrees::Split* instance, float val) {
			instance->quality = val;
	}
	
	int cv_ml_DTrees_Split_getPropNext_const(const cv::ml::DTrees::Split* instance) {
			int ret = instance->next;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropNext_int(cv::ml::DTrees::Split* instance, int val) {
			instance->next = val;
	}
	
	float cv_ml_DTrees_Split_getPropC_const(const cv::ml::DTrees::Split* instance) {
			float ret = instance->c;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropC_float(cv::ml::DTrees::Split* instance, float val) {
			instance->c = val;
	}
	
	int cv_ml_DTrees_Split_getPropSubsetOfs_const(const cv::ml::DTrees::Split* instance) {
			int ret = instance->subsetOfs;
			return ret;
	}
	
	void cv_ml_DTrees_Split_setPropSubsetOfs_int(cv::ml::DTrees::Split* instance, int val) {
			instance->subsetOfs = val;
	}
	
	void cv_DTrees_Split_delete(cv::ml::DTrees::Split* instance) {
		delete instance;
	}
	void cv_ml_DTrees_Split_Split(Result<cv::ml::DTrees::Split*>* ocvrs_return) {
		try {
			cv::ml::DTrees::Split* ret = new cv::ml::DTrees::Split();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::ml::DTrees::Split*>))
	}
	
	void cv_ml_EM_getClustersNumber_const(const cv::ml::EM* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getClustersNumber();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_EM_setClustersNumber_int(cv::ml::EM* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setClustersNumber(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_EM_getCovarianceMatrixType_const(const cv::ml::EM* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getCovarianceMatrixType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_EM_setCovarianceMatrixType_int(cv::ml::EM* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setCovarianceMatrixType(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_EM_getTermCriteria_const(const cv::ml::EM* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_EM_setTermCriteria_const_TermCriteriaR(cv::ml::EM* instance, const cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_EM_getWeights_const(const cv::ml::EM* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_EM_getMeans_const(const cv::ml::EM* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getMeans();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_EM_getCovs_const_vector_Mat_R(const cv::ml::EM* instance, std::vector<cv::Mat>* covs, Result_void* ocvrs_return) {
		try {
			instance->getCovs(*covs);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_EM_predict_const_const__InputArrayR_const__OutputArrayR_int(const cv::ml::EM* instance, const cv::_InputArray* samples, const cv::_OutputArray* results, int flags, Result<float>* ocvrs_return) {
		try {
			float ret = instance->predict(*samples, *results, flags);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_EM_predict2_const_const__InputArrayR_const__OutputArrayR(const cv::ml::EM* instance, const cv::_InputArray* sample, const cv::_OutputArray* probs, Result<cv::Vec2d>* ocvrs_return) {
		try {
			cv::Vec2d ret = instance->predict2(*sample, *probs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Vec2d>))
	}
	
	void cv_ml_EM_trainEM_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_const__OutputArrayR(cv::ml::EM* instance, const cv::_InputArray* samples, const cv::_OutputArray* logLikelihoods, const cv::_OutputArray* labels, const cv::_OutputArray* probs, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->trainEM(*samples, *logLikelihoods, *labels, *probs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_EM_trainE_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_const__OutputArrayR(cv::ml::EM* instance, const cv::_InputArray* samples, const cv::_InputArray* means0, const cv::_InputArray* covs0, const cv::_InputArray* weights0, const cv::_OutputArray* logLikelihoods, const cv::_OutputArray* labels, const cv::_OutputArray* probs, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->trainE(*samples, *means0, *covs0, *weights0, *logLikelihoods, *labels, *probs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_EM_trainM_const__InputArrayR_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_const__OutputArrayR(cv::ml::EM* instance, const cv::_InputArray* samples, const cv::_InputArray* probs0, const cv::_OutputArray* logLikelihoods, const cv::_OutputArray* labels, const cv::_OutputArray* probs, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->trainM(*samples, *probs0, *logLikelihoods, *labels, *probs);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_EM_create(Result<cv::Ptr<cv::ml::EM>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::EM> ret = cv::ml::EM::create();
			Ok(new cv::Ptr<cv::ml::EM>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::EM>*>))
	}
	
	void cv_ml_EM_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::EM>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::EM> ret = cv::ml::EM::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::EM>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::EM>*>))
	}
	
	void cv_ml_KNearest_getDefaultK_const(const cv::ml::KNearest* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getDefaultK();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_KNearest_setDefaultK_int(cv::ml::KNearest* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setDefaultK(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_KNearest_getIsClassifier_const(const cv::ml::KNearest* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getIsClassifier();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_KNearest_setIsClassifier_bool(cv::ml::KNearest* instance, bool val, Result_void* ocvrs_return) {
		try {
			instance->setIsClassifier(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_KNearest_getEmax_const(const cv::ml::KNearest* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getEmax();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_KNearest_setEmax_int(cv::ml::KNearest* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setEmax(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_KNearest_getAlgorithmType_const(const cv::ml::KNearest* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getAlgorithmType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_KNearest_setAlgorithmType_int(cv::ml::KNearest* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setAlgorithmType(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_KNearest_findNearest_const_const__InputArrayR_int_const__OutputArrayR_const__OutputArrayR_const__OutputArrayR(const cv::ml::KNearest* instance, const cv::_InputArray* samples, int k, const cv::_OutputArray* results, const cv::_OutputArray* neighborResponses, const cv::_OutputArray* dist, Result<float>* ocvrs_return) {
		try {
			float ret = instance->findNearest(*samples, k, *results, *neighborResponses, *dist);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_KNearest_create(Result<cv::Ptr<cv::ml::KNearest>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::KNearest> ret = cv::ml::KNearest::create();
			Ok(new cv::Ptr<cv::ml::KNearest>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::KNearest>*>))
	}
	
	void cv_ml_KNearest_load_const_StringR(const char* filepath, Result<cv::Ptr<cv::ml::KNearest>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::KNearest> ret = cv::ml::KNearest::load(std::string(filepath));
			Ok(new cv::Ptr<cv::ml::KNearest>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::KNearest>*>))
	}
	
	void cv_ml_LogisticRegression_getLearningRate_const(const cv::ml::LogisticRegression* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getLearningRate();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_LogisticRegression_setLearningRate_double(cv::ml::LogisticRegression* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setLearningRate(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_getIterations_const(const cv::ml::LogisticRegression* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getIterations();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_LogisticRegression_setIterations_int(cv::ml::LogisticRegression* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setIterations(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_getRegularization_const(const cv::ml::LogisticRegression* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getRegularization();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_LogisticRegression_setRegularization_int(cv::ml::LogisticRegression* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setRegularization(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_getTrainMethod_const(const cv::ml::LogisticRegression* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getTrainMethod();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_LogisticRegression_setTrainMethod_int(cv::ml::LogisticRegression* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setTrainMethod(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_getMiniBatchSize_const(const cv::ml::LogisticRegression* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMiniBatchSize();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_LogisticRegression_setMiniBatchSize_int(cv::ml::LogisticRegression* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setMiniBatchSize(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_getTermCriteria_const(const cv::ml::LogisticRegression* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_LogisticRegression_setTermCriteria_TermCriteria(cv::ml::LogisticRegression* instance, cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_LogisticRegression_predict_const_const__InputArrayR_const__OutputArrayR_int(const cv::ml::LogisticRegression* instance, const cv::_InputArray* samples, const cv::_OutputArray* results, int flags, Result<float>* ocvrs_return) {
		try {
			float ret = instance->predict(*samples, *results, flags);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_LogisticRegression_get_learnt_thetas_const(const cv::ml::LogisticRegression* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->get_learnt_thetas();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_LogisticRegression_create(Result<cv::Ptr<cv::ml::LogisticRegression>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::LogisticRegression> ret = cv::ml::LogisticRegression::create();
			Ok(new cv::Ptr<cv::ml::LogisticRegression>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::LogisticRegression>*>))
	}
	
	void cv_ml_LogisticRegression_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::LogisticRegression>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::LogisticRegression> ret = cv::ml::LogisticRegression::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::LogisticRegression>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::LogisticRegression>*>))
	}
	
	void cv_ml_NormalBayesClassifier_predictProb_const_const__InputArrayR_const__OutputArrayR_const__OutputArrayR_int(const cv::ml::NormalBayesClassifier* instance, const cv::_InputArray* inputs, const cv::_OutputArray* outputs, const cv::_OutputArray* outputProbs, int flags, Result<float>* ocvrs_return) {
		try {
			float ret = instance->predictProb(*inputs, *outputs, *outputProbs, flags);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_NormalBayesClassifier_create(Result<cv::Ptr<cv::ml::NormalBayesClassifier>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::NormalBayesClassifier> ret = cv::ml::NormalBayesClassifier::create();
			Ok(new cv::Ptr<cv::ml::NormalBayesClassifier>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::NormalBayesClassifier>*>))
	}
	
	void cv_ml_NormalBayesClassifier_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::NormalBayesClassifier>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::NormalBayesClassifier> ret = cv::ml::NormalBayesClassifier::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::NormalBayesClassifier>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::NormalBayesClassifier>*>))
	}
	
	double cv_ml_ParamGrid_getPropMinVal_const(const cv::ml::ParamGrid* instance) {
			double ret = instance->minVal;
			return ret;
	}
	
	void cv_ml_ParamGrid_setPropMinVal_double(cv::ml::ParamGrid* instance, double val) {
			instance->minVal = val;
	}
	
	double cv_ml_ParamGrid_getPropMaxVal_const(const cv::ml::ParamGrid* instance) {
			double ret = instance->maxVal;
			return ret;
	}
	
	void cv_ml_ParamGrid_setPropMaxVal_double(cv::ml::ParamGrid* instance, double val) {
			instance->maxVal = val;
	}
	
	double cv_ml_ParamGrid_getPropLogStep_const(const cv::ml::ParamGrid* instance) {
			double ret = instance->logStep;
			return ret;
	}
	
	void cv_ml_ParamGrid_setPropLogStep_double(cv::ml::ParamGrid* instance, double val) {
			instance->logStep = val;
	}
	
	void cv_ParamGrid_delete(cv::ml::ParamGrid* instance) {
		delete instance;
	}
	void cv_ml_ParamGrid_ParamGrid(Result<cv::ml::ParamGrid*>* ocvrs_return) {
		try {
			cv::ml::ParamGrid* ret = new cv::ml::ParamGrid();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::ml::ParamGrid*>))
	}
	
	void cv_ml_ParamGrid_ParamGrid_double_double_double(double _minVal, double _maxVal, double _logStep, Result<cv::ml::ParamGrid*>* ocvrs_return) {
		try {
			cv::ml::ParamGrid* ret = new cv::ml::ParamGrid(_minVal, _maxVal, _logStep);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::ml::ParamGrid*>))
	}
	
	void cv_ml_ParamGrid_create_double_double_double(double minVal, double maxVal, double logstep, Result<cv::Ptr<cv::ml::ParamGrid>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::ParamGrid> ret = cv::ml::ParamGrid::create(minVal, maxVal, logstep);
			Ok(new cv::Ptr<cv::ml::ParamGrid>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::ParamGrid>*>))
	}
	
	void cv_ml_RTrees_getCalculateVarImportance_const(const cv::ml::RTrees* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getCalculateVarImportance();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_RTrees_setCalculateVarImportance_bool(cv::ml::RTrees* instance, bool val, Result_void* ocvrs_return) {
		try {
			instance->setCalculateVarImportance(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_RTrees_getActiveVarCount_const(const cv::ml::RTrees* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getActiveVarCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_RTrees_setActiveVarCount_int(cv::ml::RTrees* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setActiveVarCount(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_RTrees_getTermCriteria_const(const cv::ml::RTrees* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_RTrees_setTermCriteria_const_TermCriteriaR(cv::ml::RTrees* instance, const cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_RTrees_getVarImportance_const(const cv::ml::RTrees* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getVarImportance();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_RTrees_getVotes_const_const__InputArrayR_const__OutputArrayR_int(const cv::ml::RTrees* instance, const cv::_InputArray* samples, const cv::_OutputArray* results, int flags, Result_void* ocvrs_return) {
		try {
			instance->getVotes(*samples, *results, flags);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_RTrees_getOOBError_const(const cv::ml::RTrees* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getOOBError();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_RTrees_create(Result<cv::Ptr<cv::ml::RTrees>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::RTrees> ret = cv::ml::RTrees::create();
			Ok(new cv::Ptr<cv::ml::RTrees>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::RTrees>*>))
	}
	
	void cv_ml_RTrees_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::RTrees>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::RTrees> ret = cv::ml::RTrees::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::RTrees>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::RTrees>*>))
	}
	
	void cv_ml_SVM_getType_const(const cv::ml::SVM* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_SVM_setType_int(cv::ml::SVM* instance, int val, Result_void* ocvrs_return) {
		try {
			instance->setType(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getGamma_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getGamma();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setGamma_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setGamma(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getCoef0_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getCoef0();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setCoef0_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setCoef0(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getDegree_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getDegree();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setDegree_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setDegree(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getC_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getC();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setC_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setC(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getNu_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getNu();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setNu_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setNu(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getP_const(const cv::ml::SVM* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getP();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_setP_double(cv::ml::SVM* instance, double val, Result_void* ocvrs_return) {
		try {
			instance->setP(val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getClassWeights_const(const cv::ml::SVM* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getClassWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_SVM_setClassWeights_const_MatR(cv::ml::SVM* instance, const cv::Mat* val, Result_void* ocvrs_return) {
		try {
			instance->setClassWeights(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getTermCriteria_const(const cv::ml::SVM* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_SVM_setTermCriteria_const_TermCriteriaR(cv::ml::SVM* instance, const cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_getKernelType_const(const cv::ml::SVM* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getKernelType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_SVM_setKernel_int(cv::ml::SVM* instance, int kernelType, Result_void* ocvrs_return) {
		try {
			instance->setKernel(kernelType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_setCustomKernel_const_Ptr_Kernel_R(cv::ml::SVM* instance, const cv::Ptr<cv::ml::SVM::Kernel>* _kernel, Result_void* ocvrs_return) {
		try {
			instance->setCustomKernel(*_kernel);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVM_trainAuto_const_Ptr_TrainData_R_int_ParamGrid_ParamGrid_ParamGrid_ParamGrid_ParamGrid_ParamGrid_bool(cv::ml::SVM* instance, const cv::Ptr<cv::ml::TrainData>* data, int kFold, cv::ml::ParamGrid* Cgrid, cv::ml::ParamGrid* gammaGrid, cv::ml::ParamGrid* pGrid, cv::ml::ParamGrid* nuGrid, cv::ml::ParamGrid* coeffGrid, cv::ml::ParamGrid* degreeGrid, bool balanced, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->trainAuto(*data, kFold, *Cgrid, *gammaGrid, *pGrid, *nuGrid, *coeffGrid, *degreeGrid, balanced);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_SVM_trainAuto_const__InputArrayR_int_const__InputArrayR_int_Ptr_ParamGrid__Ptr_ParamGrid__Ptr_ParamGrid__Ptr_ParamGrid__Ptr_ParamGrid__Ptr_ParamGrid__bool(cv::ml::SVM* instance, const cv::_InputArray* samples, int layout, const cv::_InputArray* responses, int kFold, cv::Ptr<cv::ml::ParamGrid>* Cgrid, cv::Ptr<cv::ml::ParamGrid>* gammaGrid, cv::Ptr<cv::ml::ParamGrid>* pGrid, cv::Ptr<cv::ml::ParamGrid>* nuGrid, cv::Ptr<cv::ml::ParamGrid>* coeffGrid, cv::Ptr<cv::ml::ParamGrid>* degreeGrid, bool balanced, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->trainAuto(*samples, layout, *responses, kFold, *Cgrid, *gammaGrid, *pGrid, *nuGrid, *coeffGrid, *degreeGrid, balanced);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_SVM_getSupportVectors_const(const cv::ml::SVM* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getSupportVectors();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_SVM_getUncompressedSupportVectors_const(const cv::ml::SVM* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getUncompressedSupportVectors();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_SVM_getDecisionFunction_const_int_const__OutputArrayR_const__OutputArrayR(const cv::ml::SVM* instance, int i, const cv::_OutputArray* alpha, const cv::_OutputArray* svidx, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getDecisionFunction(i, *alpha, *svidx);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
	void cv_ml_SVM_getDefaultGrid_int(int param_id, Result<cv::ml::ParamGrid*>* ocvrs_return) {
		try {
			cv::ml::ParamGrid ret = cv::ml::SVM::getDefaultGrid(param_id);
			Ok(new cv::ml::ParamGrid(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::ml::ParamGrid*>))
	}
	
	void cv_ml_SVM_getDefaultGridPtr_int(int param_id, Result<cv::Ptr<cv::ml::ParamGrid>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::ParamGrid> ret = cv::ml::SVM::getDefaultGridPtr(param_id);
			Ok(new cv::Ptr<cv::ml::ParamGrid>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::ParamGrid>*>))
	}
	
	void cv_ml_SVM_create(Result<cv::Ptr<cv::ml::SVM>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::SVM> ret = cv::ml::SVM::create();
			Ok(new cv::Ptr<cv::ml::SVM>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::SVM>*>))
	}
	
	void cv_ml_SVM_load_const_StringR(const char* filepath, Result<cv::Ptr<cv::ml::SVM>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::SVM> ret = cv::ml::SVM::load(std::string(filepath));
			Ok(new cv::Ptr<cv::ml::SVM>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::SVM>*>))
	}
	
	void cv_ml_SVM_Kernel_getType_const(const cv::ml::SVM::Kernel* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_SVM_Kernel_calc_int_int_const_floatX_const_floatX_floatX(cv::ml::SVM::Kernel* instance, int vcount, int n, const float* vecs, const float* another, float* results, Result_void* ocvrs_return) {
		try {
			instance->calc(vcount, n, vecs, another, results);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getWeights(cv::ml::SVMSGD* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_SVMSGD_getShift(cv::ml::SVMSGD* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getShift();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_SVMSGD_create(Result<cv::Ptr<cv::ml::SVMSGD>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::SVMSGD> ret = cv::ml::SVMSGD::create();
			Ok(new cv::Ptr<cv::ml::SVMSGD>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::SVMSGD>*>))
	}
	
	void cv_ml_SVMSGD_load_const_StringR_const_StringR(const char* filepath, const char* nodeName, Result<cv::Ptr<cv::ml::SVMSGD>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::SVMSGD> ret = cv::ml::SVMSGD::load(std::string(filepath), std::string(nodeName));
			Ok(new cv::Ptr<cv::ml::SVMSGD>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::SVMSGD>*>))
	}
	
	void cv_ml_SVMSGD_setOptimalParameters_int_int(cv::ml::SVMSGD* instance, int svmsgdType, int marginType, Result_void* ocvrs_return) {
		try {
			instance->setOptimalParameters(svmsgdType, marginType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getSvmsgdType_const(const cv::ml::SVMSGD* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getSvmsgdType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_SVMSGD_setSvmsgdType_int(cv::ml::SVMSGD* instance, int svmsgdType, Result_void* ocvrs_return) {
		try {
			instance->setSvmsgdType(svmsgdType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getMarginType_const(const cv::ml::SVMSGD* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getMarginType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_SVMSGD_setMarginType_int(cv::ml::SVMSGD* instance, int marginType, Result_void* ocvrs_return) {
		try {
			instance->setMarginType(marginType);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getMarginRegularization_const(const cv::ml::SVMSGD* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getMarginRegularization();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_SVMSGD_setMarginRegularization_float(cv::ml::SVMSGD* instance, float marginRegularization, Result_void* ocvrs_return) {
		try {
			instance->setMarginRegularization(marginRegularization);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getInitialStepSize_const(const cv::ml::SVMSGD* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getInitialStepSize();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_SVMSGD_setInitialStepSize_float(cv::ml::SVMSGD* instance, float InitialStepSize, Result_void* ocvrs_return) {
		try {
			instance->setInitialStepSize(InitialStepSize);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getStepDecreasingPower_const(const cv::ml::SVMSGD* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getStepDecreasingPower();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_SVMSGD_setStepDecreasingPower_float(cv::ml::SVMSGD* instance, float stepDecreasingPower, Result_void* ocvrs_return) {
		try {
			instance->setStepDecreasingPower(stepDecreasingPower);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_SVMSGD_getTermCriteria_const(const cv::ml::SVMSGD* instance, Result<cv::TermCriteria>* ocvrs_return) {
		try {
			cv::TermCriteria ret = instance->getTermCriteria();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::TermCriteria>))
	}
	
	void cv_ml_SVMSGD_setTermCriteria_const_TermCriteriaR(cv::ml::SVMSGD* instance, const cv::TermCriteria* val, Result_void* ocvrs_return) {
		try {
			instance->setTermCriteria(*val);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_StatModel_getVarCount_const(const cv::ml::StatModel* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getVarCount();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_StatModel_empty_const(const cv::ml::StatModel* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->empty();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_StatModel_isTrained_const(const cv::ml::StatModel* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isTrained();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_StatModel_isClassifier_const(const cv::ml::StatModel* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->isClassifier();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_StatModel_train_const_Ptr_TrainData_R_int(cv::ml::StatModel* instance, const cv::Ptr<cv::ml::TrainData>* trainData, int flags, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->train(*trainData, flags);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_StatModel_train_const__InputArrayR_int_const__InputArrayR(cv::ml::StatModel* instance, const cv::_InputArray* samples, int layout, const cv::_InputArray* responses, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->train(*samples, layout, *responses);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ml_StatModel_calcError_const_const_Ptr_TrainData_R_bool_const__OutputArrayR(const cv::ml::StatModel* instance, const cv::Ptr<cv::ml::TrainData>* data, bool test, const cv::_OutputArray* resp, Result<float>* ocvrs_return) {
		try {
			float ret = instance->calcError(*data, test, *resp);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_StatModel_predict_const_const__InputArrayR_const__OutputArrayR_int(const cv::ml::StatModel* instance, const cv::_InputArray* samples, const cv::_OutputArray* results, int flags, Result<float>* ocvrs_return) {
		try {
			float ret = instance->predict(*samples, *results, flags);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_TrainData_missingValue(Result<float>* ocvrs_return) {
		try {
			float ret = cv::ml::TrainData::missingValue();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ml_TrainData_getLayout_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getLayout();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getNTrainSamples_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNTrainSamples();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getNTestSamples_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNTestSamples();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getNSamples_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNSamples();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getNVars_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNVars();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getNAllVars_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNAllVars();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getSample_const_const__InputArrayR_int_floatX(const cv::ml::TrainData* instance, const cv::_InputArray* varIdx, int sidx, float* buf, Result_void* ocvrs_return) {
		try {
			instance->getSample(*varIdx, sidx, buf);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_getSamples_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getSamples();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getMissing_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getMissing();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTrainSamples_const_int_bool_bool(const cv::ml::TrainData* instance, int layout, bool compressSamples, bool compressVars, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTrainSamples(layout, compressSamples, compressVars);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTrainResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTrainResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTrainNormCatResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTrainNormCatResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTestResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTestResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTestNormCatResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTestNormCatResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getNormCatResponses_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getNormCatResponses();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getSampleWeights_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getSampleWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTrainSampleWeights_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTrainSampleWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTestSampleWeights_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTestSampleWeights();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getVarIdx_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getVarIdx();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getVarType_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getVarType();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getVarSymbolFlags_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getVarSymbolFlags();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getResponseType_const(const cv::ml::TrainData* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getResponseType();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getTrainSampleIdx_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTrainSampleIdx();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getTestSampleIdx_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTestSampleIdx();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getValues_const_int_const__InputArrayR_floatX(const cv::ml::TrainData* instance, int vi, const cv::_InputArray* sidx, float* values, Result_void* ocvrs_return) {
		try {
			instance->getValues(vi, *sidx, values);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_getNormCatValues_const_int_const__InputArrayR_intX(const cv::ml::TrainData* instance, int vi, const cv::_InputArray* sidx, int* values, Result_void* ocvrs_return) {
		try {
			instance->getNormCatValues(vi, *sidx, values);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_getDefaultSubstValues_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getDefaultSubstValues();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getCatCount_const_int(const cv::ml::TrainData* instance, int vi, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getCatCount(vi);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ml_TrainData_getClassLabels_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getClassLabels();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getCatOfs_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getCatOfs();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getCatMap_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getCatMap();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_setTrainTestSplit_int_bool(cv::ml::TrainData* instance, int count, bool shuffle, Result_void* ocvrs_return) {
		try {
			instance->setTrainTestSplit(count, shuffle);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_setTrainTestSplitRatio_double_bool(cv::ml::TrainData* instance, double ratio, bool shuffle, Result_void* ocvrs_return) {
		try {
			instance->setTrainTestSplitRatio(ratio, shuffle);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_shuffleTrainTest(cv::ml::TrainData* instance, Result_void* ocvrs_return) {
		try {
			instance->shuffleTrainTest();
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_getTestSamples_const(const cv::ml::TrainData* instance, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = instance->getTestSamples();
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getNames_const_vector_String_R(const cv::ml::TrainData* instance, std::vector<cv::String>* names, Result_void* ocvrs_return) {
		try {
			instance->getNames(*names);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ml_TrainData_getSubVector_const_MatR_const_MatR(const cv::Mat* vec, const cv::Mat* idx, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::ml::TrainData::getSubVector(*vec, *idx);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_getSubMatrix_const_MatR_const_MatR_int(const cv::Mat* matrix, const cv::Mat* idx, int layout, Result<cv::Mat*>* ocvrs_return) {
		try {
			cv::Mat ret = cv::ml::TrainData::getSubMatrix(*matrix, *idx, layout);
			Ok(new cv::Mat(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Mat*>))
	}
	
	void cv_ml_TrainData_loadFromCSV_const_StringR_int_int_int_const_StringR_char_char(const char* filename, int headerLineCount, int responseStartIdx, int responseEndIdx, const char* varTypeSpec, char delimiter, char missch, Result<cv::Ptr<cv::ml::TrainData>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::TrainData> ret = cv::ml::TrainData::loadFromCSV(std::string(filename), headerLineCount, responseStartIdx, responseEndIdx, std::string(varTypeSpec), delimiter, missch);
			Ok(new cv::Ptr<cv::ml::TrainData>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::TrainData>*>))
	}
	
	void cv_ml_TrainData_create_const__InputArrayR_int_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__InputArrayR_const__InputArrayR(const cv::_InputArray* samples, int layout, const cv::_InputArray* responses, const cv::_InputArray* varIdx, const cv::_InputArray* sampleIdx, const cv::_InputArray* sampleWeights, const cv::_InputArray* varType, Result<cv::Ptr<cv::ml::TrainData>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ml::TrainData> ret = cv::ml::TrainData::create(*samples, layout, *responses, *varIdx, *sampleIdx, *sampleWeights, *varType);
			Ok(new cv::Ptr<cv::ml::TrainData>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ml::TrainData>*>))
	}
	
}
