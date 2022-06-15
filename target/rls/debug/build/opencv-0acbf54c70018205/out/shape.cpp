#include "ocvrs_common.hpp"
#include <opencv2/shape.hpp>
#include "shape_types.hpp"

extern "C" {
	void cv_EMDL1_const__InputArrayR_const__InputArrayR(const cv::_InputArray* signature1, const cv::_InputArray* signature2, Result<float>* ocvrs_return) {
		try {
			float ret = cv::EMDL1(*signature1, *signature2);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_createAffineTransformer_bool(bool fullAffine, Result<cv::Ptr<cv::AffineTransformer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::AffineTransformer> ret = cv::createAffineTransformer(fullAffine);
			Ok(new cv::Ptr<cv::AffineTransformer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::AffineTransformer>*>))
	}
	
	void cv_createChiHistogramCostExtractor_int_float(int nDummies, float defaultCost, Result<cv::Ptr<cv::HistogramCostExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HistogramCostExtractor> ret = cv::createChiHistogramCostExtractor(nDummies, defaultCost);
			Ok(new cv::Ptr<cv::HistogramCostExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HistogramCostExtractor>*>))
	}
	
	void cv_createEMDHistogramCostExtractor_int_int_float(int flag, int nDummies, float defaultCost, Result<cv::Ptr<cv::HistogramCostExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HistogramCostExtractor> ret = cv::createEMDHistogramCostExtractor(flag, nDummies, defaultCost);
			Ok(new cv::Ptr<cv::HistogramCostExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HistogramCostExtractor>*>))
	}
	
	void cv_createEMDL1HistogramCostExtractor_int_float(int nDummies, float defaultCost, Result<cv::Ptr<cv::HistogramCostExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HistogramCostExtractor> ret = cv::createEMDL1HistogramCostExtractor(nDummies, defaultCost);
			Ok(new cv::Ptr<cv::HistogramCostExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HistogramCostExtractor>*>))
	}
	
	void cv_createHausdorffDistanceExtractor_int_float(int distanceFlag, float rankProp, Result<cv::Ptr<cv::HausdorffDistanceExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HausdorffDistanceExtractor> ret = cv::createHausdorffDistanceExtractor(distanceFlag, rankProp);
			Ok(new cv::Ptr<cv::HausdorffDistanceExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HausdorffDistanceExtractor>*>))
	}
	
	void cv_createNormHistogramCostExtractor_int_int_float(int flag, int nDummies, float defaultCost, Result<cv::Ptr<cv::HistogramCostExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HistogramCostExtractor> ret = cv::createNormHistogramCostExtractor(flag, nDummies, defaultCost);
			Ok(new cv::Ptr<cv::HistogramCostExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HistogramCostExtractor>*>))
	}
	
	void cv_createShapeContextDistanceExtractor_int_int_float_float_int_const_Ptr_HistogramCostExtractor_R_const_Ptr_ShapeTransformer_R(int nAngularBins, int nRadialBins, float innerRadius, float outerRadius, int iterations, const cv::Ptr<cv::HistogramCostExtractor>* comparer, const cv::Ptr<cv::ShapeTransformer>* transformer, Result<cv::Ptr<cv::ShapeContextDistanceExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ShapeContextDistanceExtractor> ret = cv::createShapeContextDistanceExtractor(nAngularBins, nRadialBins, innerRadius, outerRadius, iterations, *comparer, *transformer);
			Ok(new cv::Ptr<cv::ShapeContextDistanceExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ShapeContextDistanceExtractor>*>))
	}
	
	void cv_createThinPlateSplineShapeTransformer_double(double regularizationParameter, Result<cv::Ptr<cv::ThinPlateSplineShapeTransformer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ThinPlateSplineShapeTransformer> ret = cv::createThinPlateSplineShapeTransformer(regularizationParameter);
			Ok(new cv::Ptr<cv::ThinPlateSplineShapeTransformer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ThinPlateSplineShapeTransformer>*>))
	}
	
	void cv_AffineTransformer_setFullAffine_bool(cv::AffineTransformer* instance, bool fullAffine, Result_void* ocvrs_return) {
		try {
			instance->setFullAffine(fullAffine);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_AffineTransformer_getFullAffine_const(const cv::AffineTransformer* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getFullAffine();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_EMDHistogramCostExtractor_setNormFlag_int(cv::EMDHistogramCostExtractor* instance, int flag, Result_void* ocvrs_return) {
		try {
			instance->setNormFlag(flag);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_EMDHistogramCostExtractor_getNormFlag_const(const cv::EMDHistogramCostExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNormFlag();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_HausdorffDistanceExtractor_setDistanceFlag_int(cv::HausdorffDistanceExtractor* instance, int distanceFlag, Result_void* ocvrs_return) {
		try {
			instance->setDistanceFlag(distanceFlag);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_HausdorffDistanceExtractor_getDistanceFlag_const(const cv::HausdorffDistanceExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getDistanceFlag();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_HausdorffDistanceExtractor_setRankProportion_float(cv::HausdorffDistanceExtractor* instance, float rankProportion, Result_void* ocvrs_return) {
		try {
			instance->setRankProportion(rankProportion);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_HausdorffDistanceExtractor_getRankProportion_const(const cv::HausdorffDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getRankProportion();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_HistogramCostExtractor_buildCostMatrix_const__InputArrayR_const__InputArrayR_const__OutputArrayR(cv::HistogramCostExtractor* instance, const cv::_InputArray* descriptors1, const cv::_InputArray* descriptors2, const cv::_OutputArray* costMatrix, Result_void* ocvrs_return) {
		try {
			instance->buildCostMatrix(*descriptors1, *descriptors2, *costMatrix);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_HistogramCostExtractor_setNDummies_int(cv::HistogramCostExtractor* instance, int nDummies, Result_void* ocvrs_return) {
		try {
			instance->setNDummies(nDummies);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_HistogramCostExtractor_getNDummies_const(const cv::HistogramCostExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNDummies();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_HistogramCostExtractor_setDefaultCost_float(cv::HistogramCostExtractor* instance, float defaultCost, Result_void* ocvrs_return) {
		try {
			instance->setDefaultCost(defaultCost);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_HistogramCostExtractor_getDefaultCost_const(const cv::HistogramCostExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getDefaultCost();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_NormHistogramCostExtractor_setNormFlag_int(cv::NormHistogramCostExtractor* instance, int flag, Result_void* ocvrs_return) {
		try {
			instance->setNormFlag(flag);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_NormHistogramCostExtractor_getNormFlag_const(const cv::NormHistogramCostExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getNormFlag();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ShapeContextDistanceExtractor_setAngularBins_int(cv::ShapeContextDistanceExtractor* instance, int nAngularBins, Result_void* ocvrs_return) {
		try {
			instance->setAngularBins(nAngularBins);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getAngularBins_const(const cv::ShapeContextDistanceExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getAngularBins();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ShapeContextDistanceExtractor_setRadialBins_int(cv::ShapeContextDistanceExtractor* instance, int nRadialBins, Result_void* ocvrs_return) {
		try {
			instance->setRadialBins(nRadialBins);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getRadialBins_const(const cv::ShapeContextDistanceExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getRadialBins();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ShapeContextDistanceExtractor_setInnerRadius_float(cv::ShapeContextDistanceExtractor* instance, float innerRadius, Result_void* ocvrs_return) {
		try {
			instance->setInnerRadius(innerRadius);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getInnerRadius_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getInnerRadius();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setOuterRadius_float(cv::ShapeContextDistanceExtractor* instance, float outerRadius, Result_void* ocvrs_return) {
		try {
			instance->setOuterRadius(outerRadius);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getOuterRadius_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getOuterRadius();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setRotationInvariant_bool(cv::ShapeContextDistanceExtractor* instance, bool rotationInvariant, Result_void* ocvrs_return) {
		try {
			instance->setRotationInvariant(rotationInvariant);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getRotationInvariant_const(const cv::ShapeContextDistanceExtractor* instance, Result<bool>* ocvrs_return) {
		try {
			bool ret = instance->getRotationInvariant();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<bool>))
	}
	
	void cv_ShapeContextDistanceExtractor_setShapeContextWeight_float(cv::ShapeContextDistanceExtractor* instance, float shapeContextWeight, Result_void* ocvrs_return) {
		try {
			instance->setShapeContextWeight(shapeContextWeight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getShapeContextWeight_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getShapeContextWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setImageAppearanceWeight_float(cv::ShapeContextDistanceExtractor* instance, float imageAppearanceWeight, Result_void* ocvrs_return) {
		try {
			instance->setImageAppearanceWeight(imageAppearanceWeight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getImageAppearanceWeight_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getImageAppearanceWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setBendingEnergyWeight_float(cv::ShapeContextDistanceExtractor* instance, float bendingEnergyWeight, Result_void* ocvrs_return) {
		try {
			instance->setBendingEnergyWeight(bendingEnergyWeight);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getBendingEnergyWeight_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getBendingEnergyWeight();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setImages_const__InputArrayR_const__InputArrayR(cv::ShapeContextDistanceExtractor* instance, const cv::_InputArray* image1, const cv::_InputArray* image2, Result_void* ocvrs_return) {
		try {
			instance->setImages(*image1, *image2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getImages_const_const__OutputArrayR_const__OutputArrayR(const cv::ShapeContextDistanceExtractor* instance, const cv::_OutputArray* image1, const cv::_OutputArray* image2, Result_void* ocvrs_return) {
		try {
			instance->getImages(*image1, *image2);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_setIterations_int(cv::ShapeContextDistanceExtractor* instance, int iterations, Result_void* ocvrs_return) {
		try {
			instance->setIterations(iterations);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getIterations_const(const cv::ShapeContextDistanceExtractor* instance, Result<int>* ocvrs_return) {
		try {
			int ret = instance->getIterations();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<int>))
	}
	
	void cv_ShapeContextDistanceExtractor_setCostExtractor_Ptr_HistogramCostExtractor_(cv::ShapeContextDistanceExtractor* instance, cv::Ptr<cv::HistogramCostExtractor>* comparer, Result_void* ocvrs_return) {
		try {
			instance->setCostExtractor(*comparer);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getCostExtractor_const(const cv::ShapeContextDistanceExtractor* instance, Result<cv::Ptr<cv::HistogramCostExtractor>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::HistogramCostExtractor> ret = instance->getCostExtractor();
			Ok(new cv::Ptr<cv::HistogramCostExtractor>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::HistogramCostExtractor>*>))
	}
	
	void cv_ShapeContextDistanceExtractor_setStdDev_float(cv::ShapeContextDistanceExtractor* instance, float sigma, Result_void* ocvrs_return) {
		try {
			instance->setStdDev(sigma);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getStdDev_const(const cv::ShapeContextDistanceExtractor* instance, Result<float>* ocvrs_return) {
		try {
			float ret = instance->getStdDev();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeContextDistanceExtractor_setTransformAlgorithm_Ptr_ShapeTransformer_(cv::ShapeContextDistanceExtractor* instance, cv::Ptr<cv::ShapeTransformer>* transformer, Result_void* ocvrs_return) {
		try {
			instance->setTransformAlgorithm(*transformer);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeContextDistanceExtractor_getTransformAlgorithm_const(const cv::ShapeContextDistanceExtractor* instance, Result<cv::Ptr<cv::ShapeTransformer>*>* ocvrs_return) {
		try {
			cv::Ptr<cv::ShapeTransformer> ret = instance->getTransformAlgorithm();
			Ok(new cv::Ptr<cv::ShapeTransformer>(ret), ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<cv::Ptr<cv::ShapeTransformer>*>))
	}
	
	void cv_ShapeDistanceExtractor_computeDistance_const__InputArrayR_const__InputArrayR(cv::ShapeDistanceExtractor* instance, const cv::_InputArray* contour1, const cv::_InputArray* contour2, Result<float>* ocvrs_return) {
		try {
			float ret = instance->computeDistance(*contour1, *contour2);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeTransformer_estimateTransformation_const__InputArrayR_const__InputArrayR_vector_DMatch_R(cv::ShapeTransformer* instance, const cv::_InputArray* transformingShape, const cv::_InputArray* targetShape, std::vector<cv::DMatch>* matches, Result_void* ocvrs_return) {
		try {
			instance->estimateTransformation(*transformingShape, *targetShape, *matches);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ShapeTransformer_applyTransformation_const__InputArrayR_const__OutputArrayR(cv::ShapeTransformer* instance, const cv::_InputArray* input, const cv::_OutputArray* output, Result<float>* ocvrs_return) {
		try {
			float ret = instance->applyTransformation(*input, *output);
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<float>))
	}
	
	void cv_ShapeTransformer_warpImage_const_const__InputArrayR_const__OutputArrayR_int_int_const_ScalarR(const cv::ShapeTransformer* instance, const cv::_InputArray* transformingImage, const cv::_OutputArray* output, int flags, int borderMode, const cv::Scalar* borderValue, Result_void* ocvrs_return) {
		try {
			instance->warpImage(*transformingImage, *output, flags, borderMode, *borderValue);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ThinPlateSplineShapeTransformer_setRegularizationParameter_double(cv::ThinPlateSplineShapeTransformer* instance, double beta, Result_void* ocvrs_return) {
		try {
			instance->setRegularizationParameter(beta);
			Ok(ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result_void))
	}
	
	void cv_ThinPlateSplineShapeTransformer_getRegularizationParameter_const(const cv::ThinPlateSplineShapeTransformer* instance, Result<double>* ocvrs_return) {
		try {
			double ret = instance->getRegularizationParameter();
			Ok(ret, ocvrs_return);
		} OCVRS_CATCH(OCVRS_TYPE(Result<double>))
	}
	
}
