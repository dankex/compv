#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

using namespace std;
using namespace cv;

namespace HandGestureSVM {

const int FEATURE_LENGTH = 7;
const int NUM_GESTURE_TYPES = 3;

void computeHuMomentFeatures(string filePath, vector<double>& features) {
	ifstream pointFile;
	pointFile.open(filePath.c_str(), ifstream::in);

	int numPoints;
	pointFile >> numPoints;

	vector<Point> pointList;
	Point point;
	for (int i = 0; i < numPoints; ++i) {
		pointFile >> point.x >> point.y;
		pointList.push_back(point);
	}

	Moments m = moments(pointList);
	HuMoments(m, features);

//	cout << "Hu Moments:" << endl;
//	for (size_t i = 0; i < features.size(); ++i) {
//		cout << features[i] << endl;
//	}
//	cout << endl;
}

int main(int argc, char* argv[]) {
	string trainingSampleFileName = "training_samples_run10.txt";
	string testSampleFileName = "test_samples_run10.txt";

	int numTrainingSamples, numTestSamples;

	// load the training data
	ifstream sampleFile;

	sampleFile.open(trainingSampleFileName.c_str(), ifstream::in);
	sampleFile >> numTrainingSamples;

	Mat trainingData(numTrainingSamples, FEATURE_LENGTH, CV_32FC1);
	Mat trainingLabels(numTrainingSamples, 1, CV_32FC1);

	for (int i = 0; i < numTrainingSamples; ++i) {
		string filePath;
		int label;
		sampleFile >> filePath >> label;

		trainingLabels.at<float>(i) = label;

		vector<double> features;
		computeHuMomentFeatures(filePath, features);

		for (int j = 0; j < FEATURE_LENGTH; ++j) {
			trainingData.at<float>(i, j) = (float) features[j];
		}
	}

	sampleFile.close();

	// set up SVM's parameter
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::RBF;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

	// train the SVM
	cout << "training stage ..." << endl;
	CvSVM SVM;
	SVM.train(trainingData, trainingLabels, Mat(), Mat(), params);
	SVM.save("hand_gesture_svm_model.xml");

	// load the test data
	sampleFile.open(testSampleFileName.c_str(), ifstream::in);

	sampleFile >> numTestSamples;

	Mat testData(numTestSamples, FEATURE_LENGTH, CV_32FC1);
	Mat testLabels(numTestSamples, 1, CV_32FC1);
	Mat predictedLabels(numTestSamples, 1, CV_32FC1, Scalar(0));

	for (int i = 0; i < numTestSamples; ++i) {
		string filePath;
		int label;
		sampleFile >> filePath >> label;

		testLabels.at<float>(i) = label;

		vector<double> features;
		computeHuMomentFeatures(filePath, features);

		for (int j = 0; j < FEATURE_LENGTH; ++j) {
			testData.at<float>(i, j) = (float) features[j];
		}
	}

	sampleFile.close();

	cout << "prediction stage ..." << endl;
	SVM.predict(testData, predictedLabels);

	Mat confusionMatrix(NUM_GESTURE_TYPES, NUM_GESTURE_TYPES, CV_32FC1, Scalar(0));

	int numCorrectPredictions = 0;
	for (int i = 0; i < numTestSamples; ++i) {
		if (testLabels.at<float>(i) == predictedLabels.at<float>(i)) {
			numCorrectPredictions++;
		}

		int row = testLabels.at<float>(i)-1;
		int col = predictedLabels.at<float>(i)-1;
		confusionMatrix.at<float>(row,col)++;
	}

	cout << "% classification accuracy = "
			<< numCorrectPredictions * 100.0 / numTestSamples << endl;

	cout << "Confusion matrix:" << endl;

	cout << setprecision(2) << fixed;
	int numTestSamplesPerGestureType = numTestSamples / NUM_GESTURE_TYPES;
	for (int i=0; i<NUM_GESTURE_TYPES; ++i) {
		for (int j=0; j<NUM_GESTURE_TYPES; ++j) {
			confusionMatrix.at<float>(i,j) *= (100.0 / numTestSamplesPerGestureType);
			cout << setw(6) << setfill(' ') << confusionMatrix.at<float>(i,j);
			cout << "  ";
		}
		cout << endl;
	}
}

}
