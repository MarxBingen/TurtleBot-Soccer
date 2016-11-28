#include <vector>
#include <math.h>
class LogisticRegression {
	public:
		std::vector<float> gewichte;
		LogisticRegression();
		void learnFromExample(std::vector<unsigned char> *example, int iterations, float learningRate);
		float predict(std::vector<unsigned char> *x);
		float getSumPixelGewichtet(std::vector<unsigned char> *x);
		float GvonZ(float z);
};
