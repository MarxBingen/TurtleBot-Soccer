#include "daniels/LogReg.h"
//#include <iostream>
LogisticRegression::LogisticRegression()
{
	gewichte.resize(784,0.001f);
}

	//Trainingsdaten geben
void LogisticRegression::learnFromExample(std::vector<unsigned char> *example, int iterations, float learningRate)
{
       	//float likelehood= 1E-128;                           //L(w1,w2,...,wN)
       	float yi = 1.0f;
	float FwVonXi=0.0f;
       	for (int i = 0;i<iterations;i++)
	{
		//likelehood=1E-128;
		FwVonXi = GvonZ(getSumPixelGewichtet(example));
		for (int j=0;j<gewichte.size();j++)
		{
			if (example->at(i)==0)
				gewichte[j] += ( learningRate * (0.0f - FwVonXi) * 1);
			gewichte[j] += ( learningRate * (yi - FwVonXi) * example->at(j));
		}
	}
}

//Prüft, ob es sich bei den Daten um das gesuchte handelt
float LogisticRegression::predict(std::vector<unsigned char> *x)
{
	float r = getSumPixelGewichtet(x);
	return GvonZ(r);
}

float LogisticRegression::getSumPixelGewichtet(std::vector<unsigned char> *x)
{
	float result=0.0f;
	for (int i=0;i<gewichte.size();i++)
	{
		result += (gewichte[i] * float(x->at(i)));
	}
	return result;
}

float LogisticRegression::GvonZ(float z)
{
	float i = pow(M_E,(-z));
	return 1.0f/(1.0f+i);
}
