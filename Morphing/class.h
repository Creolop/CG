#include "opencv2/opencv.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

using namespace std;
using namespace cv;

void onMousel(int event,int x,int y,int,void*);
void onMouser(int event,int x,int y,int,void*);

class Line
{
public:
	Point2d P; //start 
	Point2d Q; //end
	// Point2d M; //mid
	double len;
	double degree;
	
	Line();
	Line(Point2d x,Point2d y);
	double Getu(Point2d X);
	double Getv(Point2d X);
	Point2d Get_Point(double u , double v);
	double Get_Weight(Point2d X);
};

class LinePair
{
public:
	Line leftLine;
	Line rightLine;
};

double dotproduct(Point2d x, Point2d y);
double distance(Point2d x, Point2d y);
Vec3b bilinear(Mat S, double X , double Y );
Mat warp(Mat S,Mat D, vector<LinePair> featurelineset);
