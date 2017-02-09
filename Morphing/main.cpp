#include "class.h"


double parameter_a=0.25;
double parameter_b=1.25;
double parameter_p=2;
int frame_number=10;
int lasttime=2;
int curFrame = 0;
Point pre_pt = Point(-1,-1);
Point cur_pt = Point(-1,-1);
fstream file;
Mat src_l;
Mat src_r;

int main(int argc,char** argv)
{
	// cout << "Input frame_number:" << endl;
	// cin >> frame_number;
	// cout << "Input time:" << endl;
	// cin >> lasttime;
	// cout << "Input a,b,p:(recommandation0.25,1.25,0.0) " << endl;
	// cin >> parameter_a >> parameter_b >> parameter_p;

	Mat source_image = imread("./sully1.jpg");
	Mat destination_image = imread("./sully2.jpg");
	vector<LinePair> featurelineset;
	
	Line featureline1 = Line(Point2d(139,26),Point2d(400,52)); 
	Line featureline2 = Line(Point2d(147,11),Point2d(429,36));
	LinePair featurelinepair;
	featurelinepair.leftLine = featureline1;
	featurelinepair.rightLine = featureline2;
	

	src_l = imread("sully1.jpg");
	src_r = imread("sully2.jpg");
    file.open("featureline.txt",ios_base::out);
    if(file.fail())
    {
        cerr<<"open file fails"<<endl;
        file.close();
        return -1;
    }

	namedWindow("src_l",WINDOW_AUTOSIZE);
	namedWindow("right",WINDOW_AUTOSIZE);

	imshow("src_l",src_l);
	imshow("src_r",src_r);

	setMouseCallback("src_l",onMousel,0);
	setMouseCallback("src_r",onMouser,0);

	waitKey(0);
	destroyAllWindows();
	file.close();


	file.open("featureline.txt",ios::in);
	if(file.fail())
	{
		file.close();
	}
	while(!file.eof())
	{
		file>>featureline1.P.x>>featureline1.P.y;
		file>>featureline1.Q.x>>featureline1.Q.y;
		file>>featureline2.P.x>>featureline2.P.y;
		file>>featureline2.Q.x>>featureline2.Q.y;
		featurelinepair.leftLine = featureline1;
		featurelinepair.rightLine = featureline2;
		featurelineset.push_back(featurelinepair);
	}
	featurelineset.pop_back();
	for(int i = 0;i<featurelineset.size();i++)
	{
		cout<<featurelineset[i].leftLine.P.x<<" "<<featurelineset[i].leftLine.P.y<<" ";
		cout<<featurelineset[i].leftLine.Q.x<<" "<<featurelineset[i].leftLine.Q.y<<" ";
		cout<<featurelineset[i].rightLine.P.x<<" "<<featurelineset[i].rightLine.P.y<<" ";
		cout<<featurelineset[i].rightLine.Q.x<<" "<<featurelineset[i].rightLine.Q.y<<endl;
	}



	file.close();

	for(curFrame = 0;curFrame <= (frame_number*lasttime);curFrame++)
	{
		string name("./imageFile/morphing" + std::to_string(curFrame) + ".jpg");
		// double alpha = (double)curFrame/(frame_number*lasttime);
		Mat final = Mat::zeros(source_image.size(),source_image.type());
		final = warp(source_image,destination_image,featurelineset);
		imwrite(name,final);
	}

	VideoWriter writer = VideoWriter("./video/finalvideo.avi", CV_FOURCC('M','J','P','G'), frame_number, source_image.size());
	if(!writer.isOpened())
	{
	std::cerr << "Can not create video file.\n" << std::endl;
	return -1; 
	};

	Mat result_frame;

	for(int i = 0;i <= (frame_number*lasttime);i++)
	{
	std::string name("./imageFile/morphing" + std::to_string(i) + ".jpg");
	result_frame = imread(name,1);
	writer << result_frame;
	}
	return 0;


}