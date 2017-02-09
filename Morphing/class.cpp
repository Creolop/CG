#include "class.h"

extern double parameter_a;
extern double parameter_b;
extern double parameter_p;
extern int frame_number;
extern int lasttime;
extern int curFrame;
extern Point pre_pt;
extern Point cur_pt;
extern fstream file;
extern Mat src_r;
extern Mat src_l;

double dotproduct(Point2d x, Point2d y)
{
   return ( x.x*y.x + x.y*y.y );
}

double distance(Point2d x, Point2d y)
{
   double x_XtoY = y.x - x.x;
   double y_XtoY = y.y - x.y;
   return (sqrt(pow(x_XtoY,2.0)+pow(y_XtoY,2.0)));
}

// Line class function
Line::Line()
{
   P = Point2d(0,0);
   Q = Point2d(0,0);
   // M = Point2d(0,0);
   len = 0;
   degree = 0;
}

Line::Line(Point2d x,Point2d y)
{
   P = x;
   Q = y;
   // M = Point2d((x.x+y.x)/2,(x.y+y.y)/2);
   len = distance(P,Q);
   double X_PtoQ = Q.x - P.x;
   double Y_PtoQ = Q.y - P.x;
   double X_PtoX = x.x - P.x;
   double Y_PtoX = x.y - P.y;
   double u = dotproduct(Point2d(X_PtoX,Y_PtoX),Point2d(X_PtoQ,Y_PtoQ))/(len*len);
   double dist_XtoP = distance(x,P);

   degree = acos(u/dist_XtoP);
}

double Line::Getu(Point2d X){
   /*calculate u*/
   len = distance(P,Q);
   double X_P_x = X.x - P.x; 
   double X_P_y = X.y - P.y;
   double Q_P_x = Q.x - P.x;
   double Q_P_y = Q.y - P.y;
   // cout<<X_P_x<<" "<<X_P_y<<" "<<Q_P_x<<" "<<Q_P_y<<" "<<len<<endl;
   double u = ((X_P_x * Q_P_x) + (X_P_y * Q_P_y)) / (len*len)  ;

   return u ;
}
double Line::Getv(Point2d X){
   len = distance(P,Q);
   double X_P_x = X.x - P.x;
   double X_P_y = X.y - P.y;
   double Q_P_x = Q.x - P.x;
   double Q_P_y = Q.y - P.y;
   double Perp_Q_P_x = Q_P_y ;  
   double Perp_Q_P_y = -Q_P_x ;
   double v = ((X_P_x * Perp_Q_P_x) + (X_P_y * Perp_Q_P_y))/len ; 
   return v ; 
}
Point2d Line::Get_Point(double u , double v){
   len = distance(P,Q);
   double Q_P_x = Q.x - P.x;
   double Q_P_y = Q.y - P.y;
   double Perp_Q_P_x = Q_P_y ;  
   double Perp_Q_P_y = -Q_P_x ;
   double Point_x = P.x + u * (Q.x - P.x) + ((v * Perp_Q_P_x)/len) ;
   double Point_y = P.y + u * (Q.y - P.y) + ((v * Perp_Q_P_y)/len) ;
   Point2d X;
   X.x = Point_x;
   X.y = Point_y;
   return X ;
}

double Line::Get_Weight(Point2d X ){
   double a = parameter_a;
   double b = parameter_b;
   double p = parameter_p;
   double d = 0.0;
   
   double u = Getu(X);
   if(u > 1.0 )
       d = sqrt((X.x - Q.x) * (X.x - Q.x) + (X.y - Q.y) * (X.y - Q.y));
   else if(u < 0)
       d = sqrt((X.x - P.x) * (X.x - P.x) + (X.y - P.y) * (X.y - P.y));
   else
       d = abs(Getv(X));
   

   double weight = pow( pow(len,p)/(a + d) , b);
   return weight; 
}


// warp()
Mat warp(Mat S, Mat D, vector<LinePair> featurelineset)
{
   Mat result1 = Mat::zeros(S.size(),S.type());
   Mat result2 = Mat::zeros(D.size(),D.type());
   Point2d X_l,X_r;
   int F = frame_number*lasttime;
   
    for(int y = 0 ; y < S.rows ; y++)
    {
      for(int x = 0 ; x < S.cols ; x++)
      {

         // double weightSum_l = 0;
         // double weightSum_r = 0;
         double weightSum_l = 0;
         // double weightSum_r = 0;
         
         Point2d XSum_l = Point2d(0,0);
         Point2d XSum_r = Point2d(0,0);
         // cout<<"Point("<<x<<","<<y<<")"<<endl;
         
         for(int k = 0; k < featurelineset.size(); k++)
         {
            Line src_line = featurelineset[k].leftLine;
            Line dst_line = featurelineset[k].rightLine;
            // cout<<k<<endl;

            Point2d deltaP_l = Point2d((dst_line.P.x - src_line.P.x)/(double)(F+1),(dst_line.P.y-src_line.P.y)/(double)(F+1));
            Point2d deltaQ_l = Point2d((dst_line.Q.x - src_line.Q.x)/(double)(F+1),(dst_line.Q.y-src_line.Q.y)/(double)(F+1));

            // Point2d deltaP_r = Point2d((-dst_line.P.x + src_line.P.x)/(double)(F+1),(-dst_line.P.y+src_line.P.y)/(double)(F+1));
            // Point2d deltaQ_r = Point2d((-dst_line.Q.x + src_line.Q.x)/(double)(F+1),(-dst_line.Q.y+src_line.Q.y)/(double)(F+1));

            Line interpolated_line_l = featurelineset[k].leftLine;
            // Line interpolated_line_r = featurelineset[k].leftLine;
            interpolated_line_l.P = Point2d((src_line.P.x + deltaP_l.x*(curFrame+1)),(src_line.P.y + deltaP_l.y*(curFrame+1)));
            interpolated_line_l.Q = Point2d((src_line.Q.x + deltaQ_l.x*(curFrame+1)),(src_line.Q.y + deltaQ_l.y*(curFrame+1)));
            // cout<<interpolated_line_l.P.x<< " "<<interpolated_line_l.P.y<<endl;
            // cout<<interpolated_line_l.Q.x<< " "<<interpolated_line_l.Q.y<<endl;
            // interpolated_line_r.P = Point2d((dst_line.P.x + deltaP_r.x*(curFrame+1)),(dst_line.P.y + deltaP_r.y*(curFrame+1)));
            // interpolated_line_r.Q = Point2d((dst_line.Q.x + deltaQ_r.x*(curFrame+1)),(dst_line.Q.y + deltaQ_r.y*(curFrame+1)));
            double u = interpolated_line_l.Getu(Point2d(x,y));
            double v = interpolated_line_l.Getv(Point2d(x,y));
            // cout<<u<<" "<<v<<endl;
            // double u1 = interpolated_line_r.Getu(Point2d(x,y));
            // double v1 = interpolated_line_r.Getv(Point2d(x,y));

            Point2d src_point = src_line.Get_Point(u,v);
            Point2d dst_point = dst_line.Get_Point(u,v);
            // cout<<"src_point = "<<src_point.x<<","<<src_point.y<<endl;
            // cout<<"dst_point = "<<dst_point.x<<","<<dst_point.y<<endl;

            // double src_weight = dst_line.Get_Weight(Point2d(x,y));
            // double dst_weight = src_line.Get_Weight(Point2d(x,y));
            double weight_l = interpolated_line_l.Get_Weight(Point2d(x,y));
            // cout<<weight_l<<endl;
            // double weight_r = interpolated_line_r.Get_Weight(Point2d(x,y));

            weightSum_l = weightSum_l + weight_l;
            // cout << "weightSum="<<weightSum_l<<endl;
            // weightSum_r += weight_r;
            XSum_l.x = XSum_l.x + (double)((src_point.x) * weight_l);
            XSum_l.y = XSum_l.y + (double)((src_point.y) * weight_l);
            XSum_r.x = XSum_r.x + (double)((dst_point.x) * weight_l);
            XSum_r.y = XSum_r.y + (double)((dst_point.y) * weight_l);
            // cout<<XSum_l.x<<" "<<XSum_l.y<<" "<<XSum_r.x<<" "<<XSum_r.y<<endl;
            
            // weightSum_l += src_weight;
            // weightSum_r += dst_weight;
         }
         X_l = Point2d(XSum_l.x/weightSum_l,XSum_l.y/weightSum_l);
         X_r = Point2d(XSum_r.x/weightSum_l,XSum_r.y/weightSum_l);
         // cout<<X_l.x<<" "<<X_l.y<<endl;
         // cout<<X_r.x<<" "<<X_r.y<<endl;

         if(X_l.x<0)
         {
            X_l.x = 0;
         }
         if(X_l.y<0)
         {
            X_l.y = 0;
         }
         if(X_l.x>= S.cols-1)
         {
            X_l.x = S.cols-1;
         }
         if(X_l.y>= S.rows-1)
         {
            X_l.y = S.rows-1;
         }
         if(X_r.x<0)
         {
            X_r.x = 0;
         }
         if(X_r.y<0)
         {
            X_r.y = 0;
         }
         if(X_r.x>= S.cols-1)
         {
            X_r.x = S.cols-1;
         }
         if(X_r.y>= S.rows-1)
         {
            X_r.y = S.rows-1;
         }
         result1.at<Vec3b>(y,x) = bilinear(S,X_l.x,X_l.y);
         result2.at<Vec3b>(y,x) = bilinear(D,X_r.x,X_r.y);
      }
   }
   string name1("1morphing" + std::to_string(curFrame) + ".jpg");
   string name2("2morphing" + std::to_string(curFrame) + ".jpg");
   imwrite(name1,result1);
   imwrite(name2,result2);
   Mat result = Mat::zeros(S.size(),S.type());
   double alpha = (double)(curFrame+1)/(double)(F+1);
   for(int y = 0; y < S.rows; y++)
   {
      for(int x = 0; x < S.cols; x++)
      {
         for( int c = 0; c < 3; c++ ) 
         {
            result.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( (1-alpha)*( result1.at<Vec3b>(y,x)[c] ) + alpha*(result2.at<Vec3b>(y,x)[c]) );
         }
      }
   }
   return result;
}


// bilinear

Vec3b bilinear(Mat S, double X , double Y ){
     int x_floor = (int)X ;
     int y_floor = (int)Y ;
     int x_ceil = x_floor + 1 ;
     int y_ceil = y_floor + 1 ;
     double a = X - x_floor ;
     double b = Y - y_floor ;
     if(x_ceil >= S.cols-1) 
         x_ceil = S.cols-1 ;
     if(y_ceil >= S.rows-1) 
         y_ceil = S.rows-1 ;
     Vec3b output_scalar ;
     Vec3b leftdown = S.at<Vec3b>(y_floor,x_floor);
     Vec3b lefttop = S.at<Vec3b>(y_ceil,x_floor);
     Vec3b rightdown = S.at<Vec3b>(y_floor,x_ceil);
     Vec3b righttop = S.at<Vec3b>(y_ceil,x_ceil);
     for(int i = 0 ; i < 3 ; i ++)
     {
      output_scalar[i] = (1-a)*(1-b)*leftdown[i] + a*(1-b)*rightdown[i] + a*b*righttop[i] + (1-a)*b*lefttop[i];
     }
     return output_scalar ;
}


// mouse function
void onMousel(int event,int x,int y,int flags,void*)
{

  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
  char temp[50];
  static int counter1 = 0;

    if( event == CV_EVENT_LBUTTONUP )
        {
          cur_pt = Point(x,y);
          circle(src_l,cur_pt,3,Scalar(255,0,0),CV_FILLED,CV_AA,0);
          line(src_l,pre_pt,cur_pt,Scalar(0,255,0),1,CV_AA,0);
          imshow("src_l",src_l);
            file<<x<<" "<<y<<" ";
        }
    else if( event == CV_EVENT_LBUTTONDOWN )
        {
          counter1++;
          pre_pt = Point(x,y);
          sprintf(temp,"(%d)",counter1);
          putText(src_l,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,0,0),1,8);
          circle(src_l,pre_pt,3,Scalar(255,0,0),CV_FILLED,CV_AA,0);
          imshow("src_l",src_l);
          file<<x<<" "<<y<<" ";
        }
}

void onMouser(int event,int x,int y,int flags,void*)
{

  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
  char temp[50];
  static int counter2 = 0;

    if( event == CV_EVENT_LBUTTONUP )
        {
          cur_pt = Point(x,y);
          circle(src_r,cur_pt,3,Scalar(255,0,0),CV_FILLED,CV_AA,0);
          line(src_r,pre_pt,cur_pt,Scalar(0,255,0),1,CV_AA,0);
          imshow("src_r",src_r);
          file<<x<<" "<<y<<" ";
        }
    else if( event == CV_EVENT_LBUTTONDOWN )
        {
          counter2++;
          pre_pt = Point(x,y);
          sprintf(temp,"(%d)",counter2);
          putText(src_r,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,0,0),1,8);
          circle(src_r,pre_pt,3,Scalar(255,0,0),CV_FILLED,CV_AA,0);
          imshow("src_r",src_r);
          file<<x<<" "<<y<<" ";
        }
}






