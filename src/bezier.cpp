#include "../include/interpolation/bezier.hpp"

// Default constructor.
BezierCurve::BezierCurve()
{
  // Object created.
}
// Useful constructor for pure pursuit.
BezierCurve::BezierCurve(nav_msgs::Path path, int numb_ctr_points)
{
  // Save the path from the argument.
  this->BezierCurve::setPath(path);
  // Set the number of control points.
  this->BezierCurve::setNumbCtrlPoints(numb_ctr_points);
  std::cout<<"Bezier Curve Object created"<< std::endl;
}
BezierCurve::~BezierCurve()
{
  // Object Destroyed.
}

// Object changing methods.
// Sets the path.
void BezierCurve::setPath(nav_msgs::Path path)
{
  path_ = path;
}
// Set the number of control points of the bezier-curve.
void BezierCurve::setNumbCtrlPoints(int numb_ctr_points)
{
  numb_ctr_points_ = numb_ctr_points;
}
void BezierCurve::setCenterIndex(int center_index)
{
    center_index_ = center_index;
}
void BezierCurve::setCtrlPoints()
{
  // Clear up what was saved earlier.
  delete[] x_path_;
  delete[] y_path_;
  // Set new size of control points arrays.
  x_path = new float[numb_ctr_points_];
  y_path = new float[numb_ctr_points_];
  // Assign values to resized arrays.
  // Differentiate between numb_ctr_points_ being even or odd.
  if(numb_ctr_points_%2 == 0)
  	{
      // Number of Control Points is even ==> Look one point more ahead than behind!
  		for(int i = 0; i<numb_ctr_points_; i++)
      {
        x_path_[i] = path_.poses[center_index_ - ((numb_ctr_points_ - 1)/2) + 1].pose.position.x;
        y_path_[i] = path_.poses[center_index_ - ((numb_ctr_points_ - 1)/2) + 1].pose.position.y;
      }
  	}
  	else if(numb_ctr_points_%2 != 0)
  	{
      // Number of Control Points is odd.
  		for(int i = 0; i<numb_ctr_points_; i++)
      {
        x_path_[i] = path_.poses[center_index_ - ((numb_ctr_points_ - 1)/2)].pose.position.x;
        y_path_[i] = path_.poses[center_index_ - ((numb_ctr_points_ - 1)/2)].pose.position.y;
      }
  	}
}
void BezierCurve::setActiveT(float t)
{
  active_t_ = t;
}

// Object using methods.
// Find and set the nearest t;
void BezierCurve::findT()
{
  // Iterate trough all points in a resolution of 0.1.
  // Iterate trough all points in a resolution of 0.01.
}
void BezierCurve::calcCurvature()
{
  this->BezierCurve::findT();
  *this.active_t_ = *this.nearest_t_;
  this->calcCurvature(*this.active_t_);
}
void BezierCurve::calcCurvature(float t)
{
  float curvature;
  x_dot = *this.x_dot_;
  x_dot_dot = *this.x_dot_dot_;
  y_dot = *this.y_dot_;
  y_dot_dot = *this.y_dot_dot_;
  this->BezierCurve::calcXyDot(t);
  this->BezierCurve::calcXyDotDot(t);
  curvature = (x_dot*y_dot_dot - x_dot_dot*y_dot)/pow(pow(x_dot,2) + pow(y_dot,2),1.5);
  *this.curvature_ = curvature;
}
void BezierCurve::calcXyDot(float t)
{
  float x_dot = 0.0;
  float y_dot = 0.0;
  int n = *this.numb_ctr_points_;

  for(int i = 0; i < n + 1; i++)
  {
    float dot_factor = i*pow(t, i - 1)*pow(1.0 - t, n - i) + pow(t, i)*(i - n)*pow(1.0 - t, n - i - 1) ;
    x_dot = x_dot + x_path_[i]*BezierCurve::binomial(numb_ctr_points_ - 1, i)*dot_factor;
    y_dot = y_dot + y_path_[i]*BezierCurve::binomial(numb_ctr_points_ - 1, i)*dot_factor;
  }

  x_dot_ = x_dot;
  y_dot_ = y_dot;
}
void BezierCurve::calcXyDotDot(float t)
{
  float x_dot_dot = 0.0;
  float y_dot_dot = 0.0;
  int n = *this.numb_ctr_points_;

  for(int i = 0; i < n + 1; i++)
  {
    float dot_dot_factor = i*(i - 1)*pow(t, i - 2)*pow(1.0 - t, n - i)
    + i*pow(t, i - 1)*(i - n)*pow(n - i - 1)
    + i*pow(t, i - 1)*(i - n)*pow(1.0 - t, n - i - 1)
    + pow(t, i)*(n - i)*(n - i - 1)*(-1.0)*pow(1.0 - t, n - i - 2);
    x_dot_dot = x_dot_dot + x_path_[i]*BezierCurve::binomial(numb_ctr_points_ - 1, i)*dot_dot_factor;
    y_dot_dot = y_dot_dot + y_path_[i]*BezierCurve::binomial(numb_ctr_points_ - 1, i)*dot_dot_factor;
  }

  x_dot_dot_ = x_dot_dot;
  y_dot_dot_ = y_dot_dot;
}
float BezierCurve::getCurvature
{
  return *this.curvature_;
}

// Helper methods.
// n-factorial ==> n!. Is a static method.
int BezierCurve::factorial(int n)
{
  int n_fac = 1;
  if(n == 0)
  {
    n_fac = 1;
  }
  else
  {
    n_fac = BezierCurve::factorial(n-1);
  }
  return n_fac;
}
// Calculates binomial-coefficient a lower b. Is a static method.
int BezierCurve::binomial(int a, int b)
{
  int binom = BezierCurve::factorial(a)/(BezierCurve::factorial(b)*BezierCurve::factorial(a - b));
  return binom;
}
