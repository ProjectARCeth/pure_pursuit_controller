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
void BezierCurve:.setCurrentArrayIndex(int curr_index)
{
    current_arrayposition_ = curr_index;
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

  for(int i = 0; )
  {
    x_dot = x_dot + ;
    y_dot = y_dot + ;
  }
}
void BezierCurve::calcXyDotDot(float t)
{
  float x_dot = 0.0;
  float y_dot = 0.0;

  for(int i = 0; )
  {
    x_dot = x_dot + ;
    y_dot = y_dot + ;
  }
}
float BezierCurve::getCurvature
{
  return *this.curvature_;
}
