#pragma once
#include "math.h"
#include "nav_msgs/Path.h"
#include <iostream>

class BezierCurve{

  public:
    // Default constructor.
    BezierCurve();
    // Useful constructor for pure pursuit.
    BezierCurve(nav_msgs::Path path, int numb_ctr_points);
    // Destructor.
    ~BezierCurve();

    // Object changing methods.
    // Define the bezier-curve.
    // Sets the path.
    void setPath(nav_msgs::Path path);
    // Set the number of control points of the bezier-curve.
    void setNumbCtrlPoints(int numb_ctr_points);
    // Set the index of the point (in path_) around which the bezier-curve should be located.
    void setCenterIndex(int center_index);
    // Set the points which will be used as control points (-->control polygon).
    void setCtrlPoints();
    // Set the active t (where to evaluate Curve).
    void setActiveT(float t);

    // Object using methods.
    // Do stuff with the previously defined bezier-curve.
    // Find and set the nearest t;
    void findT();
    // For a defined bezier-curve, find the nearest t and the use calcCurvature(float t). Will be useful for pure-pursuit.
    void calcCurvature();
    // For a defined bezier-curve and a given parameter t, calculate and set the curvature.
    void calcCurvature(float t);
    // For a given bezier-curve and a given t, calculate the first derivatives. Will be mainly used in calcCurvature.
    void calcXyDot(float t);
    // At the current nearest_t_, calculate and set the second derivatives w.r.t to parameter t. Will be mainly used in calcCurvature.
    void calcXyDotDot(float t);
    // Output the interpolated curve value at parameter t.
    Eigen::Vector3d curveAtT(float t);
    // Output the calculated curvature at the active t.
    float getCurvature();

    // Helper methods.
    // n-factorial ==> n!.
    static int factorial(int n);
    // Calculates binomial-coefficient a lower b.
    static int binomial(int a, int b);

  private:
      // Variables.
      // The path.
      nav_msgs::Path path_;
      // How many points should be used for the bezier curve?.
      int numb_ctr_points_;
      // Path index around which the .
      int center_index_;
      // Dynamic Vector with the needed x-coordinates of path points.
      float* x_path_;
      // Dynamic Vector with the needed y-coordinates of path points.
      float* y_path_;
      // Using the current_arrayposition find the nearest parameter t and set this t to active t.
      float nearest_t_;
      // Parameter t at which the following variables will be calculated and stored.
      float active_t_;
      // The first derivative of x w.r.t t at the active_t_.
      float x_dot_;
      // The second derivative of x w.r.t t at the active_t_.
      float x_dot_dot_;
      // The first derivative of y w.r.t t at the active_t_.
      float y_dot_;
      // The second derivative of y w.r.t t at the active_t_.
      float y_dot_dot_;
      // The curvature at the active_t_.
      float curvature_;
}
