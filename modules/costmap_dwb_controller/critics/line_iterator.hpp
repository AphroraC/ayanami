#pragma once

#include <cstdlib>

namespace dwb_critics {

/** An iterator implementing Bresenham Ray-Tracing. */
class LineIterator {
public:
  LineIterator(int x0, int y0, int x1, int y1)
  : x0(x0),
    y0(y0),
    x1(x1),
    y1(y1),
    x(x0),    // X and Y start of at first endpoint.
    y(y0),
    deltax(abs(x1 - x0)),
    deltay(abs(y1 - y0)),
    curpixel(0)
  {
    if (x1 >= x0) {                // The x-values are increasing
      xinc1 = 1;
      xinc2 = 1;
    } else {                      // The x-values are decreasing
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0) {               // The y-values are increasing
      yinc1 = 1;
      yinc2 = 1;
    } else {                      // The y-values are decreasing
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay) {        // There is at least one x-value for every y-value
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }
  }

  bool isValid() const
  {
    return curpixel <= numpixels;
  }

  void advance()
  {
    num += numadd;              // Increase the numerator by the top of the fraction
    if (num >= den) {           // Check if numerator >= denominator
      num -= den;               // Calculate the new numerator value
      x += xinc1;               // Change the x as appropriate
      y += yinc1;               // Change the y as appropriate
    }
    x += xinc2;                 // Change the x as appropriate
    y += yinc2;                 // Change the y as appropriate

    curpixel++;
  }

  int getX() const { return x; }
  int getY() const { return y; }
  int getX0() const { return x0; }
  int getY0() const { return y0; }
  int getX1() const { return x1; }
  int getY1() const { return y1; }

private:
  int x0;  ///< X coordinate of first end point.
  int y0;  ///< Y coordinate of first end point.
  int x1;  ///< X coordinate of second end point.
  int y1;  ///< Y coordinate of second end point.

  int x;  ///< X coordinate of current point.
  int y;  ///< Y coordinate of current point.

  int deltax;  ///< Difference between Xs of endpoints.
  int deltay;  ///< Difference between Ys of endpoints.

  int curpixel;  ///< index of current point in line loop.

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;
};

}  // end namespace dwb_critics
