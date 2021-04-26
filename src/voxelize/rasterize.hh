#pragma once

#include "barycentric.hh"

#include <openvdb/math/Vec2.h>
#include <openvdb/math/Vec3.h>

#include <openvdb/Types.h>

namespace flywave {
namespace voxelize {

class Edge {
public:
  int X1, Y1, X2, Y2;

  Edge(int x1, int y1, int x2, int y2) {
    if (y1 < y2) {
      X1 = x1;
      Y1 = y1;
      X2 = x2;
      Y2 = y2;
    } else {
      X1 = x2;
      Y1 = y2;
      X2 = x1;
      Y2 = y1;
    }
  }
};

class Span {
public:
  int X1, X2;

  Span(int x1, int x2) {
    if (x1 < x2) {
      X1 = x1;
      X2 = x2;
    } else {
      X1 = x2;
      X2 = x1;
    }
  }
};

class rasterizer {
protected:
  inline void draw_span(const Span &span, int y);
  inline void draw_spans_between_edges(const Edge &e1, const Edge &e2);

public:
  std::vector<openvdb::OPENVDB_VERSION_NAME::math::Vec2<unsigned int>>
  rasterize_triangle(float x1, float y1, float x2, float y2, float x3,
                     float y3) {
    draw_triangle(x1, y1, x2, y2, x3, y3);
    return std::move(result);
  }

  std::vector<openvdb::OPENVDB_VERSION_NAME::math::Vec2<unsigned int>>
  rasterize_line(float x1, float y1, float x2, float y2) {
    draw_line(x1, y1, x2, y2);
    return std::move(result);
  }

private:
  inline void draw_triangle(float x1, float y1, float x2, float y2, float x3,
                            float y3);

  inline void set_pixel(unsigned int x, unsigned int y);
  inline void set_pixel(int x, int y);
  inline void set_pixel(float x, float y);

  inline void draw_line(float x1, float y1, float x2, float y2);

  std::vector<openvdb::OPENVDB_VERSION_NAME::math::Vec2<unsigned int>> result;
};

void rasterizer::set_pixel(unsigned int x, unsigned int y) {
  result.emplace_back(x, y);
}

void rasterizer::set_pixel(int x, int y) {
  set_pixel((unsigned int)x, (unsigned int)y);
}

void rasterizer::set_pixel(float x, float y) {
  if (x < 0.0f || y < 0.0f)
    return;

  set_pixel((unsigned int)x, (unsigned int)y);
}

void rasterizer::draw_span(const Span &span, int y) {
  int xdiff = span.X2 - span.X1;
  if (xdiff == 0)
    return;

  float factor = 0.0f;
  float factorStep = 1.0f / (float)xdiff;

  for (int x = span.X1; x < span.X2; x++) {
    set_pixel(x, y);
    factor += factorStep;
  }
}

void rasterizer::draw_spans_between_edges(const Edge &e1, const Edge &e2) {
  float e1ydiff = (float)(e1.Y2 - e1.Y1);
  if (e1ydiff == 0.0f)
    return;

  float e2ydiff = (float)(e2.Y2 - e2.Y1);
  if (e2ydiff == 0.0f)
    return;

  float e1xdiff = (float)(e1.X2 - e1.X1);
  float e2xdiff = (float)(e2.X2 - e2.X1);

  float factor1 = (float)(e2.Y1 - e1.Y1) / e1ydiff;
  float factorStep1 = 1.0f / e1ydiff;
  float factor2 = 0.0f;
  float factorStep2 = 1.0f / e2ydiff;

  for (int y = e2.Y1; y < e2.Y2; y++) {
    Span span(e1.X1 + (int)(e1xdiff * factor1),
              e2.X1 + (int)(e2xdiff * factor2));
    draw_span(span, y);

    factor1 += factorStep1;
    factor2 += factorStep2;
  }
}

void rasterizer::draw_triangle(float x1, float y1, float x2, float y2, float x3,
                               float y3) {
  Edge edges[3] = {Edge((int)x1, (int)y1, (int)x2, (int)y2),
                   Edge((int)x2, (int)y2, (int)x3, (int)y3),
                   Edge((int)x3, (int)y3, (int)x1, (int)y1)};

  int maxLength = 0;
  int longEdge = 0;

  for (int i = 0; i < 3; i++) {
    int length = edges[i].Y2 - edges[i].Y1;
    if (length > maxLength) {
      maxLength = length;
      longEdge = i;
    }
  }

  int shortEdge1 = (longEdge + 1) % 3;
  int shortEdge2 = (longEdge + 2) % 3;

  draw_spans_between_edges(edges[longEdge], edges[shortEdge1]);
  draw_spans_between_edges(edges[longEdge], edges[shortEdge2]);
}

void rasterizer::draw_line(float x1, float y1, float x2, float y2) {
  float xdiff = (x2 - x1);
  float ydiff = (y2 - y1);

  if (xdiff == 0.0f && ydiff == 0.0f) {
    set_pixel(x1, y1);
    return;
  }

  if (fabs(xdiff) > fabs(ydiff)) {
    float xmin, xmax;

    if (x1 < x2) {
      xmin = x1;
      xmax = x2;
    } else {
      xmin = x2;
      xmax = x1;
    }

    float slope = ydiff / xdiff;
    for (float x = xmin; x <= xmax; x += 1.0f) {
      float y = y1 + ((x - x1) * slope);
      set_pixel(x, y);
    }
  } else {
    float ymin, ymax;

    if (y1 < y2) {
      ymin = y1;
      ymax = y2;
    } else {
      ymin = y2;
      ymax = y1;
    }

    float slope = xdiff / ydiff;
    for (float y = ymin; y <= ymax; y += 1.0f) {
      float x = x1 + ((y - y1) * slope);
      set_pixel(x, y);
    }
  }
}

} // namespace voxelize
} // namespace flywave
