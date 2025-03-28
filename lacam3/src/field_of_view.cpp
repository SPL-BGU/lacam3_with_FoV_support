#include "../include/field_of_view.hpp"

Vertices get_field_of_view(Graph *g, Vertex *node, int field_radius)
{
  Vertices field_of_view;
  int min_x = std::max(0, node->x - field_radius);
  int min_y = std::max(0, node->y - field_radius);
  // plus 1 since we want to include the field of view inside the for loop:
  int max_x = std::min(g->width, node->x + field_radius + 1);
  int max_y = std::min(g->height, node->y + field_radius + 1);
  for (int x = min_x; x < max_x; x++) {
    for (int y = min_y; y < max_y; y++) {
      // check if position in map (no obstacles):
      int index = g->width * y + x;
      if (nullptr != g->U[index]) {
        field_of_view.push_back(g->U[index]);
      }
    }
  }
  return field_of_view;
}

bool in_field_of_view(Vertex *n1, Vertex *n2, int field_radius)
{
  int x_diff = std::abs(n1->x - n2->x);
  int y_diff = std::abs(n1->y - n2->y);
  return x_diff <= field_radius && y_diff <= field_radius;
}
