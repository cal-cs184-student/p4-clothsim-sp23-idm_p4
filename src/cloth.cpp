#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
    
  // TODO (Part 1): Build a grid of masses and springs.
    for (int y = 0; y < num_height_points; y++) {
        for (int x = 0; x < num_width_points; x++) {
            double p = double(x) * (width / num_width_points);
            double q = double(y) * (height / num_height_points);
            Vector3D position;
            if (orientation == 0) {// 0 means horizontal
                position = Vector3D(p, 1.0, q);
            }
            else { // vertical
                double f = (double)rand() / RAND_MAX;
                double z = -1.0 / 1000 + f * (1.0 / 1000 + 1.0 / 1000);
                position = Vector3D(p, q, z);
            }
            bool whether_pin = false;
            for (int i = 0; i < pinned.size(); i++) {
                if (pinned[i][0] == x && pinned[i][1] == y) {
                    whether_pin = true;
                }
            }
            PointMass cur_point(position, whether_pin);
            // finished storing point_masses back to cloth
            point_masses.emplace_back(cur_point);
        }
    }
    for (int y = 0; y < num_height_points; y++) {
        for (int x = 0; x < num_width_points; x++) {
            //Left of pm
            if (x > 0) {
                PointMass* a = &point_masses[y * num_width_points + x - 1];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, STRUCTURAL);
                springs.emplace_back(s);
            }
            //Above
            if (y > 0) {
                PointMass* a = &point_masses[(y - 1) * num_width_points + x];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, STRUCTURAL);
                springs.emplace_back(s);
            }
            //Upper Left
            if (x > 0 && y > 0) {
                PointMass* a = &point_masses[(y - 1) * num_width_points + x - 1];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, SHEARING);
                springs.emplace_back(s);
            }
            //upper Right
            if (x + 1 < num_width_points && y > 0) {
                PointMass* a = &point_masses[(y - 1) * num_width_points + x + 1];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, SHEARING);
                springs.emplace_back(s);
            }
            //two above
            if ((y - 2) >= 0) {
                PointMass* a = &point_masses[(y - 2) * num_width_points + x];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, BENDING);
                springs.emplace_back(s);
            }
            //Two left from point mass
            if ((x - 2) >= 0) {
                PointMass* a = &point_masses[y * num_width_points + x - 2];
                PointMass* b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, BENDING);
                springs.emplace_back(s);
            }
        }
    }
    
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D a(0, 0, 0);
  for (int i = 0; i < external_accelerations.size(); i++) {
      // compute total acceleration
      a += external_accelerations[i];
  }

  for (int i = 0; i < point_masses.size(); i++) {
      // F = m * a and store it into point_masses
      point_masses[i].forces = mass * a;
  }
  // STRUCTURAL = 0, SHEARING = 1, BENDING = 2
  for (Spring& s : springs) {
      if ((cp->enable_structural_constraints && s.spring_type == 0) ||
          (cp->enable_shearing_constraints && s.spring_type == 1) ||
          (cp->enable_bending_constraints && s.spring_type == 2)) {
          Vector3D pab = s.pm_a->position - s.pm_b->position;
          double Fs = cp->ks * (pab.norm() - s.rest_length);
          if (s.spring_type == 2) { // the bending constraint should be weaker
              Fs *= 0.2;
          }
          Vector3D final_force = pab.unit() * Fs;
          // one add and one minus
          s.pm_a->forces -= final_force;
          s.pm_b->forces += final_force;
      }
  }
  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  for (PointMass& p : point_masses) {
      if (!p.pinned) {// if point is not pinned
          Vector3D xt_dt = p.last_position;
          Vector3D a = p.forces / mass;
          Vector3D xt = p.position;
          Vector3D new_position = xt + (1.0 - (cp->damping / 100)) * (xt - xt_dt) + a * delta_t * delta_t;
          p.last_position = xt;
          p.position = new_position;
      }
  }
  // TODO (Part 4): Handle self-collisions.


  // TODO (Part 3): Handle collisions with other primitives.


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring& s : springs) {
      Vector3D p_a = s.pm_a->position;
      Vector3D p_b = s.pm_b->position;
      double length = (p_a - p_b).norm() - s.rest_length * 1.1; // the spring's length is at most 10% greater than its rest_length
      if (length > 0) {
          Vector3D offset = (p_a - p_b).unit() * length;
          if (!s.pm_a->pinned && !s.pm_b->pinned) {
              s.pm_a->position -= 0.5 * offset;
              s.pm_b->position += 0.5 * offset;
          }
          else if (!s.pm_a->pinned) {
              s.pm_a->position -= offset;
          }
          else if (!s.pm_b->pinned) {
              s.pm_b->position += offset;
          }
          else {
              continue;
          }
      }
  }
  
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
