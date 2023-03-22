#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	Vector3D direction = pm.position - origin;
	if (direction.norm() <= radius) { // the point mass intersects with or is inside the sphere
		// "bump" it up to the surface of the sphere
		Vector3D tan_point = direction.unit() * radius + origin;
		Vector3D cor_vector = tan_point - pm.last_position;
		pm.position = pm.last_position + cor_vector * (1.0 - friction);
	}

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
