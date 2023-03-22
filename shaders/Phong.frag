#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  //out_color.a = 1;
  vec3 n = vec3(v_normal);
  vec3 pos = vec3(v_position);
  vec3 len = u_light_pos - pos;
  float r = sqrt(pow(len.x, 2) + pow(len.y, 2) + pow(len.z, 2));
  float maxVal = max(0, dot(n, len / r));
  float radiusSq = r * r;
  vec3 total_L = vec3(0.0);
  // diffuse 
  float diffuseCoefficient = 1.0;
  vec3 L_diffuse = diffuseCoefficient * (u_light_intensity / radiusSq) * maxVal;
  total_L += L_diffuse;

  // ambient lighting
  float ka = 1.0;
  vec3 Ia = vec3(0.23, 0.23, 0.23);
  vec3 ambient = ka * Ia;
  total_L += ambient;

  // specular
  vec3 v = u_cam_pos - pos;
  vec3 h = v + len;
  float h_norm = sqrt(pow(h.x, 2) + pow(h.y, 2) + pow(h.z, 2));
  float ks = 0.4;
  float temp = max(0, dot(n, h / h_norm));
  float p = 50.0;
  vec3 specular = ks * (u_light_intensity / radiusSq) * pow(temp, p);
  total_L += specular;

  // final
   out_color = vec4(total_L.xyz, 1);
  out_color.a = 1;


}

