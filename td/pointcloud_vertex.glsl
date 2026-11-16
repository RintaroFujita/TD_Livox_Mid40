// Livox point cloud - GLSL MAT vertex shader (TouchDesigner)
//
// Geometry COMP setup:
//   Instancing: On, Instance OP: the Livox Script TOP (Mode = grid, Alpha = valid)
//   Translate X / Y / Z: r / g / b     (one instance per pixel = per point)
//   Geometry inside: Rectangle SOP, size 1 x 1, centre 0 (a camera-facing quad)
//
// GLSL MAT:
//   Vertex shader: this file, Pixel shader: pointcloud_pixel.glsl
//   Samplers page:  sPoints  -> the same Livox Script TOP
//   Vectors page:   uSize    -> point size in metres (e.g. 0.01)
//                   uRange   -> colour range near, far in metres (e.g. 1, 10)
//                   uMode    -> 0 = depth colour, 1 = reflectivity colour (needs Alpha = reflectivity), 2 = white

uniform sampler2D sPoints;
uniform float uSize;
uniform vec2 uRange;
uniform float uMode;

out Vertex {
	vec4 color;
} oVert;

// Google Turbo colormap (polynomial approximation)
vec3 turbo(float t)
{
	const vec4 kR4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
	const vec4 kG4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
	const vec4 kB4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
	const vec2 kR2 = vec2(-152.94239396, 59.28637943);
	const vec2 kG2 = vec2(4.27729857, 2.82956604);
	const vec2 kB2 = vec2(-89.90310912, 27.34824973);
	t = clamp(t, 0.0, 1.0);
	vec4 v4 = vec4(1.0, t, t * t, t * t * t);
	vec2 v2 = v4.zw * v4.z;
	return clamp(vec3(dot(v4, kR4) + dot(v2, kR2),
	                  dot(v4, kG4) + dot(v2, kG2),
	                  dot(v4, kB4) + dot(v2, kB2)), 0.0, 1.0);
}

void main()
{
	// The point this instance represents (same pixel the instancer used for translate).
	int id = TDInstanceID();
	ivec2 size = textureSize(sPoints, 0);
	vec4 pt = texelFetch(sPoints, ivec2(id % size.x, id / size.x), 0);

	if (pt.a <= 0.0) {
		// empty pixel: put the vertex outside the clip volume so nothing is drawn
		gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
		oVert.color = vec4(0.0);
		return;
	}

	// Instance centre in world space (translate comes from the instancer: r, g, b).
	vec4 worldCentre = TDDeform(vec4(0.0, 0.0, 0.0, 1.0));
	// Camera-facing quad: offset the corners in camera space.
	int cameraIndex = TDCameraIndex();
	vec4 camPos = uTDMats[cameraIndex].cam * worldCentre;
	camPos.xy += P.xy * uSize;
	gl_Position = uTDMats[cameraIndex].proj * camPos;

	float t;
	if (uMode < 0.5) {
		t = (length(pt.xyz) - uRange.x) / max(uRange.y - uRange.x, 1e-6);
		oVert.color = vec4(turbo(t), 1.0);
	} else if (uMode < 1.5) {
		oVert.color = vec4(turbo(pt.a), 1.0);
	} else {
		oVert.color = vec4(1.0);
	}
}
