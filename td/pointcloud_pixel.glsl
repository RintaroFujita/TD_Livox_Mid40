// Livox point cloud - GLSL MAT pixel shader (TouchDesigner)
// Pair with pointcloud_vertex.glsl.

in Vertex {
	vec4 color;
} iVert;

layout(location = 0) out vec4 oFragColor[TD_NUM_COLOR_BUFFERS];

void main()
{
	TDCheckDiscard();
	vec4 outcol = iVert.color;
	TDAlphaTest(outcol.a);
	oFragColor[0] = TDOutputSwizzle(outcol);
	for (int i = 1; i < TD_NUM_COLOR_BUFFERS; i++)
		oFragColor[i] = vec4(0.0);
}
