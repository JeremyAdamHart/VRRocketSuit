#version 410

// location indices for these attributes correspond to those specified in the
// InitializeGeometry() function of the main program
layout(location = 0) in vec3 VertexPosition;
layout(location = 2) in vec2 VertexTexCoord;

out vec2 FragmentTexCoord;

void main()
{
    // assign vertex position without modification
    FragmentTexCoord = VertexTexCoord;
    gl_Position = vec4(VertexPosition, 1.0);
}
