#version 410

// first output is mapped to the framebuffer's colour index by default
out vec4 FragmentColour;

#ifdef NUM_SAMPLES
uniform sampler2DMS opaqueTexture;
uniform sampler2DMS translucentTexture;
#else
uniform sampler2D opaqueTexture;
uniform sampler2D translucentTexture;
#endif

in vec2 FragmentTexCoord;

void main()
{

	ivec2 texCoord = ivec2(int(FragmentTexCoord.x), int(FragmentTexCoord.y));
	vec4 opaqueColour = vec4(0.0);
	vec4 translucentColour = vec4(0.0);
	#ifdef NUM_SAMPLES
	for(int i=0; i<NUM_SAMPLES; i++){
		opaqueColour += texelFetch(opaqueTexture, texCoord, i);
	}
	opaqueColour /= NUM_SAMPLES;
	for(int i=0; i<NUM_SAMPLES; i++){
		translucentColour += texelFetch(translucentTexture, texCoord, i);
	}
	translucentColour /= NUM_SAMPLES;
	#else
	opaqueColour = texelFetch(opaqueTexture, texCoord);
	translucentColour = texelFetch(translucentTexture, texCoord);
	#endif

	float alpha = min(length(translucentColour), 1);
	FragmentColour = (1.0-alpha)*opaqueColour + alpha*translucentColour; 
}
