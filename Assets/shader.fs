#version 420

in vec3 v_WorldNormal;

struct LightProperties
{
    vec4 lightColor;
	vec3 lightDir;
	float diffuseIntensity;
	vec3 worldNormal;
	
};

float getDiffuseFactor(vec3 toLight, vec3 normal)
{
	float d = dot(toLight, normal);
	return clamp(d, 0.0, 1.0);
}

float calculateDirectionalLight(LightProperties lp)
{
  return lp.diffuseIntensity * getDiffuseFactor(-normalize(lp.lightDir), normalize(lp.worldNormal));
}

void main()                                                                                 
{                                                                                           
   LightProperties lp;
   lp.lightColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
   lp.lightDir   = vec3(-50.0f, -600.0f, -950.0f);
   lp.worldNormal = v_WorldNormal;
   lp.diffuseIntensity = 0.3f;                                                                                     
   
   gl_FragColor = lp.lightColor * calculateDirectionalLight(lp);
}
