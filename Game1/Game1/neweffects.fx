//------------------------------------------------------
//--                                                  --
//--		   www.riemers.net                    --
//--   		    Basic shaders                     --
//--		Use/modify as you like                --
//--                                                  --
//------------------------------------------------------

struct VertexToPixel
{
    float4 Position   	: POSITION;    
    float4 Color		: COLOR0;
    float LightingFactor: TEXCOORD0;
    float2 TextureCoords: TEXCOORD1;
};

struct PixelToFrame
{
    float4 Color : COLOR0;
};

//------- Constants --------
float4x4 xView;
float4x4 xProjection;
float4x4 xWorld;
float3 xLightDirection;
float xAmbient;
float xDiffuse;
bool xEnableLighting;

float4 xColor;

bool Clipping;
float4 ClipPlane0;

float xWaveLength;
float xWaveHeight;

float4x4 xReflectionView;

float3 xCamPos;
float3 xCamUp;
float xPointSpriteSize;

float xTime;
float3 xWindDirection;
float xWindForce;
//------- Texture Samplers --------

Texture xTexture;
sampler TextureSampler = sampler_state {
	texture = <xTexture>;
	magfilter = LINEAR;
	minfilter = LINEAR;
	mipfilter=LINEAR;
	AddressU = mirror;
	AddressV = mirror;
};


//------- Texture Samplers --------
Texture xTexture0;
sampler TextureSampler0 = sampler_state {
	texture = <xTexture0>; 
	magfilter = LINEAR; 
	minfilter = LINEAR; 
	mipfilter = LINEAR; 
	AddressU = wrap; 
	AddressV = wrap; 
}; 

Texture xTexture1;
sampler TextureSampler1 = sampler_state { 
	texture = <xTexture1>; 
	magfilter = LINEAR; 
	minfilter = LINEAR; 
	mipfilter = LINEAR; 
	AddressU = wrap; 
	AddressV = wrap; 
}; 

Texture xTexture2;
sampler TextureSampler2 = sampler_state { 
	texture = <xTexture2>; 
	magfilter = LINEAR; 
	minfilter = LINEAR; 
	mipfilter = LINEAR; 
	AddressU = mirror; 
	AddressV = mirror; 
}; 

Texture xTexture3;
sampler TextureSampler3 = sampler_state { 
	texture = <xTexture3>; 
	magfilter = LINEAR; 
	minfilter = LINEAR; 
	mipfilter = LINEAR; 
	AddressU = mirror; 
	AddressV = mirror; 
};

Texture xReflectionMap;
sampler ReflectionSampler = sampler_state {
	texture = <xReflectionMap>;
	magfilter = LINEAR;
	minfilter = LINEAR;
	mipfilter = LINEAR;
	AddressU = mirror;
	AddressV = mirror;
};

Texture xRefractionMap;
sampler RefractionSampler = sampler_state {
	texture = <xRefractionMap>;
	magfilter = LINEAR;
	minfilter = LINEAR;
	mipfilter = LINEAR;
	AddressU = mirror;
	AddressV = mirror;
};

Texture xWaterBumpMap;
sampler WaterBumpMapSampler = sampler_state {
	texture = <xWaterBumpMap>;
	magfilter = LINEAR;
	minfilter = LINEAR;
	mipfilter = LINEAR;
	AddressU = mirror;
	AddressV = mirror;
};


struct MTVertexToPixel
{
	float4 Position         : POSITION;
	float4 Color            : COLOR0;
	float3 Normal            : TEXCOORD0;
	float2 TextureCoords    : TEXCOORD1;
	float4 LightDirection    : TEXCOORD2;
	float4 TextureWeights    : TEXCOORD3;

	float Depth				: TEXCOORD4;

	float4 clipDistances     : TEXCOORD5;
};

struct MTPixelToFrame
{
	float4 Color : COLOR0;
};

MTVertexToPixel MultiTexturedVS(float4 inPos : POSITION, float3 inNormal : NORMAL, float2 inTexCoords : TEXCOORD0, float4 inTexWeights : TEXCOORD1)
{
	MTVertexToPixel Output = (MTVertexToPixel)0;
	float4x4 preViewProjection = mul(xView, xProjection);
		float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);

		Output.Position = mul(inPos, preWorldViewProjection);
	Output.Normal = mul(normalize(inNormal), xWorld);
	Output.TextureCoords = inTexCoords;
	Output.LightDirection.xyz = -xLightDirection;
	Output.LightDirection.w = 1;
	Output.TextureWeights = inTexWeights;

	Output.Depth = Output.Position.z / Output.Position.w;

	Output.clipDistances = dot(inPos, ClipPlane0);

	return Output;
}

MTPixelToFrame MultiTexturedPS(MTVertexToPixel PSIn)
{
	MTPixelToFrame Output = (MTPixelToFrame)0;

	float lightingFactor = 1;
	if (xEnableLighting)
		lightingFactor = saturate(saturate(dot(PSIn.Normal, PSIn.LightDirection)) + xAmbient);


	float blendDistance = 0.85f;
	float blendWidth = 0.2f;
	float blendFactor = clamp((PSIn.Depth - blendDistance) / blendWidth, 0, 1);

	float4 farColor;
	farColor = tex2D(TextureSampler0, PSIn.TextureCoords)*PSIn.TextureWeights.x;
	farColor += tex2D(TextureSampler1, PSIn.TextureCoords)*PSIn.TextureWeights.y;
	farColor += tex2D(TextureSampler2, PSIn.TextureCoords)*PSIn.TextureWeights.z;
	farColor += tex2D(TextureSampler3, PSIn.TextureCoords)*PSIn.TextureWeights.w;

	float4 nearColor;
	float2 nearTextureCoords = PSIn.TextureCoords * 30;
	nearColor = tex2D(TextureSampler0, nearTextureCoords)*PSIn.TextureWeights.x;
	nearColor += tex2D(TextureSampler1, nearTextureCoords)*PSIn.TextureWeights.y;
	nearColor += tex2D(TextureSampler2, nearTextureCoords)*PSIn.TextureWeights.z;
	nearColor += tex2D(TextureSampler3, nearTextureCoords)*PSIn.TextureWeights.w;

	Output.Color = lerp(nearColor, farColor, blendFactor);
	Output.Color *= lightingFactor;


	if (Clipping)
		clip(PSIn.clipDistances);

	return Output;
}

technique MultiTextured
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 MultiTexturedVS();
		PixelShader = compile ps_3_0 MultiTexturedPS();
	}
}











//------- Technique: Textured --------
struct TVertexToPixel
{
	float4 Position     : POSITION;
	float4 Color        : COLOR0;
	float LightingFactor : TEXCOORD0;
	float2 TextureCoords: TEXCOORD1;
};

struct TPixelToFrame
{
	float4 Color : COLOR0;
};

TVertexToPixel TexturedVS(float4 inPos : POSITION, float3 inNormal : NORMAL, float2 inTexCoords : TEXCOORD0)
{
	TVertexToPixel Output = (TVertexToPixel)0;
	float4x4 preViewProjection = mul(xView, xProjection);
		float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);

		Output.Position = mul(inPos, preWorldViewProjection);
	Output.TextureCoords = inTexCoords;

	float3 Normal = normalize(mul(normalize(inNormal), xWorld));
		Output.LightingFactor = 1;
	if (xEnableLighting)
		Output.LightingFactor = saturate(dot(Normal, -xLightDirection));

	return Output;
}

TPixelToFrame TexturedPS(TVertexToPixel PSIn)
{
	TPixelToFrame Output = (TPixelToFrame)0;

	Output.Color = tex2D(TextureSampler, PSIn.TextureCoords);
	Output.Color.rgb *= saturate(PSIn.LightingFactor + xAmbient);

	return Output;
}

technique Textured_2_0
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 TexturedVS();
		PixelShader = compile ps_3_0 TexturedPS();
	}
}

technique Textured
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 TexturedVS();
		PixelShader = compile ps_3_0 TexturedPS();
	}
}

//------- Technique: Colored --------

VertexToPixel ColoredVS(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
	VertexToPixel Output = (VertexToPixel)0;
	float4x4 preViewProjection = mul(xView, xProjection);
		float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);

		Output.Position = mul(inPos, preWorldViewProjection);
	Output.Color = inColor;

	float3 Normal = normalize(mul(normalize(inNormal), xWorld));
		Output.LightingFactor = 1;
	if (xEnableLighting)
		Output.LightingFactor = dot(Normal, -xLightDirection);

	return Output;
}

PixelToFrame ColoredPS(VertexToPixel PSIn)
{
	PixelToFrame Output = (PixelToFrame)0;

	Output.Color = PSIn.Color;
	Output.Color.rgb *= saturate(PSIn.LightingFactor) + xAmbient;

	return Output;
}

technique Colored
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 ColoredVS();
		PixelShader = compile ps_3_0 ColoredPS();
	}
}


//------- Technique: Monochromatic --------

VertexToPixel MonochromaticVS(float4 inPos : POSITION, float3 inNormal : NORMAL)
{
	VertexToPixel Output = (VertexToPixel)0;
	float4x4 preViewProjection = mul(xView, xProjection);
	float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);

	Output.Position = mul(inPos, preWorldViewProjection);
	Output.Color = xColor;

	//float3 Normal = normalize(mul(normalize(inNormal), xWorld));
	float3 Normal = normalize(inNormal);
		Output.LightingFactor = 1;
	if (xEnableLighting)
		Output.LightingFactor = xDiffuse * saturate(dot(Normal, -xLightDirection));

	return Output;
}

PixelToFrame MonochromaticPS(VertexToPixel PSIn)
{
	PixelToFrame Output = (PixelToFrame)0;

	Output.Color = PSIn.Color;
	Output.Color.rgb *= saturate(PSIn.LightingFactor) + xAmbient;

	return Output;
}

technique Monochromatic
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 MonochromaticVS();
		PixelShader = compile ps_3_0 MonochromaticPS();
	}
}






//------- Technique: Water --------
struct WVertexToPixel
{
	float4 Position						: POSITION;
	float4 ReflectionMapSamplingPos		: TEXCOORD1;
	float2 BumpMapSamplingPos			: TEXCOORD2;
	float4 RefractionMapSamplingPos		: TEXCOORD3;
	float4 Position3D					: TEXCOORD4;
};

struct WPixelToFrame
{
	float4 Color : COLOR0;
};

WVertexToPixel WaterVS(float4 inPos : POSITION, float2 inTex : TEXCOORD)
{
	WVertexToPixel Output = (WVertexToPixel)0;

	float4x4 preViewProjection = mul(xView, xProjection);
	float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);
	float4x4 preReflectionViewProjection = mul(xReflectionView, xProjection);
	float4x4 preWorldReflectionViewProjection = mul(xWorld, preReflectionViewProjection);

	float3 windDir = normalize(xWindDirection);
	float3 perpDir = cross(xWindDirection, float3(0, 1, 0));
	float yDot = dot(inTex, xWindDirection.xz);
	float xDot = dot(inTex, perpDir.xz);
	float2 moveVector = float2(xDot, yDot);
	moveVector.y += xTime * xWindForce;



	Output.Position = mul(inPos, preWorldViewProjection);
	Output.ReflectionMapSamplingPos = mul(inPos, preWorldReflectionViewProjection);
	Output.BumpMapSamplingPos = inTex / xWaveLength;

	Output.RefractionMapSamplingPos = mul(inPos, preWorldViewProjection);
	Output.Position3D = mul(inPos, xWorld);

	Output.BumpMapSamplingPos = moveVector / xWaveLength;

	return Output;
}

WPixelToFrame WaterPS(WVertexToPixel PSIn)
{
	WPixelToFrame Output = (WPixelToFrame)0;

	float2 ProjectedTexCoords;
	ProjectedTexCoords.x = PSIn.ReflectionMapSamplingPos.x / PSIn.ReflectionMapSamplingPos.w / 2.0f + 0.5f;
	ProjectedTexCoords.y = -PSIn.ReflectionMapSamplingPos.y / PSIn.ReflectionMapSamplingPos.w / 2.0f + 0.5f;

	float4 bumpColor = tex2D(WaterBumpMapSampler, PSIn.BumpMapSamplingPos);
	float2 perturbation = xWaveHeight * (bumpColor.rg - 0.5f) * 2.0f;
	float2 perturbatedTexCoords = ProjectedTexCoords + perturbation;
	float4 reflectiveColor = tex2D(ReflectionSampler, perturbatedTexCoords);
	
	
	float2 ProjectedRefrTexCoords;
	ProjectedRefrTexCoords.x = PSIn.RefractionMapSamplingPos.x / PSIn.RefractionMapSamplingPos.w / 2.0f + 0.5f;
	ProjectedRefrTexCoords.y = -PSIn.RefractionMapSamplingPos.y / PSIn.RefractionMapSamplingPos.w / 2.0f + 0.5f;
	float2 perturbatedRefrTexCoords = ProjectedRefrTexCoords + perturbation;
	float4 refractiveColor = tex2D(RefractionSampler, perturbatedRefrTexCoords);

	float3 eyeVector = normalize(xCamPos - PSIn.Position3D);
	float3 normalVector = float3(0, 1, 0);
	float fresnelTerm = dot(eyeVector, normalVector);
	float4 combinedColor = lerp(reflectiveColor, refractiveColor, fresnelTerm);

	float4 dullColor = float4(0.3f, 0.3f, 0.5f, 1.0f);

	Output.Color = lerp(combinedColor, dullColor, 0.2f);

	return Output;
}

technique Water
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 WaterVS();
		PixelShader = compile ps_3_0 WaterPS();
	}
}



//------- Technique: SimpleTextured --------
struct STVertexToPixel
{
	float4 Position     : POSITION;
	float2 TextureCoords: TEXCOORD0;
};

struct STPixelToFrame
{
	float4 Color : COLOR0;
};

STVertexToPixel STexturedVS(float4 inPos : POSITION, float2 inTexCoords : TEXCOORD)
{
	STVertexToPixel Output = (STVertexToPixel)0;
	float4x4 preViewProjection = mul(xView, xProjection);
	float4x4 preWorldViewProjection = mul(xWorld, preViewProjection);

	Output.Position = mul(inPos, preWorldViewProjection);
	Output.TextureCoords = inTexCoords;

	return Output;
}

STPixelToFrame STexturedPS(STVertexToPixel PSIn)
{
	STPixelToFrame Output = (STPixelToFrame)0;

	Output.Color = tex2D(TextureSampler, PSIn.TextureCoords);

	return Output;
}

technique SimpleTextured
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 STexturedVS();
		PixelShader = compile ps_3_0 STexturedPS();
	}
}


//------- Technique: PointSprites --------

VertexToPixel PointSpriteVS(float3 inPos: POSITION0, float2 inTexCoord : TEXCOORD0)
{
	VertexToPixel Output = (VertexToPixel)0;

	float3 center = mul(inPos, xWorld);
		float3 eyeVector = center - xCamPos;

		float3 sideVector = cross(eyeVector, xCamUp);
		sideVector = normalize(sideVector);
	float3 upVector = cross(sideVector, eyeVector);
		upVector = normalize(upVector);

	float3 finalPosition = center;
		finalPosition += (inTexCoord.x - 0.5f)*sideVector*0.5f*xPointSpriteSize;
	finalPosition += (0.5f - inTexCoord.y)*upVector*0.5f*xPointSpriteSize;

	float4 finalPosition4 = float4(finalPosition, 1);

		float4x4 preViewProjection = mul(xView, xProjection);
		Output.Position = mul(finalPosition4, preViewProjection);

	Output.TextureCoords = inTexCoord;

	return Output;
}

PixelToFrame PointSpritePS(VertexToPixel PSIn) : COLOR0
{
	PixelToFrame Output = (PixelToFrame)0;
	Output.Color = tex2D(TextureSampler, PSIn.TextureCoords);
	return Output;
}

technique PointSprites
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 PointSpriteVS();
		PixelShader = compile ps_3_0 PointSpritePS();
	}
}