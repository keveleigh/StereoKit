// SPDX-License-Identifier: MIT
// The authors below grant copyright rights under the MIT license:
// Copyright (c) 2026 Nick Klingensmith

using StereoKit;

class DemoPostFX : ITest
{
	string title       = "Post Processing";
	string description = "Post-process effects are Materials whose shaders read the scene through a pixel-local input attachment - on mobile GPUs the whole chain stays in tile memory, so these are safe for XR framerates!\n\nEffects added with Renderer.AddPostProcess apply to the main display and screenshots, ordered by Material.QueueOffset. Up to 2 can be active at a time.";

	Pose windowPose = Demo.contentPose.Pose;

	Material vignetteMat;
	Material invertMat;
	Material fogMat;
	bool     vignetteOn;
	bool     invertOn;
	bool     fogOn;
	float    vignetteStrength = 0.4f;
	float    fogDensity       = 0.5f;

	Model model;

	public void Initialize()
	{
		/// :CodeSample: Renderer.AddPostProcess Renderer.RemovePostProcess
		/// ### Adding a post-process effect
		/// A post-process effect is just a Material! Its shader reads the
		/// scene through an input attachment named 'color', and its
		/// parameters can be changed live, like any other Material.
		Material vignette = new Material("postfx_vignette.hlsl");
		vignette["strength"] = 0.4f;
		Renderer.AddPostProcess(vignette);
		///
		/// And when you're done with it:
		Renderer.RemovePostProcess(vignette);
		/// :End:
		vignetteMat = vignette;

		invertMat = new Material("postfx_math.hlsl");
		invertMat["mul_color"] = new Vec3(-1, -1, -1);
		invertMat["add_color"] = new Vec3( 1,  1,  1);
		// Queue offsets order the chain - invert first, vignette after, so
		// the vignette darkens the inverted image's corners.
		invertMat  .QueueOffset = 0;
		vignetteMat.QueueOffset = 10;

		fogMat = new Material("postfx_depth_fog.hlsl");
		fogMat["fog_density"] = fogDensity;
		fogMat.QueueOffset = -10; // Fog goes before color effects

		/// :CodeSample: RenderSettings Renderer.RenderTo
		/// ### Rendering a viewpoint with post-processing
		/// RenderSettings works with Renderer.RenderTo and RenderList.DrawNow,
		/// and can carry a post-process chain that applies to just that pass!
		Tex target = Tex.RenderTarget(512, 512);
		Renderer.RenderTo(target, Matrix.T(0, 0, 1), Matrix.Perspective(90, 1, 0.1f, 50),
			new RenderSettings { clearColor  = Color.Black,
			                     postProcess = new Material[] { vignette } });
		/// :End:

		model = Model.FromFile("DamagedHelmet.gltf");

		// Testing takes a screenshot of this demo, and screenshots follow the
		// display's post-process chain - turn some effects on for the shot!
		if (Tests.IsTesting)
		{
			vignetteOn = true; Renderer.AddPostProcess(vignetteMat);
			fogOn      = true; Renderer.AddPostProcess(fogMat);
		}
	}

	public void Shutdown()
	{
		Renderer.RemovePostProcess(vignetteMat);
		Renderer.RemovePostProcess(invertMat);
		Renderer.RemovePostProcess(fogMat);
	}

	public void Step()
	{
		UI.WindowBegin("Post Processing", ref windowPose, new Vec2(0.28f, 0));

		if (UI.Toggle("Vignette", ref vignetteOn))
		{
			if (vignetteOn) Renderer.AddPostProcess   (vignetteMat);
			else            Renderer.RemovePostProcess(vignetteMat);
		}
		if (vignetteOn && UI.HSlider("strength", ref vignetteStrength, 0, 1))
			vignetteMat["strength"] = vignetteStrength;

		if (UI.Toggle("Invert", ref invertOn))
		{
			if (invertOn) Renderer.AddPostProcess   (invertMat);
			else          Renderer.RemovePostProcess(invertMat);
		}

		if (UI.Toggle("Depth fog", ref fogOn))
		{
			if (fogOn) Renderer.AddPostProcess   (fogMat);
			else       Renderer.RemovePostProcess(fogMat);
		}
		if (fogOn && UI.HSlider("density", ref fogDensity, 0, 2))
			fogMat["fog_density"] = fogDensity;

		UI.WindowEnd();

		model.Draw(Matrix.TRS(V.XYZ(0, -0.1f, -0.5f), Quat.FromAngles(0, 160, 0), 0.25f));

		Tests.Screenshot("Demos/PostFX.jpg", 600, 600, V.XYZ(0, -0.1f, 0.1f), V.XYZ(0, -0.1f, -0.5f));
		Demo.ShowSummary(title, description, new Bounds(V.XY0(0, -0.16f), V.XYZ(.34f, .5f, 0.6f)));
	}
}
