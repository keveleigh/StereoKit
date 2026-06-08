// SPDX-License-Identifier: MIT
// The authors below grant copyright rights under the MIT license:
// Copyright (c) 2026 Nick Klingensmith
// Copyright (c) 2026 Qualcomm Technologies, Inc.

using System;
using StereoKit;

/// This test turns off StereoKit's built-in interactor visualization
/// (`Interaction.DefaultDraw`) and attempts to replicate the internal far-ray
/// indicator (`interactor_show_ray` in interactor_modes.cpp) entirely at the
/// application level, using only the public `Interactor` API.
///
/// It drives a single custom Line interactor with simulated motion so the ray
/// focuses and activates a UI button, exercising the "centered" / curved ray
/// behavior the internal indicator produces.
class TestInteractorVis : ITest
{
	DefaultInteractors prevDefault;
	bool               prevDraw;
	Interactor         ray;

	// Per-interactor smoothing state. Internally this lives alongside each
	// default interactor (interact_mode_hands_t::ray_visible / ray_active);
	// here we keep it ourselves since we own the interactor.
	float rayVisible = 0;
	float rayActive  = 0;

	Pose windowPose = new Pose(0, 0, -0.5f, Quat.LookDir(-Vec3.Forward));

	// Origin of the ray, roughly where a hand's aim ray would start.
	Vec3 rayOrigin = V.XYZ(0, -0.15f, 0.25f);

	public void Initialize()
	{
		// Replace SK's default interactors with our own, and turn off the
		// built-in ray visualization so we can draw our own.
		prevDefault = Interaction.DefaultInteractors;
		prevDraw    = Interaction.DefaultDraw;
		Interaction.DefaultInteractors = DefaultInteractors.None;
		Interaction.DefaultDraw        = false;

		ray = Interactor.Create(InteractorType.Line, InteractorEvent.Pinch | InteractorEvent.Poke, InteractorActivation.State, InteractorSource.Max, 0.05f, 2);

		// The creation parameters are now readable back off the interactor.
		Tests.Test(CreationParamsReadBack);

		Tests.RunForFrames(20);
	}

	// Verifies the creation parameters can be read back off the Interactor
	// exactly as they were passed to Interactor.Create above.
	bool CreationParamsReadBack() =>
		ray.Type          == InteractorType.Line                          &&
		ray.Events        == (InteractorEvent.Pinch|InteractorEvent.Poke) &&
		ray.Activation    == InteractorActivation.State                   &&
		ray.Source        == InteractorSource.Max                         &&
		ray.SecondaryDims == 2;

	public void Shutdown()
	{
		ray.Destroy();
		Interaction.DefaultInteractors = prevDefault;
		Interaction.DefaultDraw        = prevDraw;
	}

	Vec3 target;
	public void Step()
	{
		// Aim the ray straight at the button so it stays focused, and report it
		// as active so we exercise the active/curved branch of the indicator.
		Vec3     dir    = (target - rayOrigin).Normalized;
		BtnState active = BtnState.Active;
		ray.Update(rayOrigin, rayOrigin + dir * 100, new Pose(rayOrigin, Quat.LookDir(dir)), rayOrigin, Vec3.Zero, active, BtnState.Active);

		// Our application-level replica of the internal indicator.
		ShowRay(ray, 0.07f, true, ref rayVisible, ref rayActive);

		UI.WindowBegin("Interactor Vis", ref windowPose, V.XY(0.2f, 0));
		UI.Button("Target");
		target = Hierarchy.ToWorld(UI.LayoutLast.center - V.XYZ(UI.LayoutLast.dimensions.x/2 + ray.Radius*0.8f,0,0));
		UI.WindowEnd();

		// Screenshot late enough that the visibility/active smoothing has
		// ramped up to full.
		Tests.Screenshot("Tests/InteractorVis.jpg", 18, 600, 400, 60, V.XYZ(0.25f, 0.1f, 0.4f), V.XYZ(0, 0, -0.5f));
	}

	/// A direct port of `interactor_show_ray` from interactor_modes.cpp, built
	/// only from the public `Interactor` API.
	static void ShowRay(Interactor actor, float skip, bool hideInactive, ref float refVisibleAmt, ref float refActiveAmt)
	{
		if (!actor.Tracked.IsActive()) return;

		bool  actorVisible = hideInactive == false || actor.Focused != IdHash.None;
		refVisibleAmt = SKMath.Lerp(refVisibleAmt, actorVisible ? 1f : 0f, 16.0f * Time.StepUnscaledf);
		float visibility = refVisibleAmt;
		if (visibility < 0.001f) return;

		refActiveAmt = SKMath.Lerp(refActiveAmt, actor.Active != IdHash.None ? 1f : 0f, 16.0f * Time.StepUnscaledf);
		float active = refActiveAmt;

		Vec3  motionPos     = actor.Motion.position;
		float length        = 0.35f;
		Vec3  uncenteredDir = (actor.End - motionPos).Normalized;
		Vec3  centeredDir   = uncenteredDir;
		if (actor.Focused != IdHash.None && actor.TryGetFocusBounds(out Pose poseWorld, out Bounds boundsLocal, out Vec3 atLocal))
		{
			Vec3 pt = poseWorld.ToMatrix().Transform(boundsLocal.center + atLocal);
			length      = Vec3.Distance(pt, motionPos);
			centeredDir = (pt - motionPos).Normalized;
		}
		length = SKMath.Lerp(0.35f, length, visibility);
		length = Math.Max(0, length - skip);

		float alpha = 0.35f + active * 0.65f;
		if (hideInactive) alpha *= visibility;

		const int   ct      = 20;
		const float raySnap = 1.0f;
		LinePoint[] pts = new LinePoint[ct];
		for (int i = 0; i < ct; i++)
		{
			float pct   = (float)i / (ct - 1);
			float blend = pct * pct * pct * raySnap;
			float d     = skip + pct * length;

			float pctI  = 1 - pct;
			float curve = SKMath.Lerp(
				MathF.Sin(pctI * pctI * MathF.PI),
				MathF.Min(1f, MathF.Sin(pct * pct * MathF.PI) * 1.5f), active);
			float width = (0.002f + curve * 0.003f) * visibility;
			Vec3  at    = motionPos + Vec3.Lerp(uncenteredDir * d, centeredDir * d, blend);
			pts[i] = new LinePoint(at, new Color32(255, 255, 255, (byte)(curve * alpha * 255)), width);
		}
		Lines.Add(pts);
	}
}
