﻿using System;
using System.Collections.Generic;

namespace StereoKit
{
	/// <summary>Do you need to draw something? Well, you're probably in the right place!
	/// This static class includes a variety of different drawing methods, from rendering 
	/// Models and Meshes, to setting rendering options and drawing to offscreen surfaces!
	/// Even better, it's entirely a static class, so you can call it from anywhere :)</summary>
	public static class Renderer
	{
		/// <summary>A queue is used to prevent premature garbage collection
		/// of the user-defined callbacks.</summary>
		private static Queue<RenderOnScreenshotCallback> _renderCaptureCallbacks;

		/// <summary>Set a cubemap skybox texture for rendering a background! This is only visible on Opaque
		/// displays, since transparent displays have the real world behind them already! StereoKit has a
		/// a default procedurally generated skybox. You can load one with `Tex.FromEquirectangular`, 
		/// `Tex.GenCubemap`. If you're trying to affect the lighting, see `Renderer.SkyLight`.</summary>
		public static Tex SkyTex
		{
			get  { IntPtr ptr = NativeAPI.render_get_skytex(); return ptr == IntPtr.Zero ? null : new Tex(ptr); }
			set => NativeAPI.render_set_skytex(value == null ? IntPtr.Zero : value._inst);
		}

		/// <summary>This is the Material that StereoKit is currently using to
		/// draw the skybox! It needs a special shader that's tuned for a
		/// full-screen quad. If you just want to change the skybox image, try
		/// setting `Renderer.SkyTex` instead.
		/// 
		/// This value will never be null! If you try setting this to null, it
		/// will assign SK's built-in default sky material. If you want to turn
		/// off the skybox, see `Renderer.EnableSky` instead.
		/// 
		/// Recommended Material settings would be:
		/// - DepthWrite: false
		/// - DepthTest: LessOrEq
		/// - QueueOffset: 100</summary>
		public static Material SkyMaterial {
			get { return new Material(NativeAPI.render_get_skymaterial()); }
			set => NativeAPI.render_set_skymaterial(value == null ? IntPtr.Zero : value._inst);
		}

		/// <summary>Sets the lighting information for the scene! You can
		/// build one through `SphericalHarmonics.FromLights`, or grab one
		/// from `Tex.FromEquirectangular` or `Tex.GenCubemap`</summary>
		public static SphericalHarmonics SkyLight
		{
			set => NativeAPI.render_set_skylight(value);
			get => NativeAPI.render_get_skylight();
		}

		/// <summary>Enables or disables rendering of the skybox texture! It's enabled by default on Opaque
		/// displays, and completely unavailable for transparent displays.</summary>
		public static bool EnableSky
		{
			get => NativeAPI.render_enabled_skytex();
			set => NativeAPI.render_enable_skytex(value);
		}

		/// <summary>By default, StereoKit renders all first-person layers.
		/// This is a bit flag that allows you to change which layers StereoKit
		/// renders for the primary viewpoint. To change what layers a visual
		/// is on, use a Draw method that includes a RenderLayer as a
		/// parameter.</summary>
		public static RenderLayer LayerFilter {
			set => NativeAPI.render_set_filter(value);
			get => NativeAPI.render_get_filter();
		}

		/// <summary>OpenXR has a recommended default for the main render
		/// surface, this variable allows you to set SK's surface to a multiple
		/// of the recommended size. Note that the final resolution may also be
		/// clamped or quantized. Only works in XR mode. If known in advance,
		/// set this via SKSettings in initialization. This is a _very_ costly
		/// change to make. Consider if ViewportScaling will work for you
		/// instead, and prefer that.</summary>
		public static float Scaling {
			get => NativeAPI.render_get_scaling();
			set => NativeAPI.render_set_scaling(value);
		}

		/// <summary>This allows you to trivially scale down the area of the
		/// swapchain that StereoKit renders to! This can be used to boost
		/// performance in situations where full resolution is not needed, or
		/// to reduce GPU time. This value is locked to the 0-1 range.</summary>
		public static float ViewportScaling {
			get => NativeAPI.render_get_viewport_scaling();
			set => NativeAPI.render_set_viewport_scaling(value);
		}

		/// <summary>Allows you to set the multisample (MSAA) level of the
		/// render surface. Valid values are 1, 2, 4, 8, 16, though some OpenXR
		/// runtimes may clamp this to lower values. Note that while this can
		/// greatly smooth out edges, it also greatly increases RAM usage and
		/// fill rate, so use it sparingly. Only works in XR mode. If known in
		/// advance, set this via SKSettings in initialization. This is a
		/// _very_ costly change to make.</summary>
		public static int Multisample {
			get => NativeAPI.render_get_multisample();
			set => NativeAPI.render_set_multisample(value);
		}

		/// <summary>This tells if CaptureFilter has been overridden to a
		/// specific value via `Renderer.OverrideCaptureFilter`.</summary>
		public static bool HasCaptureFilter => NativeAPI.render_has_capture_filter();

		/// <summary>This is the current render layer mask for Mixed Reality
		/// Capture, or 2nd person observer rendering. By default, this is
		/// directly linked to Renderer.LayerFilter, but this behavior can be
		/// overridden via `Renderer.OverrideCaptureFilter`.</summary>
		public static RenderLayer CaptureFilter => NativeAPI.render_get_capture_filter();

		/// <summary>This is the gamma space color the renderer will clear
		/// the screen to when beginning to draw a new frame.</summary>
		public static Color ClearColor
		{
			get => NativeAPI.render_get_clear_color();
			set => NativeAPI.render_set_clear_color(value);
		}

		/// <summary>Sets and gets the root transform of the camera! This
		/// will be the identity matrix by default. The user's head  location
		/// will then be relative to this point. This is great to use if 
		/// you're trying to do teleportation, redirected walking, or just 
		/// shifting the floor around.</summary>
		public static Matrix CameraRoot
		{
			get => NativeAPI.render_get_cam_root();
			set => NativeAPI.render_set_cam_root(value);
		}

		/// <summary>For flatscreen applications only! This allows you to
		/// change the camera projection between perspective and orthographic
		/// projection. This may be of interest for some category of UI work,
		/// but is generally a niche piece of functionality.
		/// 
		/// Swapping between perspective and orthographic will also switch the
		/// clipping planes and field of view to the values associated with
		/// that mode. See `SetClip`/`SetFov` for perspective, and
		/// `SetOrthoClip`/`SetOrthoSize` for orthographic.</summary>
		public static Projection Projection
		{
			get => NativeAPI.render_get_projection();
			set => NativeAPI.render_set_projection(value);
		}

		/// <summary>The CaptureFilter is a layer mask for Mixed Reality
		/// Capture, or 2nd person observer rendering. On HoloLens and WMR,
		/// this is the video rendering feature. This allows you to hide, or
		/// reveal certain draw calls when rendering video output.
		/// 
		/// By default, the CaptureFilter will always be the same as
		/// `Render.LayerFilter`, overriding this will mean this filter no
		/// longer updates with `LayerFilter`.</summary>
		/// <param name="useOverrideFilter">Enables (true) or disables (false)
		/// the overridden filter value provided here.</param>
		/// <param name="overrideFilter">The filter for capture rendering to
		/// use. This is ignored if useOverrideFilter is false.</param>
		public static void OverrideCaptureFilter(bool useOverrideFilter, RenderLayer overrideFilter = RenderLayer.All)
			=> NativeAPI.render_override_capture_filter(useOverrideFilter, overrideFilter);

		/// <summary>Adds a mesh to the render queue for this frame! If the
		/// Hierarchy has a transform on it, that transform is combined with
		/// the Matrix provided here.</summary>
		/// <param name="mesh">A valid Mesh you wish to draw.</param>
		/// <param name="material">A Material to apply to the Mesh.</param>
		/// <param name="transform">A Matrix that will transform the mesh
		/// from Model Space into the current Hierarchy Space.</param>
		public static void Add(Mesh mesh, Material material, Matrix transform)
			=> NativeAPI.render_add_mesh(mesh._inst, material._inst, transform, Color.White, RenderLayer.Layer0);
		/// <summary>Adds a mesh to the render queue for this frame! If the
		/// Hierarchy has a transform on it, that transform is combined with
		/// the Matrix provided here.</summary>
		/// <param name="mesh">A valid Mesh you wish to draw.</param>
		/// <param name="material">A Material to apply to the Mesh.</param>
		/// <param name="transform">A Matrix that will transform the mesh
		/// from Model Space into the current Hierarchy Space.</param>
		/// <param name="colorLinear">A per-instance linear space color value
		/// to pass into the shader! Normally this gets used like a material
		/// tint. If you're  adventurous and don't need per-instance colors,
		/// this is a great spot to pack in extra per-instance data for the
		/// shader!</param>
		/// <param name="layer">All visuals are rendered using a layer 
		/// bit-flag. By default, all layers are rendered, but this can be 
		/// useful for filtering out objects for different rendering 
		/// purposes! For example: rendering a mesh over the user's head from
		/// a 3rd person perspective, but filtering it out from the 1st
		/// person perspective.</param>
		public static void Add(Mesh mesh, Material material, Matrix transform, Color colorLinear, RenderLayer layer = RenderLayer.Layer0)
			=> NativeAPI.render_add_mesh(mesh._inst, material._inst, transform, colorLinear, layer);

		/// <summary>Adds a Model to the render queue for this frame! If the
		/// Hierarchy has a transform on it, that transform is combined with
		/// the Matrix provided here.</summary>
		/// <param name="model">A valid Model you wish to draw.</param>
		/// <param name="transform">A Matrix that will transform the Model
		/// from Model Space into the current Hierarchy Space.</param>
		public static void Add(Model model, Matrix transform)
			=> NativeAPI.render_add_model(model._inst, transform, Color.White, RenderLayer.Layer0);
		/// <summary>Adds a Model to the render queue for this frame! If the
		/// Hierarchy has a transform on it, that transform is combined with
		/// the Matrix provided here.</summary>
		/// <param name="model">A valid Model you wish to draw.</param>
		/// <param name="transform">A Matrix that will transform the Model
		/// from Model Space into the current Hierarchy Space.</param>
		/// <param name="colorLinear">A per-instance linear space color value
		/// to pass into the shader! Normally this gets used like a material
		/// tint. If you're  adventurous and don't need per-instance colors,
		/// this is a great spot to pack in extra per-instance data for the
		/// shader!</param>
		/// <param name="layer">All visuals are rendered using a layer 
		/// bit-flag. By default, all layers are rendered, but this can be 
		/// useful for filtering out objects for different rendering 
		/// purposes! For example: rendering a mesh over the user's head from
		/// a 3rd person perspective, but filtering it out from the 1st
		/// person perspective.</param>
		public static void Add(Model model, Matrix transform, Color colorLinear, RenderLayer layer = RenderLayer.Layer0)
			=> NativeAPI.render_add_model(model._inst, transform, colorLinear, layer);

		/// <summary>Set the near and far clipping planes of the camera!
		/// These are important to z-buffer quality, especially when using
		/// low bit depth z-buffers as recommended for devices like the
		/// HoloLens. The smaller the range between the near and far planes,
		/// the better your z-buffer will look! If you see flickering on
		/// objects that are overlapping, try making the range smaller.
		/// 
		/// These values only affect perspective mode projection, which is the
		/// default projection mode.</summary>
		/// <param name="nearPlane">The GPU discards pixels that are too
		/// close to the camera, this is that distance! It must be larger
		/// than zero, due to the projection math, which also means that
		/// numbers too close to zero will produce z-fighting artifacts. This
		/// has an enforced minimum of 0.001, but you should probably stay
		/// closer to 0.1.</param>
		/// <param name="farPlane">At what distance from the camera does the
		/// GPU discard pixel? This is not true distance, but rather Z-axis
		/// distance from zero in View Space coordinates!</param>
		public static void SetClip(float nearPlane = 0.08f, float farPlane = 50)
			=> NativeAPI.render_set_clip(nearPlane, farPlane);

		/// <summary>Only works for flatscreen! This updates the camera's 
		/// projection matrix with a new field of view.
		/// 
		/// This value only affects perspective mode projection, which is the
		/// default projection mode.</summary>
		/// <param name="fieldOfViewDegrees">Vertical field of view in degrees.
		/// </param>
		public static void SetFOV(float fieldOfViewDegrees)
			=> NativeAPI.render_set_fov(fieldOfViewDegrees);

		/// <summary>Set the near and far clipping planes of the camera!
		/// These are important to z-buffer quality, especially when using
		/// low bit depth z-buffers as recommended for devices like the
		/// HoloLens. The smaller the range between the near and far planes,
		/// the better your z-buffer will look! If you see flickering on
		/// objects that are overlapping, try making the range smaller.
		/// 
		/// These values only affect orthographic mode projection, which is 
		/// only available in flatscreen.</summary>
		/// <param name="nearPlane">The GPU discards pixels that are too
		/// close to the camera, this is that distance!</param>
		/// <param name="farPlane">At what distance from the camera does the
		/// GPU discard pixel? This is not true distance, but rather Z-axis
		/// distance from zero in View Space coordinates!</param>
		public static void SetOrthoClip(float nearPlane = 0, float farPlane = 50)
			=> NativeAPI.render_set_ortho_clip(nearPlane, farPlane);

		/// <summary>This sets the size of the orthographic projection's
		/// viewport. You can use this feature to zoom in and out of the scene.
		/// 
		/// This value only affects orthographic mode projection, which is only
		/// available in flatscreen.</summary>
		/// <param name="viewportHeightMeters">The vertical size of the
		/// projection's viewport, in meters.</param>
		public static void SetOrthoSize(float viewportHeightMeters)
			=> NativeAPI.render_set_ortho_size(viewportHeightMeters);

		/// <summary>Renders a Material onto a rendertarget texture! StereoKit uses a 4 vert quad stretched
		/// over the surface of the texture, and renders the material onto it to the texture.</summary>
		/// <param name="toRendertarget">A texture that's been set up as a render target!</param>
		/// <param name="material">This material is rendered onto the texture! Set it up like you would
		/// if you were applying it to a plane, or quad mesh.</param>
		public static void Blit(Tex toRendertarget, Material material)
			=> NativeAPI.render_blit(toRendertarget._inst, material._inst);

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given position at the given point, with a
		/// resolution the same size as the screen's surface. It'll be saved as
		/// a JPEG or PNG file depending on the filename extension provided.
		/// </summary>
		/// <param name="from">Viewpoint location.</param>
		/// <param name="at">Direction the viewpoint is looking at.</param>
		/// <param name="width">Size of the screenshot horizontally, in pixels.</param>
		/// <param name="height">Size of the screenshot vertically, in pixels.</param>
		/// <param name="filename">Filename to write the screenshot to! This
		/// will be a PNG if the extension ends with (case insensitive)
		/// ".png", and will be a 90 quality JPEG if it ends with anything
		/// else.</param>
		[Obsolete("For removal in v0.4. Use the overload that takes filename first.")]
		public static void Screenshot(Vec3 from, Vec3 at, int width, int height, string filename)
			=> NativeAPI.render_screenshot_pose(NativeHelper.ToUtf8(filename), 90, Pose.LookAt(from, at), width, height, 90);

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given position at the given point, with a
		/// resolution the same size as the screen's surface. It'll be saved as
		/// a JPEG or PNG file depending on the filename extension provided.
		/// </summary>
		/// <param name="filename">Filename to write the screenshot to! This
		/// will be a PNG if the extension ends with (case insensitive)
		/// ".png", and will be a 90 quality JPEG if it ends with anything
		/// else.</param>
		/// <param name="from">Viewpoint location.</param>
		/// <param name="at">Direction the viewpoint is looking at.</param>
		/// <param name="width">Size of the screenshot horizontally, in pixels.
		/// </param>
		/// <param name="height">Size of the screenshot vertically, in pixels.
		/// </param>
		/// <param name="fieldOfViewDegrees">The angle of the viewport, in 
		/// degrees.</param>
		public static void Screenshot(string filename, Vec3 from, Vec3 at, int width, int height, float fieldOfViewDegrees = 90)
			=> NativeAPI.render_screenshot_pose(NativeHelper.ToUtf8(filename), 90, Pose.LookAt(from, at), width, height, fieldOfViewDegrees);

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given pose, with a resolution the same
		/// size as the screen's surface. It'll be saved as a JPEG or PNG file
		/// depending on the filename extension provided.</summary>
		/// <param name="filename">Filename to write the screenshot to! This
		/// will be a PNG if the extension ends with (case insensitive)
		/// ".png", and will be a JPEG if it ends with anything else.</param>
		/// <param name="fileQuality">For JPEG files, this is the compression
		/// quality of the file from 0-100, 100 being highest quality, 0 being
		/// smallest size. SK uses a default of 90 here.</param>
		/// <param name="viewpoint">Viewpoint location and orientation.</param>
		/// <param name="width">Size of the screenshot horizontally, in pixels.
		/// </param>
		/// <param name="height">Size of the screenshot vertically, in pixels.
		/// </param>
		/// <param name="fieldOfViewDegrees">The angle of the viewport, in 
		/// degrees.</param>
		public static void Screenshot(string filename, int fileQuality, Pose viewpoint, int width, int height, float fieldOfViewDegrees = 90)
			=> NativeAPI.render_screenshot_pose(NativeHelper.ToUtf8(filename), fileQuality, viewpoint, width, height, fieldOfViewDegrees);

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given pose, with a resolution the same
		/// size as the screen's surface. It'll be saved as a JPEG or PNG file
		/// depending on the filename extension provided.</summary>
		/// <param name="filename">Filename to write the screenshot to! This
		/// will be a PNG if the extension ends with (case insensitive)
		/// ".png", and will be a 90 quality JPEG if it ends with anything
		/// else.</param>
		/// <param name="viewpoint">Viewpoint location and orientation.</param>
		/// <param name="width">Size of the screenshot horizontally, in pixels.
		/// </param>
		/// <param name="height">Size of the screenshot vertically, in pixels.
		/// </param>
		/// <param name="fieldOfViewDegrees">The angle of the viewport, in 
		/// degrees.</param>
		public static void Screenshot(string filename, Pose viewpoint, int width, int height, float fieldOfViewDegrees = 90)
			=> NativeAPI.render_screenshot_pose(NativeHelper.ToUtf8(filename), 90, viewpoint, width, height, fieldOfViewDegrees);

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given position at the given point, with a
		/// resolution the same size as the screen's surface. This overload
		/// allows for retrieval of the color data directly from the render
		/// thread! You can use the color data directly by saving/processing it
		/// inside your callback, or you can keep the data alive for as long as
		/// it is referenced.</summary>
		/// <param name="onScreenshot">Outputs a reference to the color data
		/// and its length which represent the current scene from a requested
		/// viewpoint.</param>
		/// <param name="from">Viewpoint location.</param>
		/// <param name="at">Direction the viewpoint is looking at.</param>
		/// <param name="width">Size of the screenshot horizontally, in pixels.
		/// </param>
		/// <param name="height">Size of the screenshot vertically, in pixels.
		/// </param>
		/// <param name="fieldOfViewDegrees">The angle of the viewport, in 
		/// degrees.</param>
		/// <param name="texFormat">The pixel format of the color data.</param>
		public static void Screenshot(ScreenshotCallback onScreenshot, Vec3 from, Vec3 at, int width, int height, float fieldOfViewDegrees = 90, TexFormat texFormat = TexFormat.Rgba32)
		{
			if (_renderCaptureCallbacks is null) _renderCaptureCallbacks = new Queue<RenderOnScreenshotCallback>();
			RenderOnScreenshotCallback renderCaptureCallback = (IntPtr dataPtr, int w, int h, IntPtr context) =>
			{
				onScreenshot.Invoke(dataPtr, w, h);
				_ = _renderCaptureCallbacks.Dequeue();
			};
			_renderCaptureCallbacks.Enqueue(renderCaptureCallback);
			NativeAPI.render_screenshot_capture(renderCaptureCallback, Pose.LookAt(from, at), width, height, fieldOfViewDegrees, texFormat, IntPtr.Zero);
		}

		/// <summary>Schedules a screenshot for the end of the frame! The view
		/// will be rendered from the given position at the given point, with a
		/// resolution the same size as the screen's surface. This overload
		/// allows for retrieval of the color data directly from the render
		/// thread! You can use the color data directly by saving/processing it
		/// inside your callback, or you can keep the data alive for as long as
		/// it is referenced.</summary>
		/// <param name="onScreenshot">Outputs a reference to the color data
		/// and its length which represent the current scene from a requested
		/// viewpoint.</param>
		/// <param name="camera">A TRS matrix representing the location and
		/// orientation of the camera. This matrix gets inverted later on, so
		/// no need to do it yourself.</param>
		/// <param name="projection">The projection matrix describes how the
		/// geometry is flattened onto the draw surface. Normally, you'd use 
		/// Matrix.Perspective, and occasionally Matrix.Orthographic might be
		/// helpful as well.</param>
		/// <param name="layerFilter">This is a bit flag that allows you to
		/// change which layers StereoKit renders for this particular render
		/// viewpoint. To change what layers a visual is on, use a Draw
		/// method that includes a RenderLayer as a parameter.</param>
		/// <param name="clear">Describes if and how the rendertarget should
		/// be cleared before rendering. Note that clearing the target is
		/// unaffected by the viewport, so this will clean the entire 
		/// surface!</param>
		/// <param name="texFormat">The pixel format of the color data.</param>
		public static void Screenshot(ScreenshotCallback onScreenshot, Matrix camera, Matrix projection, int width, int height, RenderLayer layerFilter = RenderLayer.All, RenderClear clear = RenderClear.All, Rect viewport = default(Rect), TexFormat texFormat = TexFormat.Rgba32)
		{
			if (_renderCaptureCallbacks is null) _renderCaptureCallbacks = new Queue<RenderOnScreenshotCallback>();
			RenderOnScreenshotCallback renderCaptureCallback = (IntPtr dataPtr, int w, int h, IntPtr context) =>
			{
				onScreenshot.Invoke(dataPtr, w, h);
				_ = _renderCaptureCallbacks.Dequeue();
			};
			_renderCaptureCallbacks.Enqueue(renderCaptureCallback);
			NativeAPI.render_screenshot_viewpoint(renderCaptureCallback, camera, projection, width, height, layerFilter, clear, viewport, texFormat, IntPtr.Zero);
		}

		/// <summary>This renders the current scene to the indicated 
		/// rendertarget texture, from the specified viewpoint. This call 
		/// enqueues a render that occurs immediately before the screen 
		/// itself is rendered.</summary>
		/// <param name="toRendertarget">The texture to which the scene will
		/// be rendered to. This must be a Rendertarget type texture.</param>
		/// <param name="camera">A TRS matrix representing the location and
		/// orientation of the camera. This matrix gets inverted later on, so
		/// no need to do it yourself.</param>
		/// <param name="projection">The projection matrix describes how the
		/// geometry is flattened onto the draw surface. Normally, you'd use 
		/// Matrix.Perspective, and occasionally Matrix.Orthographic might be
		/// helpful as well.</param>
		/// <param name="layerFilter">This is a bit flag that allows you to
		/// change which layers StereoKit renders for this particular render
		/// viewpoint. To change what layers a visual is on, use a Draw
		/// method that includes a RenderLayer as a parameter.</param>
		/// <param name="clear">Describes if and how the rendertarget should
		/// be cleared before rendering. Note that clearing the target is
		/// unaffected by the viewport, so this will clean the entire 
		/// surface!</param>
		/// <param name="viewport">Allows you to specify a region of the
		/// rendertarget to draw to! This is in normalized coordinates, 0-1.
		/// If the width of this value is zero, then this will render to the
		/// entire texture.</param>
		public static void RenderTo(Tex toRendertarget, Matrix camera, Matrix projection, RenderLayer layerFilter = RenderLayer.All, RenderClear clear = RenderClear.All, Rect viewport = default(Rect))
			=> NativeAPI.render_to(toRendertarget._inst, camera, projection, layerFilter, clear, viewport);

	}
}
