using StereoKit;
using System;

// Verifies the Backend.OpenXR pre-session create callback surface:
// - Callbacks registered before SK.Initialize via Backend.OpenXR.OnPreCreateSession
//   and Backend.OpenXR.OnPreCreateSessionInfo
// - On non-OpenXR backends (Simulator), CleanupInitialize releases callbacks cleanly without invoking them.
// - On OpenXR backends (when running with -xr), callbacks fire and receive valid session_info.
// - Calling registration APIs post-initialization behaves gracefully without crashing.
class TestBackendOpenXR : ITest
{
	public static bool   PreInitRegistered     { get; private set; }
	public static bool   PreInitActionInvoked  { get; private set; }
	public static bool   PreInitInfoInvoked    { get; private set; }
	public static IntPtr ReceivedSessionInfo   { get; private set; }

	public static void PreInit()
	{
		Backend.OpenXR.OnPreCreateSession += () => {
			PreInitActionInvoked = true;
		};
		Backend.OpenXR.OnPreCreateSessionInfo += (info) => {
			PreInitInfoInvoked  = true;
			ReceivedSessionInfo = info;
		};

		PreInitRegistered = true;
	}

	public void Initialize()
	{
		Tests.Test(() => PreInitRegistered == true);

		if (Backend.XRType == BackendXRType.OpenXR)
		{
			Tests.Test(() => PreInitActionInvoked == true);
			Tests.Test(() => PreInitInfoInvoked   == true);
			Tests.Test(() => ReceivedSessionInfo  != IntPtr.Zero);
		}
		else
		{
			// On Simulator (offscreen/headless default), OpenXR session was not created.
			// CleanupInitialize safely detached delegates without firing them.
			Tests.Test(() => PreInitActionInvoked == false);
			Tests.Test(() => PreInitInfoInvoked   == false);
		}

		// Post-init registration calls must be safely rejected without crashing.
		Backend.OpenXR.OnPreCreateSession += () => { };
		Backend.OpenXR.OnPreCreateSessionInfo += (info) => { };

		// Querying an unregistered / fake extension name must return false
		Tests.Test(() => Backend.OpenXR.ExtEnabled("XR_NOT_A_REAL_EXTENSION") == false);
	}

	public void Shutdown() { }
	public void Step()     { }
}
