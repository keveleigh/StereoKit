using StereoKit;
using System;

// Verifies the Backend.OpenXR pre-session create callback surface:
// - Callbacks registered before SK.Initialize via Backend.OpenXR.OnPreCreateSession
// - On non-OpenXR backends (Simulator), CleanupInitialize releases callbacks cleanly without invoking them.
// - On OpenXR backends (when running with -xr), callbacks fire.
// - Calling registration APIs post-initialization behaves gracefully without crashing.
class TestBackendOpenXR : ITest
{
	public static bool PreInitRegistered    { get; private set; }
	public static bool PreInitActionInvoked { get; private set; }

	public static void PreInit()
	{
		Backend.OpenXR.OnPreCreateSession += () => {
			PreInitActionInvoked = true;
		};

		PreInitRegistered = true;
	}

	public void Initialize()
	{
		Tests.Test(() => PreInitRegistered == true);

		if (Backend.XRType == BackendXRType.OpenXR)
		{
			Tests.Test(() => PreInitActionInvoked == true);
		}
		else
		{
			// On Simulator (offscreen/headless default), OpenXR session was not created.
			// CleanupInitialize safely detached delegates without firing them.
			Tests.Test(() => PreInitActionInvoked == false);
		}

		// Post-init registration calls must be safely rejected without crashing.
		Backend.OpenXR.OnPreCreateSession += () => { };

		// Querying an unregistered / fake extension name must return false
		Tests.Test(() => Backend.OpenXR.ExtEnabled("XR_NOT_A_REAL_EXTENSION") == false);
	}

	public void Shutdown() { }
	public void Step()     { }
}
