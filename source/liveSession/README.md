An attempt to get https://github.com/gazebosim/gz-omni working with Harmonic and updated Omniverse libraries

The short quick fix was to use the `liveSession` example from `connect-sdk` and copy the code from `gz-omni` in to get it working.

The steps for the current setup:

1. Install Gazebo Harmonic and IsaacSim
2. Follow steps from https://github.com/gazebosim/gz-omni/blob/main/tutorials/01_compile.md in a separate workspace (ie. `gzOmniWs`)
3. Clone this fork and branch https://github.com/gazebosim/connect-samples/tree/jennuine/harmonic
4. Update paths in `premake5.lua` and `run_live_session.sh` to match your setup
5. Terminal 1: `gz sim shapes.sdf`; in Gazebo delete cone, ellipsoid, and capsule
6. Terminal 2: source the `gzOmniWs` then the `connect-sdk` fork run `./run_live_session.sh -p omniverse://localhost/Users/gz/shapes.usd -w shapes --pose gz`, create a new session or join a previously created
7. Run IsaacSim, open `omniverse://localhost/Users/gz/shapes.usd` and join the session from step 6
8. In Gazebo, move the shapes and see the shapes update in IsaacSim