#!/usr/bin/env bash
# Launch the Go2 SIM mission-control MCP server (stdio transport).
#
# An MCP client spawns this script. It sources ROS 2 + the sim workspace (so rclpy + the ZoneTask interface
# are importable) and matches the sim's DDS env (so it DISCOVERS the running mission_control), then runs
# the server in the venv that has fastmcp. Register it once with your MCP client as a stdio server pointing
# at this script's absolute path.
#
# NOTE: the env exports below MUST match the terminals you run the sim in, or DDS discovery will not find
# the services. They default to the sim runbook (domain 0, UDPv4, localhost-only) but respect any value
# already exported.

# resolve the workspace relative to this script so it runs from any checkout
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/go2_ws"
VENV_PY="$HOME/gauge_venv/bin/python"
SERVER="$WS/src/go2_inspection/go2_inspection/mcp_mission_server.py"

# Source ROS + workspace. Redirect any sourcing chatter to stderr so it never corrupts the MCP stdout
# (the JSON-RPC channel).
source /opt/ros/jazzy/setup.bash 1>&2
source "$WS/install/setup.bash" 1>&2

# DDS discovery: the sim runs on the default domain (0); default DDS transport already discovers
# mission_control (verified). We only pin the domain (the client may spawn us with a minimal env). We do NOT
# force FASTDDS_BUILTIN_TRANSPORTS / ROS_LOCALHOST_ONLY -- forcing localhost-only can MISMATCH a sim that
# was started without it and break discovery. If your sim sets them and the tools report "not available",
# uncomment the two lines below to match your sim terminals. (The client may spawn this with a minimal env,
# so we pin the domain explicitly.)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# export ROS_LOCALHOST_ONLY=1

exec "$VENV_PY" "$SERVER"
