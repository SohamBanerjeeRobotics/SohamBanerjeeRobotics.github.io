# Work section clips

Short muted clips that autoplay/loop across the full width of each
project card on the home page (`index.html`, `#work` section). If a
file here goes missing, the card just falls back to the project's
still photo as a poster.

The fire-drone, motion-planning, and receding-horizon cards each show
two clips side by side (`.work-media.dual`): a live/hardware run
paired with a simulation run.

| File                                  | Card                                                | Source (YouTube id, linked from `_projects/.../index.md`) |
| -------------------------------------- | ---------------------------------------------------- | ------------------------------------------------------ |
| `fire-drone-navigation.mp4`            | Autonomous Navigation in Fire Environments using Drone (left) | `iQTsDOnC4lY`, 0:38–0:50, 1080p source (skips an on-screen recording-software overlay earlier in the video) |
| `fire-environment-simulation.mp4`      | Autonomous Navigation in Fire Environments using Drone (right) | `dBY5fK-LCBU`, 0:15–0:30 — native 638x360 source (that's the highest resolution YouTube has for this one) |
| `bipedal-ankle-pushoff.mp4`            | Underactuated Bipedal Locomotion with Ankle Push-Off | `IacNitoO_S4`, 0:00.5–0:12, 1080p source |
| `mapless-navigation-rl.mp4`            | Virtual-to-Real Mapless Navigation via Deep RL      | `sNinZlbigUI`, full 0:00–0:08.5 — native 638x360 source (highest available) |
| `turtlebot-astar-hardware.mp4`         | Motion Planning & Coordination for Autonomous Vehicles (left) | `9YX_3wYgB8M`, 0:20–0:33 — native 638x360 source (highest available); the paired `dD21LSMiPJs` RRT hardware demo was removed from YouTube, so it's gone from the case-study page too |
| `motion-planning-turtlebot.mp4`        | Motion Planning & Coordination for Autonomous Vehicles (right) | `rTKAuFA87B4`, 0:15–0:28, 1080p source, cropped to `808x724` to cut the source's own letterbox/pillarbox bars (the raw recording pads the sim window on all sides) |
| `receding-horizon-hardware.mp4`        | Receding Horizon Control for Crazyflie Navigation (left) | `K4zf3WzXbAw`, 0:00–0:13 — native 848x480 source (highest available) |
| `receding-horizon-crazyflie.mp4`       | Receding Horizon Control for Crazyflie Navigation (right) | `LKxUU5CVA6w`, 0:00–0:15 — native 640x358 source (highest available) |

Each clip was trimmed from the project's own demo video (already
linked on its `projects/<name>/` case-study page) with `yt-dlp` +
`ffmpeg`: grab the highest-resolution stream YouTube has (1080p where
available), `-an` (video only, since it autoplays muted anyway),
`scale='min(1280,iw)':-2` (caps width, never upscales past the
source's native resolution — upscaling a low-res source just bakes in
blur), H.264 `-preset slow -crf 20` (`crf 18` for the two clips whose
source tops out at 360p, to avoid compounding compression blur on
top of already-soft footage), `+faststart`. Re-run the same recipe
with different `-ss`/`-t` values to pick a different moment, or drop
in a replacement file with the same name.
