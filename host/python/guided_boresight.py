"""Interactive fixed-wall ToF/body boresight capture workflow.

This command owns capture warm-up and motion timing so builders do not need to
coordinate manual sleeps or know when ``capture.py`` begins writing packets.
It deliberately reuses the existing canonical capture and boresight inspectors
rather than introducing a second acquisition/quality path.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import time


DEFAULT_WARMUP_SECONDS = 3.0
DEFAULT_STATIONARY_CAPTURE_SECONDS = 8.0
DEFAULT_MOTION_CAPTURE_SECONDS = 14.0
DEFAULT_RECORDED_INITIAL_HOLD_SECONDS = 5.0
DEFAULT_MOVE_SECONDS = 3.0

STEP_INSTRUCTIONS = (
    "Pitch upward by roughly 15 deg (+device_body Rx).",
    "Move partway back toward neutral pitch and yaw right by roughly 15 deg (+device_body Ry).",
    "Yaw left across neutral to a clearly different pose (-device_body Ry), keeping a modest pitch if convenient.",
    "Add roughly 15 deg clockwise roll viewed from behind (+device_body Rz), with a modest pitch component.",
)


def _banner(text: str) -> None:
    line = "=" * 72
    print()
    print(line)
    print(text)
    print(line)
    print("\a", end="", flush=True)


def _session_prefix() -> str:
    return "boresight-guided-" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _capture_command(
    *,
    port: str,
    root: Path,
    name: str,
    seconds: float,
    warmup_seconds: float,
    notes: str = "",
) -> list[str]:
    script = Path(__file__).with_name("capture.py")
    command = [
        sys.executable,
        str(script),
        port,
        "--seconds",
        str(float(seconds)),
        "--warmup",
        str(float(warmup_seconds)),
        "--root",
        str(root),
        "--name",
        name,
    ]
    if notes:
        command.extend(("--notes", notes))
    return command


def _sequence_command(
    baseline: Path,
    steps: list[tuple[Path, Path]],
) -> list[str]:
    script = Path(__file__).with_name("inspect_boresight_sequence.py")
    command = [sys.executable, str(script), "--baseline", str(baseline)]
    for motion, pose in steps:
        command.extend(("--step", str(motion), str(pose)))
    return command


def _validate_timing(
    *,
    warmup_seconds: float,
    motion_capture_seconds: float,
    recorded_initial_hold_seconds: float,
    move_seconds: float,
) -> None:
    if warmup_seconds < 0.0:
        raise ValueError("warm-up cannot be negative")
    if motion_capture_seconds <= 0.0:
        raise ValueError("motion capture duration must be positive")
    if recorded_initial_hold_seconds < 1.0:
        raise ValueError("recorded initial hold must be at least 1 second")
    if move_seconds <= 0.0:
        raise ValueError("move duration must be positive")
    final_hold = motion_capture_seconds - recorded_initial_hold_seconds - move_seconds
    if final_hold < 2.0:
        raise ValueError("motion schedule must leave at least 2 seconds of recorded final hold")


def _new_capture_directory(root: Path, before: set[Path], name: str) -> Path:
    candidates = [
        path
        for path in root.glob("capture_*")
        if path.is_dir() and path not in before and path.name.endswith("_" + name)
    ]
    if len(candidates) != 1:
        raise RuntimeError(
            "expected exactly one new capture ending in {!r}; found {}".format(
                name, len(candidates)
            )
        )
    return candidates[0]


def _run_stationary_capture(
    *,
    port: str,
    root: Path,
    name: str,
    seconds: float,
    warmup_seconds: float,
    notes: str,
) -> Path:
    _banner("HOLD STILL — stationary ToF pose capture")
    print(
        "Keep the sensing head completely stationary until the capture command finishes.\n"
        "The first {:.1f} s are acquisition warm-up and are discarded automatically.".format(
            warmup_seconds
        )
    )
    before = {path for path in root.glob("capture_*") if path.is_dir()}
    result = subprocess.run(
        _capture_command(
            port=port,
            root=root,
            name=name,
            seconds=seconds,
            warmup_seconds=warmup_seconds,
            notes=notes,
        ),
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"capture.py failed with exit code {result.returncode}")
    return _new_capture_directory(root, before, name)


def _wait_with_process(process: subprocess.Popen, seconds: float, phase: str) -> None:
    deadline = time.monotonic() + seconds
    while True:
        returncode = process.poll()
        if returncode is not None:
            raise RuntimeError(
                f"capture.py exited early with code {returncode} during {phase}"
            )
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(0.10, remaining))


def _run_motion_capture(
    *,
    port: str,
    root: Path,
    name: str,
    instruction: str,
    seconds: float,
    warmup_seconds: float,
    recorded_initial_hold_seconds: float,
    move_seconds: float,
    notes: str,
) -> Path:
    _banner("HOLD STILL — motion capture starting")
    print("Do not move yet.")
    print("Upcoming movement: " + instruction)
    print(
        "The tool will discard {:.1f} s of warm-up, then retain {:.1f} s of the initial "
        "stationary pose before giving the MOVE NOW cue.".format(
            warmup_seconds, recorded_initial_hold_seconds
        )
    )

    before = {path for path in root.glob("capture_*") if path.is_dir()}
    process = subprocess.Popen(
        _capture_command(
            port=port,
            root=root,
            name=name,
            seconds=seconds,
            warmup_seconds=warmup_seconds,
            notes=notes,
        )
    )

    _wait_with_process(
        process,
        warmup_seconds + recorded_initial_hold_seconds,
        "warm-up/initial hold",
    )
    _banner("MOVE NOW")
    print(instruction)
    print("Move smoothly and continuously; exact angles and exact timing are not measurements.")

    _wait_with_process(process, move_seconds, "movement")
    _banner("STOP MOVING — HOLD THIS NEW POSE")
    print("Keep the sensing head completely stationary until capture.py finishes.")

    returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(f"capture.py failed with exit code {returncode}")
    return _new_capture_directory(root, before, name)


def _run_sequence_check(baseline: Path, steps: list[tuple[Path, Path]]) -> None:
    _banner("AUTOMATIC BORESIGHT QUALITY CHECK")
    result = subprocess.run(_sequence_command(baseline, steps), check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "boresight quality check failed; keep the captures and stop this sequence"
        )


def _print_plan(
    *,
    steps: int,
    warmup_seconds: float,
    stationary_capture_seconds: float,
    motion_capture_seconds: float,
    recorded_initial_hold_seconds: float,
    move_seconds: float,
) -> None:
    print("Rangeweave guided fixed-wall boresight plan")
    print("  P0: stationary baseline")
    for index in range(steps):
        print(f"  M{index + 1}: {STEP_INSTRUCTIONS[index]}")
        print(f"  P{index + 1}: stationary endpoint pose")
    print()
    print(
        "  motion timing: {:.1f}s discarded warm-up + {:.1f}s recorded initial hold + "
        "~{:.1f}s movement + {:.1f}s recorded final hold".format(
            warmup_seconds,
            recorded_initial_hold_seconds,
            move_seconds,
            motion_capture_seconds - recorded_initial_hold_seconds - move_seconds,
        )
    )
    print(
        "  stationary pose: {:.1f}s discarded warm-up + {:.1f}s recorded hold".format(
            warmup_seconds, stationary_capture_seconds
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Interactively capture and quality-check a fixed-wall ToF/body boresight sequence"
    )
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--root", default="captures", help="capture directory root")
    parser.add_argument(
        "--steps",
        type=int,
        default=4,
        help="number of motion/pose pairs (1..4; default 4 gives five stationary poses)",
    )
    parser.add_argument("--warmup", type=float, default=DEFAULT_WARMUP_SECONDS)
    parser.add_argument(
        "--stationary-seconds",
        type=float,
        default=DEFAULT_STATIONARY_CAPTURE_SECONDS,
        help="recorded duration of each stationary pose capture",
    )
    parser.add_argument(
        "--motion-seconds",
        type=float,
        default=DEFAULT_MOTION_CAPTURE_SECONDS,
        help="recorded duration of each hold-move-hold motion capture",
    )
    parser.add_argument(
        "--initial-hold",
        type=float,
        default=DEFAULT_RECORDED_INITIAL_HOLD_SECONDS,
        help="recorded stationary time before MOVE NOW",
    )
    parser.add_argument(
        "--move-seconds",
        type=float,
        default=DEFAULT_MOVE_SECONDS,
        help="approximate time between MOVE NOW and STOP MOVING cues",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the sequence/timing plan without opening the serial port",
    )
    args = parser.parse_args()

    if not 1 <= args.steps <= len(STEP_INSTRUCTIONS):
        parser.error(f"--steps must be between 1 and {len(STEP_INSTRUCTIONS)}")
    if args.stationary_seconds <= 0.0:
        parser.error("--stationary-seconds must be positive")
    try:
        _validate_timing(
            warmup_seconds=args.warmup,
            motion_capture_seconds=args.motion_seconds,
            recorded_initial_hold_seconds=args.initial_hold,
            move_seconds=args.move_seconds,
        )
    except ValueError as exc:
        parser.error(str(exc))

    _print_plan(
        steps=args.steps,
        warmup_seconds=args.warmup,
        stationary_capture_seconds=args.stationary_seconds,
        motion_capture_seconds=args.motion_seconds,
        recorded_initial_hold_seconds=args.initial_hold,
        move_seconds=args.move_seconds,
    )
    if args.dry_run:
        return 0

    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    prefix = _session_prefix()

    print()
    print("Physical setup before starting:")
    print("  - rigid sensing head on a 3-axis holder")
    print("  - approximately 500 mm from a flat wall")
    print("  - ToF aimed near the centre of at least ~1 m x 1 m clear wall area")
    print("  - P0 approximately square-on; exact distance/angles are not measurements")
    input("\nPress Enter when P0 is aligned and you are ready to keep the rig still... ")

    try:
        baseline = _run_stationary_capture(
            port=args.port,
            root=root,
            name=prefix + "-p0",
            seconds=args.stationary_seconds,
            warmup_seconds=args.warmup,
            notes="guided boresight P0 baseline; fixed wall",
        )
        _run_sequence_check(baseline, [])

        captured_steps: list[tuple[Path, Path]] = []
        for index in range(args.steps):
            step_number = index + 1
            instruction = STEP_INSTRUCTIONS[index]
            _banner(f"NEXT: M{step_number} -> P{step_number}")
            print(instruction)
            print("Do not reposition the wall. Do not move the sensing head yet.")
            input("Press Enter when you are ready for the automatic HOLD / MOVE / STOP cues... ")

            motion = _run_motion_capture(
                port=args.port,
                root=root,
                name=f"{prefix}-m{step_number}",
                instruction=instruction,
                seconds=args.motion_seconds,
                warmup_seconds=args.warmup,
                recorded_initial_hold_seconds=args.initial_hold,
                move_seconds=args.move_seconds,
                notes=f"guided boresight M{step_number}; {instruction}",
            )

            pose = _run_stationary_capture(
                port=args.port,
                root=root,
                name=f"{prefix}-p{step_number}",
                seconds=args.stationary_seconds,
                warmup_seconds=args.warmup,
                notes=f"guided boresight P{step_number}; endpoint after M{step_number}",
            )
            captured_steps.append((motion, pose))
            _run_sequence_check(baseline, captured_steps)

    except (OSError, RuntimeError) as exc:
        _banner("SEQUENCE STOPPED")
        print(str(exc))
        print("No captures are deleted. Keep the wall/rig fixed if you want to diagnose or retry immediately.")
        return 2

    _banner("GUIDED CAPTURE SEQUENCE COMPLETE")
    print("Session prefix: " + prefix)
    print("Baseline:       " + str(baseline))
    for index, (motion, pose) in enumerate(captured_steps, start=1):
        print(f"M{index}:             {motion}")
        print(f"P{index}:             {pose}")
    print()
    print(
        "The last pose is retained for held-out validation. Do not promote the all-pose solver "
        "result as the final extrinsic yet."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
