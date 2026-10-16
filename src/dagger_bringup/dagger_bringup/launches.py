"""The SLAM launches the map manager runs, started and stopped off its executor.

No ROS here (tested offline: tools/test_launches.py). A `ros2 launch` takes seconds
to stop, longer than the gateway waits for a map command's answer: Stop showing
went unanswered on the rover (2026-10-16). So the map manager only says what it
wants -- localize on a map, map or not -- and answers at once; this worker does
it, one launch at a time and in the order asked.
"""
import os
import queue
import signal
import subprocess
import threading

SLAM_STOP_WAIT_S = 15.0


def stop_group(proc):
    """SIGINT a launch's whole process group; SIGKILL if it does not stop in time."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=SLAM_STOP_WAIT_S)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


class Launches:
    """The SLAM launches (shadow mapping, shadow localization on one map), started and
    stopped on a worker thread, one at a time and in the order asked. Never both: the
    node always asks for localization off before mapping on."""

    def __init__(self, folder, log):
        self.folder, self.log = folder, log
        self.q = queue.Queue()
        self.mapping = None            # Popen of the mapping launch
        self.loc = None                # (map name, Popen) of the localization launch
        threading.Thread(target=self._run, daemon=True).start()

    def localize(self, name):
        """Localize on this map ("" = none)."""
        self.q.put(("loc", name))

    def map(self, on):
        self.q.put(("mapping", on))

    def _run(self):
        while True:
            kind, arg = self.q.get()
            try:
                self._localize(arg) if kind == "loc" else self._map(arg)
            except Exception as e:  # noqa: BLE001 -- the worker must outlive one bad launch
                self.log.error(f"{kind} {arg!r}: {e!r}")

    def _launch(self, logname, args):
        log = open(os.path.join(self.folder, logname), "w")
        # Its own process group, so stopping it stops every node the launch started.
        return subprocess.Popen(["ros2", "launch", "dagger_bringup", "slam.launch.py"] + args,
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def _localize(self, name):
        if self.loc is not None and self.loc[0] == name:
            return
        if self.loc is not None:
            stop_group(self.loc[1])
            self.log.info(f"localization on '{self.loc[0]}' stopped")
            self.loc = None
        if name:
            proc = self._launch(".localizing.log", ["mode:=localization", "shadow:=true",
                                                    f"map:={os.path.join(self.folder, name)}"])
            self.loc = (name, proc)
            self.log.info(f"finding the rover on '{name}' (shadow localization, pid {proc.pid})")

    def _map(self, on):
        if on and self.mapping is None:
            self.mapping = self._launch(".mapping.log", [])
            self.log.info(f"mapping started (slam.launch.py, shadow; pid {self.mapping.pid})")
        elif not on and self.mapping is not None:
            stop_group(self.mapping)
            self.mapping = None
            self.log.info("mapping stopped")

    def kill_all(self):
        """At exit: SIGINT to both launches, no waiting."""
        for proc in (self.mapping, self.loc[1] if self.loc else None):
            if proc is not None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
