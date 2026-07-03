import subprocess
import signal
import sys
import importlib.resources as resources

def get_cpp_binary():
    return str(resources.files("go2sim").joinpath("cpp/go2sim_cpp"))


def _wait_or_kill(proc: subprocess.Popen, timeout: float = 5.0) -> None:
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def main():
    cpp_bin = get_cpp_binary()

    cpp = subprocess.Popen([cpp_bin])
    py = subprocess.Popen([sys.executable,"-m", "go2sim.main"], start_new_session=True)

    def cleanup(*_):
        py.terminate()
        cpp.terminate()

        _wait_or_kill(py)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    cpp.wait()
    py.wait()


if __name__ == "__main__":
    main()