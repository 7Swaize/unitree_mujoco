import subprocess
import signal
import sys
import importlib.resources as resources

def get_cpp_binary():
    return str(resources.files("go2sim").joinpath("cpp/go2sim_cpp"))

def main():
    cpp_bin = get_cpp_binary()

    cpp = subprocess.Popen([cpp_bin])
    py = subprocess.Popen([sys.executable,"-m", "go2sim.main"])

    def cleanup(*_):
        cpp.terminate()
        py.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    cpp.wait()
    py.wait()


if __name__ == "__main__":
    main()