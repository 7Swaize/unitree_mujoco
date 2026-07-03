import asyncio
import signal
from .bridge import SportBridge


bridge = SportBridge()

async def main():
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, bridge.start)

    stop_event = asyncio.Event()

    def _request_shutdown():
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_shutdown)

    try:
        await stop_event.wait()
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())