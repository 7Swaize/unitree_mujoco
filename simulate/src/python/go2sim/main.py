import asyncio
from .bridge import SportBridge


bridge = SportBridge()

async def main():
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, bridge.start)

    try:
        await asyncio.Event().wait()
    finally:
        print("starting shutdown")
        bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())