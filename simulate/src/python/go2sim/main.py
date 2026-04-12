import asyncio
from .bridge import SportBridge



async def main():
    bridge = SportBridge()

    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, bridge._start)

    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass