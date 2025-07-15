import asyncio

import dotenv

import nova

dotenv.load_dotenv()


@nova.program(id="test")
async def test():
    print("Hello, world!")


@nova.program(
    id="simple_program",
    name="Simple Program",
    description="Simple program that prints 'Hello World!' and then sleeps a bit.",
)
async def simple_program(number_of_steps: int = 30):
    """Simple program that prints "Hello World!" and then sleeps a bit."""
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


if __name__ == "__main__":
    import uvicorn

    from novax import Novax

    novax = Novax()
    app = novax.create_app()
    novax.include_programs_router(app)

    novax.register_program(test)
    novax.register_program(simple_program)

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )
