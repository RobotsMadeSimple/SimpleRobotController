"""Demo plugin: a counting step, an expression function, a live property and event logging."""
import asyncio

from simplerobot_plugin import Plugin

plugin = Plugin()  # reads SRC_PLUGIN_* from the environment
total = 0.0


@plugin.on_ready
async def ready(ctx):
    ctx.log(f"demo_counter ready, controller {ctx.controller_version}")
    ctx.set_properties(total=total)


@plugin.step("count")
async def count(ctx, params):
    global total
    total += float(params.get("by", 1))
    ctx.set_properties(total=total)
    return {"total": total}


@plugin.function("double")
def double(ctx, x):
    return x * 2


@plugin.on("program.*")
async def program_event(ctx, event):
    ctx.log(f"{event.name}: {event.get('programName')}")


@plugin.background
async def republish(ctx):
    while True:
        ctx.set_properties(total=total)
        await asyncio.sleep(1)


if __name__ == "__main__":
    plugin.run()
