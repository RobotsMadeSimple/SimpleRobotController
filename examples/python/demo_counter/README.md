# demo_counter

Example plugin for the Python SDK (`plugins-sdk/python`), see `docs/plugins.md` section 8.3.

- step `count` (param `by`, default 1; output `total`)
- function `demo_counter.double(x)`
- property `$demo_counter.total` (republished every second by a background task)
- logs every `program.*` event

## Run it in development

Create a venv, `pip install -r requirements.txt` (installs the SDK from the repo), set
`"runtime": "external"` in `plugin.json`, copy the token from the plugin page, then:

```
SRC_PLUGIN_ID=demo_counter SRC_PLUGIN_URL=ws://127.0.0.1:<port>/plugin SRC_PLUGIN_TOKEN=<token> python main.py
```

## Install it

`requirements.txt` points at the SDK by relative path (`../../../plugins-sdk/python`), which only
works inside this repository. Before zipping the folder for `POST /plugins/install`, pin to a
release instead: replace that line with `simplerobot-plugin==0.1.0` (or
`simplerobot-plugin @ file:///absolute/path/to/plugins-sdk/python`). Restore
`"runtime": "python"` if you switched it to `external`.
