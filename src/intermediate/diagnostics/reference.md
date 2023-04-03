(sec:devel_sw_diagnostics_reference)=
# Reference

In this section, we will describe the various arguments that the diagnostics
tool accepts. Use them to configure the diagnostics tool to fit your needs.

## Usage

You can run a diagnostics test using the command:

```bash
dts diagnostics run \
    -H/--machine [ROBOT] \
    -G/--group [EXPERIMENT] \
    -d/--duration [SECONDS] \
    [OPTIONS]
```


## Options

The following table describes the **options** available to the diagnostics
tool.

:::{list-table} Options available to the command `dts diagnostics run`
:header-rows: 1
:name: tab:devel_sw_diagnostics_dts_diag_run_options
:widths: 20, 20, 60

* - Argument
  - Type
  - Description
* - `-H`<br/>`--machine`
  - `-`
  - Machine where the diagnostics tool will run. This can be any machine with a network connection to the target machine.
* - `-T`<br/>`--target`
  - localhost
  - Machine target of the diagnostics. This is the machine about which the log is created.
* - `--type`
  - auto
  - Specify a device type (e.g., duckiebot, watchtower). Use `--help` to see the list of allowed values.
* - `--app-id`
  - `-`
  - ID of the API App used to authenticate the push to the server. Must have access to the 'data/set' API endpoint
* - `--app-secret`
  - `-`
  - Secret of the API App used to authenticate the push to the server
* - `-D`<br/>`--database`
  - `-`
  - Name of the logging database. Must be an existing database.
* - `-G`<br/>`--group`
  - `-`
  - Name of the experiment (e.g., new_fan)
* - `-S`<br/>`--subgroup`
  - "default"
  - Name of the test within the experiment (e.g., fan_model_X)
* - `-D`<br/>`--duration`
  - `-`
  - Length of the analysis in seconds, (-1: indefinite)
* - `-F`<br/>`--filter`
  - "*"
  - Specify regexes used to filter the monitored containers
* - `-m`<br/>`--notes`
  - "empty"
  - Custom notes to attach to the log
* - `--no-pull`
  - False
  - Whether we do not try to pull the diagnostics image before running the experiment
* - `--debug`
  - False
  - Run in debug mode
* - `--vv`<br/>`--verbose`
  - False
  - Run in debug mode
:::