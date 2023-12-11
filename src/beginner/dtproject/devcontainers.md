# Developing in a devcontainer

The dockerized nature of the DTProjects can be leveraged for the development process by creating a `devcontainer.json` configuration file. This enables spinning up a container from the project's docker image and attach an editor/IDE such as VSCode or PyCharm to it, allowing to run the code as if one were developing on a local machine.

```{seealso}
To learn more about devcontainers have a look at the VSCode documentation [here](https://code.visualstudio.com/docs/devcontainers/containers).
```

The dts provides the `vscode open` command to build the DTProject image, create a `devcontainer.json` configuration, spin up an instance of the devcontainer and attach VSCode to it.

```{note}
Using devcontainers is only supported in `v4` of the DTProjects.
```

## Installing the devcontainer CLI

In order to automatically open a devcontainer from the shell you need to install the devcontainer CLI first. You can quickly do so by installing the Dev Containers extension in VSCode and selecting the `Dev Containers: Install devcontainer CLI` command from the Command Palette (`F1`).

## Tutorial

Let's see how to configure the DTProject devcontainer and start it up. There are two files that we use to do this, both of them located in the `dtproject` directory:

- `containers.yaml`
- `devcontainers.yaml`

### 1. Configuring the base container

First it is necessary to configure a base container for the project. This is done in the `containers.yaml`. It allows us to have a container configuration for the project that we can also deploy on robots. This file is written in a docker-compose compatible configuration, learn more about it [here](https://docs.docker.com/compose/compose-file/compose-file-v3/).

```{warning}
In the `environment` field only the dictionary syntax is supported.
```
#### Example:

```yaml
default:
  restart: unless-stopped
  network_mode: host
  privileged: true
  volumes:
    - /data:/data
    # avahi socket
    - /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket
    # nvargus socket
    - /tmp/argus_socket:/tmp/argus_socket
```

In this example file the `default` configuration is allowing the container to have full access to the host's network and devices with the `network_mode` and `privileged` flags respectively and is mapping local directories to the container directories in the `volumes` field. The mapping is of the form `<local-dir>:<container-dir>`. 

It is possible to define several named configurations, as:

```yaml
config_1:
  key: value
  ...

config_2:
  key: value
  ...
```

### 2. Configuring the devcontainer

After setting up the base container we will need to define a configuration for the devcontainer. This is done in the `devcontainers.yaml`, where we can define multiple devcontainers configurations to use.

#### Example:

```yaml
default:
  container: default
  service: dt-devcontainer
  customizations:
    vscode:
      extensions:
        - "ms-iot.vscode-ros"
```

In each configuration we can define which container we want to use as our devcontainer, in this example we chose in the `container` field the `default` configuration from the `containers.yaml` file we had defined previously. We also provide the name we want to give to the `service` (this is the instance of devcontainer we are going to use), in this case `dt-devcontainer`.
You can customize the devcontainer by adding default VSCode extensions you want to have installed in it in the `customization` section, here we are installing the ROS extension.

```{note}
We support a limited subset of the `devcontainer.json` specification, specifically the following field of [the devcontainers reference](https://containers.dev/implementors/json_reference/):

- `container`
- `remoteEnv`
- `remoteUser`
- `containerUser`
- `updateRemoteUserUID`
- `shutdownAction`
- `init`
- `customizations`
- `dockerComposeFile`
- `service`
- `workspaceFolder`
```

### 3. Spin up the devcontainer instance

Now you're ready to spin up a devcontainer. To do so, open a terminal in the project's directory and simply type `dts vscode open --devcontainer`. This will proceed to build the `default` devcontainer and open an instance of VSCode attached to it. 

You can also specify a configuration from `devcontainers.yaml` that you want to open with `--devcontainer <configuration-name>`.

If you want to connect the container to the ROS instance running on your duckiebot you can do so by adding the `-R <duckiebot-name>` flag to the command.

You can learn about additional options of the `vscode open` command by running `dts vscode open --help`.

#### Example:

```{vimeo} 893342004
:alt: example of dts vscode open commmand.
```

If you do not provide the `--devcontainer` the default behavior of `dts vscode open` is to open the project's folder in a local VSCode instance.

Happy coding!
