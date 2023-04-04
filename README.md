# ros2_ws

Tiny changes with reference to the original repository. Several tasks and launch settings have been changed. Moreover, in this case container has to be created with [rocker](https://github.com/osrf/rocker) using your own docker image. The instructions have been updated.

# VSCode ROS2 Workspace Template

This template will get you set up using ROS2 with VSCode as your IDE.

See [how I develop with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

## Features

### Style

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.  Feel free to adjust them to suit your needs.  

Take a look at [how I develop using tasks](https://www.allisonthackston.com/articles/vscode_tasks.html) for an idea on how I use tasks in my development.

### Debugging

This template sets up debugging for python files and gdb for cpp programs.  See [`.vscode/launch.json`](.vscode/launch.json) for configuration details.

### Continuous Integration

The template also comes with basic continuous integration set up. See [`.github/workflows/ros.yaml`](/.github/workflows/ros.yaml).

To remove a linter just delete it's name from this line:

```yaml
      matrix:
          linter: [cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257]
```

## How to use this template

### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
* [rocker](https://github.com/osrf/rocker)

### Get the template

Click on "use this template"

![template_use](https://user-images.githubusercontent.com/6098197/91331899-43f23b80-e780-11ea-92c8-b4665ce126f1.png)

### Create your repository

On the next dialog, name the repository you would like to start and decide if you want all of the branches, or just the latest LTS: humble.

![template_new](https://user-images.githubusercontent.com/6098197/91332035-713ee980-e780-11ea-81d3-13b170f568b0.png)

Github will then create a new repository with the contents of this one in your account.  It grabs the latest changes as "initial commit".

### Clone your repo

Now you can clone your repo as normal

![template_download](https://user-images.githubusercontent.com/6098197/91332342-e4e0f680-e780-11ea-9525-49b0afa0e4bb.png)

### Download extensions

Open VSCode and download required extensions (Ctrl + Shift + X) on your host:
```
althack.ament-task-provider
DotJoshJohnson.xml
ms-azuretools.vscode-docker
ms-iot.vscode-ros
ms-python.python
ms-vscode.cpptools
ms-vscode-remote.vscode-remote-extensionpack
redhat.vscode-yaml
smilerobotics.urdf
streetsidesoftware.code-spell-checker
twxs.cmake
yzhang.markdown-all-in-one
zachflower.uncrustify
```

### Build and run

Build your Docker image. If needed, edit `.devcontainer/Dockerfile` and add your dependencies. Make sure arg `WORKSPACE` in `build.sh` has the same value as your workspace name.
```bash
cd path/to/your/workspace
cd .devcontainer
./build.sh
cd ..
./run.sh
cd your_workspace
vcs import < default.repos
colcon build
```
To attach container to new terminal use `enter.sh`.

### Attach to container

Now that you've run your container, you can attach to it using VSCode through Remote Explorer (right mouse click on container name -> `Attach to Container`).

![attach_vscode](https://user-images.githubusercontent.com/37396312/212543105-3a0b01ec-fe66-4586-9a9a-526e4e294f3f.png)

When you build your image, you have to install extensions (which you already have on your host) inside the container. To do that, go to Extensions tab (Ctrl + Shift + X), click Cloud icon (Install Local Extensions in...) next to the Container section, select all extensions and install.

![install_extensions](https://user-images.githubusercontent.com/37396312/212544102-ad98c50d-a270-4855-aea6-a0108ba2e4d9.png)

Now you can open your workspace in the container. In Explorer tab (CRT + Shift + E) choose `Open Folder` and select your workspace folder.

### Predefined tasks usage and debugging

`.vscode` contains predefined list of tasks for currently open file. You should pass all the test regarding to the file type (ctrl + shift + p -> Tasks: Run task):
* C++ - fix, uncrustify, cpplint, cppcheck
* CMakesLists.txt - lint_cmake
* Python - flake8, pep257
* XML - xmllint

Moreover, several build tasks are available. Check `.vscode/tasks.json` file to speedup your development, especially `new ROS2 package` task which call custom package creation script with a few build type layout.
To debug your code, switch beetween C++/Python debug mode (Ctrl + Shift + D) and run `Debug: Start Debugging` (F5). The debug call is defined in `.vscode/launch.json` file, thus you need to modify it to fit your needs (e.g. add your package name, executable name etc. to `pickString` inputs).

### Update the template with your code

1. Specify the repositories you want to include in your workspace in `src/default.repos`.
2. If you are using a `default.repos` file, import the contents `Terminal->Run Task..->import from workspace file`.
3. Update your repositories `Terminal->Run Task..->update repositories`.
4. Install dependencies `Terminal->Run Task..->install dependencies`.
5. Build your workspace/package/up-to package using Release/Debug build type `Terminal->Run Task..->build...`.
6. Test your repositories `Terminal->Run Task..->test`.
7. Develop!
