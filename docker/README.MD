# Overview

This folder contains files necessary to package and develop the Hybrid Glider
simulator using Docker. There are two images defined within: the "dev" image
for local development, and the "prod" image for packaging the simulator and
distributing to others for use (not further development).

Both images create a non-privileged user (default: ros 1000:1000) and build
using workspaces located in that user's home folder. All dependencies of
`glider_hybrid_whoi` are located in the `glider_hybrid_whoi_deps` workspace,
and `glider_hybrid_whoi` itself is located in the `glider_hybrid_whoi`
workspace. In both images, the deps are installed to the workspace's install
space and the source is not persisted in the final image.

In the prod image, `glider_hybrid_whoi` is compiled and installed to the
install space. In the dev image, the `glider_hybrid_whoi` workspace is empty
and is designed to have the source code mounted into it for use with the devel
workspace. Additionally, both image uses `fixuid` to ensure the UID of the user
is changed appropriately. This allows you to freely edit/share files without
permissions getting in the way in the dev image and easily connect to the X
server for both images.

Even though we are calling it a "prod" image, it is still really meant for
running locally on trusted input. If this image were ever to be deployed as a
service accepting untrusted input (say for public outreach), a new image should
be made that eschews `fixuid` (it is a SUID executable *and* changing UIDs in a
service is typically an antipattern).

The default name of the image is currently `glider_hybrid_whoi`. This may
change if we decide to publish the image on some hub.

# Dev Image

To create and use an image for development purposes, simply run the
`build-dev.bash` script. The resulting image will be tagged as
`glider_hybrid_whoi:dev`.

You can then run the image using the `run-dev.bash` script. The script accepts
the following arguments. Although if you provide none of them it should do The
Right Thing.

+ `-w`/`--whole-workspace` : Mount the entire Catkin workspace. Instead of
  mounting only the `glider_hybrid_whoi` project to
  `/home/ros/glider_hybrid_whoi/src/glider_hybrid_whoi`, the parent directory
  of the `glider_hybrid_whoi` project is mounted instead.
+ `-i` : Set the image name to use.
+ `--without-nvidia` : Force the script to not expose any GPUs.
+ `--nvidia` : Force the script to expose all GPUs.
+ `--name` : Specify the name of the container.
+ `--run-args` : Extra arguments to pass to the `docker run` command. Must be a
  single string.
+ `--` : Any arguments after this are passed on as the command to use when
  starting the image.

# Prod Image

To create and use an image for non-development purposes, simply run the
`build.bash` script. The resulting image will be tagged as
`glider_hybrid_whoi:latest` and `glider_hybrid_whoi:DATE`.

You can then run the image using the `run.bash` script. The script accepts the
following arguments. Although if you provide none of them it should do The
Right Thing.

+ `-i` : Set the image name to use.
+ `--without-nvidia` : Force the script to not expose any GPUs.
+ `--nvidia` : Force the script to expose all GPUs.
+ `--name` : Specify the name of the container.
+ `--run-args` : Extra arguments to pass to the `docker run` command. Must be a
  single string.
+ `--` : Any arguments after this are passed on as the command to use when
  starting the image.
