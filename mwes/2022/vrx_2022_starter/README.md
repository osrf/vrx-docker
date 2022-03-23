# VRX 2022 Starter Image

## Summary
The intent of this image is to give an example of a Dockerfile that could be
used by competitors who want to copy custom source code into a workspace in
their image and build it. 

### Note about this example:
* We use the `vrx` repository as an example of repository of custom source 
code that you may want to build.
* In practice, your image may not need to build the `vrx` environment.
* If this is the case, simply replace the `vrx` repository with whatever you
  do want to use, and update the `Dockerfile` to use the correct name.
   
## Build
* Clone the source repository into the directory that contains this `Dockerfile` and the startup scripts:
  ```
  cd vrx-docker/mwes/2022/vrx_2022_starter
  git clone https://github.com/osrf/vrx
  ```
* Make sure there is a `COPY` command in your `Dockerfile` that will put this
  directory where it needs to go inside your image. For example:
  ```
  COPY vrx /vrx_ws/src/vrx   
  ```
* Build the docker image:
  ```
  docker build -t <image_name> .
  ``` 
  where `<image_name>` is a name of your choosing.

### Rebuilding your image
* If you change your source code and want the changes to be reflected in your
image, simply re-run the build command. 
* The `COPY` command will detect any local changes and cause `docker` to copy
  the updated file.
* This will also cause all subsequent commands in the `Dockerfile` to be re-run.
* This feature is the main reason to use `COPY` to copy files from a local
  filesystem, rather than using `git clone` to pull directoy from a remote
  repository.

### Run
To run your image, execute:
```
docker run -it <image_name> 
```
