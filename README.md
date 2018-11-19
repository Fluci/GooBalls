# GooBalls

Semester project in Physically-Based Simulations 2018.

## Code base

- `src`: This folder should hold all C++ production code. It is split in a library part `lib` and an applicaiton part `GooBallsApp`. `lib` holds modular components, they should be well designed and reusable. `GoofBallApp` holds "messy" code: Things we want to hard-code, these are the application specific parts that aren't necessarily reusable. If your code can be formulated in a "library-way", please make it nice and clean and put it in `lib`. But there's no need to put all command line arguments in a modular framework, just hard-code them in a configuration `.cpp` file within the app directory and we're good.

- `README.md`: Everything you need to know to get started in this repository.

## Submodules

If you can't find nanogui or one of its dependencies, try:

```bash
$ git submodule update --init --recursive
```

Or 
```bash
$ git submodule update --recursive
```


## Build instructions

As for each cmake project, create a `build` folder and run `cmake ..`. Then call `make` and wait until it compiled.

### Building tools

Ubuntu:

`sudo apt install build-essentials git cmake`

### Dependencies

Boost for unit testing and logging.

Box2D as rigid body solver.

Eigen as linear algebra library.

Ubuntu packages:

`sudo apt install libbox2d-dev libboost-filesystem-dev libboost-program-options-dev libboost-log-dev libboost-test-dev libjsoncpp-dev xorg-dev libgl1-mesa-dev`

