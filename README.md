# Clone the repo and its dependencies
Clone this repository and its hooked submodules:
```
git clone --recurse-submodules https://github.com/danielemorra98/mpc.git
```
# Install dependencies

## ACADO installation
To install ACADO Toolkit on Ubuntu:
```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```
The last three packages are optional but you need them to visualixe the results.

```
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
```

Check if the installation was successful by running an example:
```
cd ..
cd examples/getting_started
./simple_ocp
```

## Nlohmann Json installation
Just run the following command on a shell:
```
sudo apt-get install nlohmann-json3-dev
```