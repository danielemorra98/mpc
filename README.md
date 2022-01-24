# Install dependencies
To install ACADO Toolkit on Ubuntu:
```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz

```
The last three packages are optional but you need them to visualixe the results.

```
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
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
