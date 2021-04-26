# Nosetip finder
This is part of my undergraduate thesis, an algorithm to find nose tips in point cloud images. Check also [the web viewer repository](https://github.com/MarcusVLMA/latin-pcd-viewer) to see the web application used to explore point cloud images (.pcd) and intermediary results of this algorithm.

## Dependencies
To run this project you will need:
- [Point Cloud Library](https://pointclouds.org/)
- [CMake](https://cmake.org/) to build the output.

If you intend to generate the `.node` file, you will also need:
- Node 10 (I recommend using [nvm](https://github.com/nvm-sh/nvm) since this is a old node version).
- [CMake.js](https://github.com/cmake-js/cmake-js) to build using cmake.

## Building
### Regular C++ output
Now that you have installed Point Cloud Library and CMake, just get into `cmd/regular`:
```
ls cmd/regular
```
and build with cmake:
```
ccmake .
make
```
The executable output file `nosetip_finder` can be found inside of `cmd/regular/build` folder.

### To use with Node.js
You may want to use this algorithm with a web application, like I did in this [repository](https://github.com/MarcusVLMA/latin-pcd-viewer). A way to do this is generating the `.node` build, to use as a dependency in Node.js code.

As mencioned earlier, you will need Node.js 10. If you are using `nvm`, you can set Node version to 10 by running:
```
nvm install 10.23.0
nvm use 10
```
_Note that this will set your Node version only in the current terminal._

After that, install `cmake-js` globally with:
```
npm install -g cmake-js
```

Now, go into `cmd/nan-module`:
```
ls cmd/nan-module
```
And run:
```
npm run install
```
This will install any necessary dependencies and compile the code with `cmake-js`.

The output file `nosetip_finder.node` can be found inside of `cmd/nan-module/build/Release` folder.
## Parameters
For executing the algorithm, you need to provide this set of parameters **(in this order)**:
- 1 - Input filename - `STRING`
- 2 - Flexibilize thresholds - `BOOLEAN`
- 3 - Flexibilize crop - `BOOLEAN`
- 4 - Normal estimation size (radius size or k value) - `INTEGER`
- 5 - Normal estimation method - `radius` or `k` (as string)
- 6 - Minimum threshold of Gaussian Curvature - `DOUBLE`
- 7 - Maximum threshold of Shape Index - `DOUBLE` (between `-1` and `+1`)
- 8 - Minimum crop size - `FLOAT`
- 9 - Maximum crop size - `FLOAT`
- 10 - Minimum points to continue - `INTEGER`
- 11 - Removal radius of isolated points - `FLOAT`
- 12 - Threshold of points count of isolated points - `INTEGER`
- 13 - Search radius of nose tip - `INTEGER`

If using regular C++ build, pass this parameters inline, like: 
```
[project_path]/cmd/regular/build/nosetip_finder parameter1 parameter2 parameter3 [...]
```

If using `.node` dependency, pass this parameters like a regular function call: 
```js
const nosetip_finder = require('[path to nosetip_finder.node]');

const response = nosetip_finder.findNoseTip(
          parameter1,
          parameter2,
          parameter3,
          [...]
        );
```

### Regular C++ output exclusive parameters
If you are using the regular C++ build, you can use some other parameters. Note that, considering the order, these parameters should be set after the parameters listed on the last section.

**Note that the only optional parameter is the 16. All the others are required.**

Exclusive parameters for C++ regular build **(in this order)**:
- 14 - Ouput file to save the nosetip as a `.pcd` file - `STRING`
- 15 - Definition if you want to visualize the final and the intermediary results - Set this parameter as `visualizar` if you want to visualize or set as any other string if you do not want to visualize the results.
- 16 - A file with a known nosetip to verify if the algorithm result is a good nosetip - `STRING`
- 17 - A CSV file to save the processing results _(analyzed cloud, if the result is a good nosetip, execution time, known nosetip and the nosetip found)_ - `STRING`
- 18 - A CSV file where eventual errors will be written - `STRING`

Obs: If you don't pass the parameter 16 and parameter 15 is set as `visualizar`, you will be prompted by the terminal to input if the result is a good nosetip or not.