# ESKF-2D-localization
a very simple implementation for ESKF

## Dependencies
* OpenCV3
* Eigen3

## Compile
```bash
mkdir build
cd build
cmake ..
make
```

## Usage
```bash
./eskf_locate
```

![ScreenShot](Screenshot.png)

* use mouse to drag the car (in left part view).
* long press `Q` or `W` to rotate the car.
* press `Esc` to exit.
