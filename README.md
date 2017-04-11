# smoothie-nuttx
A version of nuttx used by smoothie-v2

This is preconfigured to include all drivers that are required for smoothie to operate.

To build and export the library 

```
cd nuttx/tools
./configure.sh bambino-200e/smoothiedev
cd ..
make export
```

Then unzip the resulting nuttx-export.zip into the top of the directory you want to develop in.
