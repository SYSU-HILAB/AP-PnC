# ORIGIN

This is from https://github.com/autodiff/autodiff.git --depth=1 -b v1.1.2.

# Purpose
The package is separate from the main `autodiff` repository for two reason:
1. the official installation methods
    (1) conda install conda-devenv
    (2) source build 
both are heavy and not easy-to-migrate to a typical ROS2-C++ project.
2. the `autodiff` is header-only, so we can put core-part except test part of the original one here, for the downstreaming package to use at ease.


The usage can be seen in:
https://github.com/WarriorHanamy/AP-PnC/tree/main/src/tailsitter_df

