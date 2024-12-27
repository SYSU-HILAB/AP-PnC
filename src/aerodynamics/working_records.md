# Aerodynamics Development Working Records

## To-Do List

### 1. Aerodynamics Visualization
- [ ] Create visualization script for all Angles of Attack
- [ ] Plot force coefficients vs AoA
- [ ] Plot moment coefficients vs AoA
- [ ] Add interactive visualization features
- [ ] Export plots in publication quality

### 2. Gazebo Integration
- [ ] Create ROS2 plugin interface
- [ ] Implement aerodynamics force calculation in plugin
- [ ] Add configuration file loading in Gazebo
- [ ] Test real-time performance
- [ ] Validate forces in simulation

<!-- /* The data is coming from the following paper:
@article{MaAero,
  author    = {Z. Ma and E. J. J. Smeur and G. C. H. E. de Croon},
  title     = {Wind tunnel tests of a wing at all angles of attack},
  journal   = {Int. J. Micro Air Veh.},
  year      = {2022},
  volume    = {14},
  pages     = {17568293221110931},
  month     = {Jan},
  doi       = {10.1177/17568293221110931},
} */ -->
### 3. Matlab data to BsplineAerodynamics
- [x] Convert data to CSV format
- [x] Implement B-spline fitting in Python
- [x] Configure and tune parameters in Python
- [x] Dump parameters in Python and read them in C++
- [ ] Implement B-spline basis functions and create interpolated fitting functions in C++
- [ ] Perform testing in C++

### 4. SQ Config Reader Assembly
- [x] the encapsulation of the SQ Config Reader is not good
- [x] the APIs design of aerodynamics is weird.

### 5. Comparative Analysis Plots
- [ ] Collect all aerodynamics results
- [ ] Design publication-quality plot layout
- [ ] Create comparison plots with error bars
- [ ] Add statistical analysis
- [ ] Generate IEEE-standard figures

## Progress Notes
- Started: [Date]
- Last Updated: [Date]

## References
- IEEE plotting guidelines
- Gazebo plugin documentation
- ROS2 integration guides 