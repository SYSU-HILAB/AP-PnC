# One reference point: tracking reference only.
# No body rate (the NMPC rate cost weight is zero) and no control/u fields
# (the controller owns its own actuation reference).
float64[3] pos   # world position [m]
float64[3] vel   # world velocity [m/s]
float64[3] yb    # desired body y-axis, quad FLU
