#let Acc = $bold(a)$
#let AccP = $bold(a_(p))$
#let TanDir = $bold(T)$
#let NorDir = $bold(N)$
#let BinDir = $bold(B)$
#let accTan = $a_(t)$
#let accNor = $a_(n)$
#let AccGra = $bold(g)$
#let SaccPx = $attach(a_(p_x), tl: cal(S))$
#let SaccPz = $attach(a_(p_z), tl: cal(S))$
#let Xs = $bold(x)_s$
#let Zs = $bold(z)_s$
#let Xb = $bold(x)_b$
#let Yb = $bold(y)_b$
#let Zb = $bold(z)_b$
#let Vel = $bold(v)$
#let DistX = $bold(delta)_x$
#let DistY = $bold(delta)_y$
#let BapxEst = $attach(a_(p_x)^("est"), tl: cal(B))$
#let span = $"span"$
#let Bfay = $attach(f_(a_y), tl:cal(B))$
#let VelFlu = $bold(v)_("flu")$
#let Velb = $bold(v)_b$

#let SideForce = $attach(bold(f)_(a_y), tl: cal(S))$
#let CY = $C_Y$
#let CL = $C_L$
#let CD = $C_D$

#set math.equation(numbering: "(1)")


= Aerodynamics
== Aerodynamic Forces
Typically, we can view the sideforce 
$
  CY = (partial CY) (partial beta) approx kappa(<0)
$
i.e.,
