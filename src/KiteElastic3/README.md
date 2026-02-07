# KiteElastic3
This module of LAKSA is similar to KiteElastic, but instead of using normalized units it operates with fully dimensional variables. It also features a closed-loop controller and a reworked structure to allow for code compiling. The compiled simulator is much faster than the bare Matlab version, but the required changes broke compatibility with other LAKSA models, so the common folder has been duplicated.

The simulator includes an aerodynamical model of the Rigid-Framed Delta (RFD) kite used by UC3M. The code may contain disabled references to other experimental models not yet included.
 
## Description
MATLAB source code and compiled function.

## Authors
This module was developed by:
- Francisco de los Ríos Navarrete <francisco.los@alumnos.uc3m.es>
- Iván Castro-Fernández <ivancastrofernandez95@gmail.com>

Based on KiteElastic and KiteAcrobat by:
- Gonzalo Sánchez-Arriaga <gonsanch@ing.uc3m.es>
- Alejandro Pastor-Rodríguez <alejandro.pastor@alumnos.uc3m.es>

The code was refactored to make it compatible with Matlab Coder by:
- Duc Nguyen <duc.nguyen@bristol.ac.uk>
- Kevin Yu <2003kevinyu@gmail.com>