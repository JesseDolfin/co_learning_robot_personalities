"""
@author Jesse Haviland for <https://github.com/petercorke/robotics-toolbox-python>
modified by: Leandro de Souza Rosa <l.desouzarosa@tudelft.nl>
"""

import numpy as np
from collections import namedtuple

# from roboticstoolbox.tools.null import null
import roboticstoolbox as rtb
from spatialmath import base
from spatialmath import SE3
import scipy.optimize as opt
import math
from roboticstoolbox.tools.p_servo import p_servo

iksol = namedtuple("IKsolution", "q, success, reason, iterations, residual")

try:
    import qpsolvers as qp

    _qp = True
except ImportError:  # pragma nocover
    _qp = False

def _angle_axis(T, Td):
    d = base.transl(Td) - base.transl(T)
    R = base.t2r(Td) @ base.t2r(T).T
    li = np.r_[R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]

    if base.iszerovec(li):
        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = base.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    return np.r_[d, a]

def _ikine_min(
    self,
    T,
    q0=None,
    qlim=True,
    ilimit=1000,
    tol=1e-16,
    method=None,
    stiffness=0,
    costfun=None,
    options={},
    end=None,
):
    r"""
    Inverse kinematics by optimization with joint limits (Robot superclass)
                                                                                                
    :param T: The desired end-effector pose or pose trajectory
    :type T: SE3
    :param q0: initial joint configuration (default all zeros)
    :type q0: ndarray(n)
    :param qlim: enforce joint limits
    :type qlim: bool
    :param ilimit: Iteration limit (default 1000)
    :type ilimit: int
    :param tol: Tolerance (default 1e-16)
    :type tol: tol
    :param method: minimization method to use
    :type method: str
    :param stiffness: Stiffness used to impose a smoothness contraint on
        joint angles, useful when n is large (default 0)
    :type stiffness: float
    :param costfun: User supplied cost term, optional
    :type costfun: callable
    :return: inverse kinematic solution
    :rtype: named tuple
                                                                                                
    ``sol = robot.ikine_min(T)`` are the joint coordinates (n)
    corresponding to the robot end-effector pose T which is an SE3 object.
    The return value ``sol`` is a named tuple with elements:
                                                                                                
    ============    ==========  ============================================================
    Element         Type        Description
    ============    ==========  ============================================================
    ``q``           ndarray(n)  joint coordinates in units of radians or metres, or ``None``
    ``success``     bool        whether a solution was found
    ``reason``      str         reason for the failure
    ``iterations``  int         number of iterations
    ``residual``    float       final value of cost function
    ============    ==========  ============================================================
                                                                                                
    **Minimization method**:
                                                                                                
    By default this method uses:
                                                                                                
    - the Scipy ``SLSQP`` (Sequential Least Squares Programming) minimizer
      for the case of no joint limits
    - the Scipy ``trust-constr`` minimizer for the case with joint limits.
      This gives good results but is very slow.  An alternative is
      ``L-BFGS-B`` (Broyden–Fletcher–Goldfarb–Shanno) but for redundant
      robots can sometimes give poor results, pushing against the joint
      limits when there is no need to.
                                                                                                
    In both case the function to be minimized is the squared norm of a
    vector :math:`[d,a]` with components respectively the
    translation error and rotation error in Euler vector form, between the
    desired pose and the current estimate obtained by inverse kinematics.
                                                                                                
    **Additional cost terms**:
                                                                                                
    This method supports two additional costs:
                                                                                                
    - ``stiffness`` imposes a penalty on joint variation
      :math:`\sum_{j=1}^N (q_j - q_{j-1})^2` which tends to keep the
      arm straight
    - ``costfun`` add a cost given by a user-specified function
      ``costfun(q)``
                                                                                                
    **Trajectory operation**:
                                                                                                
    If ``len(T) > 1`` it is considered to be a trajectory, and the result
    is a list of named tuples such that ``sol[k]`` corresponds to
    ``T[k]``. The initial estimate of q for each time step is taken as the
    solution from the previous time step.
                                                                                                
    .. note::
                                                                                                
        - See `Toolbox kinematics wiki page
            <https://github.com/petercorke/robotics-toolbox-python/wiki/Kinematics>`_
        - Uses ``SciPy.minimize`` with bounds.
        - Joint limits are considered in this solution.
        - Can be used for robots with arbitrary degrees of freedom.
        - The inverse kinematic solution is generally not unique, and
          depends on the initial guess ``q0``.
        - The default value of ``q0`` is zero which is a poor choice for
          most manipulators since it often corresponds to a
          kinematic singularity.
        - Such a solution is completely general, though much less
          efficient than analytic inverse kinematic solutions derived
          symbolically.
        - The objective function (error) is
          :math:`\sum \left( (\mat{T}^{-1} \cal{K}(\vec{q}) - \mat{1} ) \mat{\Omega} \right)^2`
          where :math:`\mat{\Omega}` is a diagonal matrix.
        - Joint offsets, if defined, are accounted for in the solution.
                                                                                                
    .. warning::
                                                                                                
        - The objective function is rather uncommon.
        - Order of magnitude slower than ``ikine_LM`` or ``ikine_LMS``, it
          uses a scalar cost-function and does not provide a Jacobian.
                                                                                                
    :author: Bryan Moutrie, for RTB-MATLAB
                                                                                                
    :seealso: :func:`ikine_LM`, :func:`ikine_LMS`, :func:`ikine_unc`,
        :func:`ikine_min`
                                                                                                
    """  # noqa E501
    
    if not isinstance(T, SE3):
        raise TypeError("argument must be SE3")
    
    if isinstance(self, rtb.DHRobot):
        end = None
    
    if q0 is None:
        q0 = np.zeros((self.n))
    else:
        q0 = base.getvector(q0, self.n)
    
    solutions = []
    
    if self.reach == 0:
        wr = 1
    else:
        wr = 1 / self.reach
    
    weight = np.r_[wr, wr, wr, 1, 1, 1]
    
    optdict = {"maxiter": ilimit}
    if options is not None and isinstance(options, dict):
        optdict.update(options)
    else:
        raise ValueError("options must be a dict")
    
    if qlim:
        # dealing with joint limits
        bounds = opt.Bounds(self.qlim[0, :], self.qlim[1, :])
        
        if method is None:
            method = "trust-constr"
    else:
        # no joint limits
        if method is None:
            method = "SLSQP"
        bounds = None
    
    def cost(q, T, weight, costfun, stiffness):
        # T, weight, costfun, stiffness = args
        e = _angle_axis(self.fkine(q, end=end).A, T) * weight
        E = (e**2).sum()
        
        if stiffness > 0:
            # Enforce a continuity constraint on joints, minimum bend
            E += np.sum(np.diff(q) ** 2) * stiffness
        
        if costfun is not None:
            E += (e**2).sum() + costfun(q)
        
        return E
    
    for Tk in T:
        res = opt.minimize(
            cost,
            q0,
            args=(Tk.A, weight, costfun, stiffness),
            bounds=bounds,
            method=method,
            tol=tol,
            options=options,
        )
        
        # trust-constr seems to work better than L-BFGS-B which often
        # runs a joint up against its limit and terminates with position
        # error.
        # but 'truts-constr' is 5x slower
        
        solution = iksol(res.x, res.success, res.message, res.nit, res.fun)
        solutions.append(solution)
        q0 = res.x  # use this solution as initial estimate for next time
    
    if len(T) == 1:
        return solutions[0]
    else:
        return solutions

# --------------------------------------------------------------------- #
