import dwl
import os
import numpy as np
import time



class HS071(dwl.OptimizationModel):
  def __init__(self):
    dwl.OptimizationModel.__init__(self)
    self.setDimensionOfState(4)
    self.setDimensionOfConstraints(2)
    self.setNumberOfNonzeroJacobian(8)
    self.setNumberOfNonzeroHessian(10)


  def getStartingPoint(self, x):
    x[0] = 1.0
    x[1] = 5.0
    x[2] = 5.0
    x[3] = 1.0


  def evaluateBounds(self, x_l, x_u, g_l, g_u):
    # the variables have lower bounds of 1
    for i in range(0, 4):
      x_l[i] = 1.0

    # the variables have upper bounds of 5
    for i in range(0, 4):
      x_u[i] = 5.0
    # the first constraint g1 has a lower bound of 25
    g_l[0] = 25.

    # the first constraint g1 has NO upper bound, here we set it to 2e19.
    # Ipopt interprets any number greater than nlp_upper_bound_inf as 
    # infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
    # is 1e19 and can be changed through ipopt options.
    g_u[0] = 2e19

    # the second constraint g2 is an equality constraint, so we set the 
    # upper and lower bound to the same value
    g_l[1] = 40.
    g_u[1] = 40.



  def evaluateCosts(self, cost, x):
    cost[0] = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2]


  def evaluateCostGradient(self, grad_f, x):
    grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2])
    grad_f[1] = x[0] * x[3]
    grad_f[2] = x[0] * x[3] + 1
    grad_f[3] = x[0] * (x[0] + x[1] + x[2])


  def evaluateConstraints(self, g, x):
    g[0] = x[0] * x[1] * x[2] * x[3]
    g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]


  def evaluateConstraintJacobian(self, jac_values, rind, cind, x, flag):
    if flag:
      jrow = np.array([0, 0, 0, 0, 1, 1, 1, 1])
      jcol = np.array([0, 1, 2, 3, 0, 1, 2, 3])
      np.copyto(rind, jrow)
      np.copyto(cind, jcol)
    else:
      jac = np.array([x[1] * x[2] * x[3], 
                      x[0] * x[2] * x[3], 
                      x[0] * x[1] * x[3], 
                      x[0] * x[1] * x[2],
                      2 * x[0],
                      2 * x[1],
                      2 * x[2],
                      2 * x[3] ])
      np.copyto(jac_values, jac)


  def evaluateLagrangianHessian(self, hess_values, rind, cind, obj_factor, lagrange, x, flag):
    if flag:
      hrow = np.array([0, 1, 1, 2, 2, 2, 3, 3, 3, 3])
      hcol = np.array([0, 0, 1, 0, 1, 2, 0, 1, 2, 3])
      np.copyto(rind, hrow)
      np.copyto(cind, hcol)
    else:
      # Fill the objective portion
      hess_values[0] = obj_factor * (2 * x[3])
      hess_values[1] = obj_factor * (x[3])
      hess_values[2] = 0
      hess_values[3] = obj_factor * (x[3])
      hess_values[4] = 0
      hess_values[5] = 0
      hess_values[6] = obj_factor * (2 * x[0] + x[1] + x[2])
      hess_values[7] = obj_factor * (x[0])
      hess_values[8] = obj_factor * (x[0])
      hess_values[9] = 0

      # Fill the portion for the first constraint
      hess_values[1] += lagrange[0] * (x[2] * x[3])
      hess_values[3] += lagrange[0] * (x[1] * x[3])
      hess_values[4] += lagrange[0] * (x[0] * x[3])
      hess_values[6] += lagrange[0] * (x[1] * x[2])
      hess_values[7] += lagrange[0] * (x[0] * x[2])
      hess_values[8] += lagrange[0] * (x[0] * x[1])

      # Fill the portion for the second constraint
      hess_values[0] += lagrange[1] * 2
      hess_values[2] += lagrange[1] * 2
      hess_values[5] += lagrange[1] * 2
      hess_values[9] += lagrange[1] * 2




# import adolc
# class HS071_adolc(dwl.OptimizationModel):
#   def __init__(self):
#     dwl.OptimizationModel.__init__(self)
#     self.setDimensionOfState(4)
#     self.setDimensionOfConstraints(2)
#     self.setNumberOfNonzeroJacobian(8)
#     self.setNumberOfNonzeroHessian(10)
# 
#     # trace objective function
#     x0 = np.array([1.0, 5.0, 5.0, 1.0])
#     self.getStartingPoint(x0)
#     f = np.array([0.0])
#     adolc.trace_on(1)
#     ax = adolc.adouble(x0)
#     af = adolc.adouble(f)
#     adolc.independent(ax)
#     self.costFunction(af, ax)
#     adolc.dependent(af)
#     adolc.trace_off()
# 
#     # trace constraint function
#     adolc.trace_on(2)
#     ax = adolc.adouble(x0)
#     g = np.zeros(self.getDimensionOfConstraints())
#     ag = adolc.adouble(g)
#     adolc.independent(ax)
#     self.constraintFunction(ag, ax)
#     adolc.dependent(ag)
#     adolc.trace_off()
# 
#     # trace lagrangian function
#     adolc.trace_on(3)
#     ax = adolc.adouble(x0)
#     alagrange = adolc.adouble([1.,1.])
#     aobj_factor = adolc.adouble(1.)
#     adolc.independent(ax)
#     adolc.independent(alagrange)
#     adolc.independent(aobj_factor)
#     ay = self.eval_lagrangian(ax, alagrange, aobj_factor)
#     adolc.dependent(ay)
#     adolc.trace_off()
# 
#     x = np.array([1.0, 5.0, 5.0, 1.0])
#     hoptions = np.array([0,0],dtype=int)
#     hess_result = adolc.colpack.sparse_hess_no_repeat(3, x, hoptions)
#     self.hrow = np.asarray(hess_result[1], dtype=int)
#     self.hcol = np.asarray(hess_result[2], dtype=int)
#     self.hess_values = np.asarray(hess_result[3], dtype=float)
#     # need only upper left part of the Hessian
#     self.mask = np.where(self.hcol < 4)
# 
#   def getStartingPoint(self, x):
#     x[0] = 1.0
#     x[1] = 5.0
#     x[2] = 5.0
#     x[3] = 1.0
# 
#   def evaluateBounds(self, x_l, x_u, g_l, g_u):
#     # the variables have lower bounds of 1
#     for i in range(0, 4):
#       x_l[i] = 1.0
# 
#     # the variables have upper bounds of 5
#     for i in range(0, 4):
#       x_u[i] = 5.0
#     # the first constraint g1 has a lower bound of 25
#     g_l[0] = 25.
# 
#     # the first constraint g1 has NO upper bound, here we set it to 2e19.
#     # Ipopt interprets any number greater than nlp_upper_bound_inf as 
#     # infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
#     # is 1e19 and can be changed through ipopt options.
#     g_u[0] = 2e19
# 
#     # the second constraint g2 is an equality constraint, so we set the 
#     # upper and lower bound to the same value
#     g_l[1] = 40.
#     g_u[1] = 40.
# 
# 
#   def evaluateCosts(self, cost, x):
#     cost[0] = adolc.function(1, x)[0]
# 
#   def costFunction(self, cost, x):
#     cost[0] = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2]
# 
# 
#   def evaluateCostGradient(self, grad_f, x):
#     agrad_f = adolc.gradient(1, x)
#     np.copyto(grad_f, agrad_f)
# 
# 
#   def evaluateConstraints(self, g, x):
#     ag = adolc.function(2,x)
#     np.copyto(g, ag)
# 
#   def constraintFunction(self, g, x):
#     g[0] = x[0] * x[1] * x[2] * x[3]
#     g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]
# 
# 
#   def evaluateConstraintJacobian(self, jac_values, jrind, jcind, x, flag):
#     joptions = np.array([1,1,0,0],dtype=int)
#     jac_result = adolc.colpack.sparse_jac_no_repeat(2, x, joptions)
#     self.nnz = jac_result[0]
#     self.jrow = np.asarray(jac_result[1], dtype=int)
#     self.jcol = np.asarray(jac_result[2], dtype=int)
#     self.jac_values = np.asarray(jac_result[3], dtype=float)
#     if flag:
#       np.copyto(jrind, self.jrow)
#       np.copyto(jcind, self.jcol)
#     else:
#       result = adolc.colpack.sparse_jac_repeat(2, x, self.nnz, self.jrow, self.jcol, self.jac_values)
#       np.copyto(jac_values, result[3])
# 
# 
#   def eval_lagrangian(self, x, lagrange, obj_factor):
#     f = np.array([x[0]])
#     g = np.array([x[0], x[0]])
#     self.costFunction(f, x)
#     self.constraintFunction(g, x)
#     return obj_factor * f + np.dot(lagrange, g)
# 
#   def evaluateLagrangianHessian(self, hess_values, rind, cind, obj_factor, lagrange, x, flag):
#     if flag:
#       np.copyto(rind, self.hrow[self.mask])
#       np.copyto(cind, self.hcol[self.mask])
#     else:
#       x = np.hstack([x, lagrange, obj_factor])
#       result = adolc.colpack.sparse_hess_repeat(3, x, self.hrow, self.hcol, self.hess_values)
#       np.copyto(hess_values, result[3][self.mask])


if __name__ == '__main__':
  op = HS071()
#   op = HS071_adolc()
#  cmaes = dwl.cmaesSO()
#  cmaes.setFromConfigFile("../../config/cmaes_config.yaml")
#  op = op.__disown__()
#  cmaes.setOptimizationModel(op)
#  cmaes.init()
#  cmaes.compute()
#  print cmaes.getSolution().transpose()
#  del cmaes



  ipopt = dwl.IpoptNLP()
  fpath = str(os.path.dirname(os.path.abspath(__file__)))
  ipopt.setFromConfigFile(fpath + "/../../config/ipopt_config.yaml")
  op = op.__disown__()
  ipopt.setOptimizationModel(op)
  ipopt.init()
  startcputime = time.clock()
  ipopt.compute()
  cpu_duration = (time.clock() - startcputime) * 1000;
  print("Computation time: ", cpu_duration, "(milisecs, CPU time)")
  print ipopt.getSolution().transpose()
#  del ipopt

