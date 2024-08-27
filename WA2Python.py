from asyncore import read
from sympy.parsing.mathematica import parse_mathematica
from sympy import Symbol, solve, Eq, S, solveset, nsolve
from sympy import *
from time import time

"""
from wolframclient.language import wl
from wolframclient.evaluation import WolframLanguageSession

# Start a Mathematica session
session = WolframLanguageSession()

# Call a Mathematica 
L0 =session.evaluate(wl.Symbol("L0"))
L1 =session.evaluate(wl.Symbol("L1"))
V0 =session.evaluate(wl.Symbol("V0"))
t =session.evaluate(wl.Symbol("t"))
x =session.evaluate(wl.Symbol("x"))
#eq1 = session.evaluate(wl.Expression("(L0-L1+L1*x^2/2)*9.8==(V0*(1-(x+t)^2/2))^2/2"))
#result = session.evaluate(wl.Solve(eq1, {x}))
#sample = session.evaluate(wl.RandomVariate(wl.NormalDistribution(0,1), 1e6))
#result = session.evaluate(wl.Differential("Derivative[1][x^2]"))
#result = session.evaluate(wl.Integrate("x^2", x))
result = session.evaluate(wl.D("x^2", x))
# Print the result
print(result)

# End the Mathematica session
session.terminate()
"""
"""

# using Teilor sequences cos(x) = 1 - x**2/2  in sympy library
start1 = time()
L0 = Symbol('L0')
L1 = Symbol('L1')
V0 = Symbol('V0')
x = Symbol('x')
t = Symbol('t')
equation = Eq((L0-L1+L1*x**2/2)*9.8, (V0*(1-(x+t)**2/2))**2/2)
#solution = solve(equation, x, chop=True)
#solution = solveset(equation, x, domain=S.Reals)
equation_s = equation.subs(L0, 0.180).subs(L1, 0.180).subs(V0, 0.5).subs(t, 0.3).simplify()
print(equation_s)
#print(solution[2])
#print('solving time : ', time()- start1)
#result = solution[2].subs(L0, 0.18).subs(L1, 0.18).subs(V0, 0.5).subs(t, 0.3)
result = solve(equation_s, x)
#result_s = result.simplify()
print('solving time : ', time()- start1)
print(result)

"""

"""
# trigonometry
L0 = Symbol('L0')
L1 = Symbol('L1')
V0 = Symbol('V0')
K = Symbol('K')
x = Symbol('x')
t = Symbol('t')
#equation = Eq((L0-L1* cos(x))*9.8, (V0*cos(x+t))**2/2)
#equation = Eq((0.18 - 0.13 * cos(x))*9.8, (0.1*cos(x+0.7))**2/2)
equation = Eq(K * x**2, (2 - x**2) * (1 - (x+t)**2/2)**2)
solution = solve(equation, x)
print(solution)
"""


t = Symbol('t')
x = Function('x')
K = Symbol('K')
C1 = Symbol('C1')
C2 = Symbol('C2')
x = C1*exp(-sqrt(K)*t) + C2*exp(sqrt(K)*t)
xp = x.diff(t)
xpp = xp.diff(t)

#result = integrate(tan(t), t)
# C1*exp(-sqrt(K)*t) + C2*exp(sqrt(K)*t)
#-C1*K*exp(-sqrt(K)*t) + C2*K*exp(sqrt(K)*t) 

#eq = xpp - K* x(t) 
#result = dsolve(eq, x(t))
#print(result)

#eq = C1*exp(-sqrt(K)*0) + C2*exp(sqrt(K)*0) - 0.0359
#result = solve(eq, C1)
#print(result)
print(xp)