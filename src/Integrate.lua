--[[
	Numerical integration functions.
	f: function to integrate
	a: lower bound
	b: upper bound

	TODO:
	Adaptive Simpson's method
	Newton-Cotes
	Gaussian quadrature
	Gauss-Kronrod quadrature
	Clenshaw-Curtis quadrature
	Romberg's method
]]

local Integrate = {}

-- Linear interpolation
local function lerp(a, b, t)
	return a * (1 - t) + b * t
end

-- Simpson's 1/3 rule
function Integrate.Simp13(f, a, b)
	return (b - a) / 6 * (f(a) + 4 * f((a + b) / 2) + f(b))
end

-- Simpson's 3/8 rule
function Integrate.Simp38(f, a, b)
	return (b - a) / 8 * (f(a) + 3 * f((2 * a + b) / 3) + 3 * f((a + 2 * b) / 3) + f(b))
end

-- Composite Simpson's 1/3 rule
function Integrate.Simp13Comp(f, a, b, steps)
	steps = steps or 10
	local stepsInv = 1 / steps
	local sum = f(a) * 0
	for t = 1, steps do
		t = t / steps
		sum = sum + Integrate.Simp13(f, lerp(a, b, t - stepsInv), lerp(a, b, t))
	end
	return sum
end

-- Composite Simpson's 3/8 rule
function Integrate.Simp38Comp(f, a, b, steps)
	steps = steps or 10
	local stepsInv = 1 / steps
	local sum = f(a) * 0
	for t = 1, steps do
		t = t / steps
		sum = sum + Integrate.Simp38(f, lerp(a, b, t - stepsInv), lerp(a, b, t))
	end
	return sum
end

-- Riemann sum left
function Integrate.RiemannSumLeft(f, a, b, steps)
	steps = steps or 10
	local dx = (b - a) / steps
	local sum = f(a) * 0
	for t = 0, steps - 1 do
		sum = sum + f(lerp(a, b, t / steps)) * dx
	end
	return sum
end

-- Riemann sum right
function Integrate.RiemannSumRight(f, a, b, steps)
	steps = steps or 10
	local dx = (b - a) / steps
	local sum = f(a) * 0
	for t = 1, steps do
		sum = sum + f(lerp(a, b, t / steps)) * dx
	end
	return sum
end

-- Riemann sum midpoint
function Integrate.RiemannSumMid(f, a, b, steps)
	steps = steps or 10
	local dx = (b - a) / steps
	local halfStep = 1 / 2 / steps
	local sum = f(a) * 0
	for t = 1, steps do
		sum = sum + f(lerp(a, b, t / steps - halfStep)) * dx
	end
	return sum
end

-- Trapezoidal rule
function Integrate.Trapezoid(f, a, b)
	return (b - a) * (f(a) + f(b)) / 2
end

-- Composite trapezoidal rule
function Integrate.TrapezoidComp(f, a, b, steps)
	steps = steps or 10
	local stepsInv = 1/ steps
	local sum = f(a) * 0
	for t = 0, steps - 1 do
		t = t / steps
		sum = sum + Integrate.Trapezoid(f, lerp(a, b, t), lerp(a, b, t + stepsInv))
	end
	return sum
end

-- -- test
-- local function f(x)
-- 	return math.cos(x)
-- end
-- local a = 0
-- local b = 10

-- local expected = Integrate.Simp38Comp(f, a, b, 1000)

-- for name, fn in pairs(Integrate) do
-- 	local t = os.clock()
-- 	local integral
-- 	for i = 1, 1e5 do
-- 		integral = fn(f, a, b, 100)
-- 	end
-- 	print(name, math.abs(integral - expected) / expected, os.clock() - t)
-- end

return Integrate