local FuzzyEq = require(script.Parent.FuzzyEq)
local Squad = require(script.Parent.Squad)
local Types = require(script.Parent.Types)

local Spline = {}
Spline.__index = Spline

local DEFAULT_TENSION = 0
local DEFAULT_KNOT_PARAMETERIZATION = 0.5
local RIEMANN_STEPS = 1000 -- Number of samples for calculating arc length

-- Type checking
local tUnitInterval = Types.tUnitInterval
local tOptionalUnitInterval = Types.tOptionalUnitInterval
local tKnots = Types.tKnots

-- Constructor
function Spline.new(k0: Types.Knot, k1: Types.Knot, k2: Types.Knot, k3: Types.Knot, alpha: number?, tension: number?)
	assert(tOptionalUnitInterval(alpha))
	assert(tKnots(k0, k1, k2, k3))

	alpha = alpha or DEFAULT_KNOT_PARAMETERIZATION
	tension = tension or DEFAULT_TENSION
	
	local p0, p1, p2, p3
	local className
	if typeof(k0) == "Vector3" or typeof(k0) == "Vector2" then
		p0, p1, p2, p3 = k0, k1, k2, k3
		className = "VectorSpline"
	elseif typeof(k0) == "CFrame" then
		p0, p1, p2, p3 = k0.Position, k1.Position, k2.Position, k3.Position
		className = "CFrameSpline"
	end

	local a, b, c
	if FuzzyEq(p1, p2) then
		a = p1 * 0
		b = a
		c = a
	elseif FuzzyEq(p0, p1) or FuzzyEq(p2, p3) then
		a = p1 * 0
		b = a
		c = p2 - p1
	else
		-- https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
		local t0 = 0
		local t1 = (p1 - p0).Magnitude ^ alpha + t0
		local t2 = (p2 - p1).Magnitude ^ alpha + t1
		local t3 = (p3 - p2).Magnitude ^ alpha + t2
		local m1 = (1 - tension) * (t2 - t1) * ((p1 - p0)/(t1 - t0) - (p2 - p0)/(t2 - t0) + (p2 - p1)/(t2 - t1))
		local m2 = (1 - tension) * (t2 - t1) * ((p2 - p1)/(t2 - t1) - (p3 - p1)/(t3 - t1) + (p3 - p2)/(t3 - t2))
		a = 2 * (p1 - p2) + m1 + m2
		b = 3 * (p2 - p1) - 2 * m1 - m2
		c = m1
	end


	local self = setmetatable({
		ClassName = className,
		Length = nil,
		RiemannSteps = RIEMANN_STEPS,

		-- knots
		k0 = k0,
		k1 = k1,
		k2 = k2,
		k3 = k3,

		-- coefficient vectors for position/velocity/acceleration polynomials
		a = a,
		b = b,
		c = c,
		d = p1
	}, Spline)
	self.Length = self:SolveLength()

	return self
end

-- Internal methods
function Spline:_Reparameterize(alpha: number)
	if alpha == 0 or alpha == 1 then return alpha end

	local goal = self.Length * alpha
	local length = 0
	local lastPos = self:SolvePosition(0)

	for i = 1, RIEMANN_STEPS do
		i /= RIEMANN_STEPS

		local pos = self:SolvePosition(i)
		length += (pos - lastPos).Magnitude
		lastPos = pos

		if length > goal then
			return i
		end
	end

	error("Failed to reparameterize")
end

-- Methods
function Spline:SolvePosition(alpha: number)
	return self.a*alpha^3 + self.b*alpha^2 + self.c*alpha + self.d
end

function Spline:SolveVelocity(alpha: number)
	return 3*self.a*alpha^2 + 2*self.b*alpha + self.c -- dr / dt
end

function Spline:SolveAcceleration(alpha: number)
	return 6*self.a*alpha + 2*self.b -- d^2r / dt^2
end

function Spline:SolveTangent(alpha: number)
	return self:SolveVelocity(alpha).Unit -- T(t) = r'(t) / |r'(t)|
end

function Spline:SolveNormal(alpha: number)
	-- r is conventionally position
	-- p means prime (as in derivative)
	-- pp means prime prime (as in second derivative)
	local rp = self:SolveVelocity(alpha) -- r'(t)
	local rpp = self:SolveAcceleration(alpha) -- r''(t)
	-- N(t) = T'(t) / |T'(t)| =  (r'(t) / |r'(t)|)' / |(r'(t) / |r'(t)|)'|
	-- the following is equivalent to the rightmost expression
	return (rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude^3).Unit
end

function Spline:SolveBinormal(alpha: number)
	return self:SolveTangent(alpha):Cross(self:SolveNormal(alpha))
end

function Spline:SolveCurvature(alpha: number)
	local rp = self:SolveVelocity(alpha)
	local rpp = self:SolveAcceleration(alpha)
	-- non-unitized normal
	local tangentp = rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude^3

	local curvature = tangentp.Magnitude / rp.Magnitude
	local unitNormal = tangentp.Unit
	
	return curvature, unitNormal
end

function Spline:SolveCFrame(alpha: number)
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveVelocity(alpha)
	return CFrame.lookAt(position, position + tangent)
end

function Spline:SolveLength(a: number?, b: number?)
	assert(tOptionalUnitInterval(a))
	assert(tOptionalUnitInterval(b))
	a = a or 0
	b = b or 1

	local length = 0
	local lastPosition = self:SolvePosition(a)
	local steps = (b - a) * RIEMANN_STEPS
	for i = 1, steps do
		i /= steps
		local thisPosition = self:SolvePosition(a + (b - a) * i)
		length += (thisPosition - lastPosition).Magnitude
		--length += ((thisPosition - lastPosition) / (Vector3.new(1, 1, 1) * RIEMANN_STEP)).Magnitude * RIEMANN_STEP
		lastPosition = thisPosition
		-- equivalent to (dp / dt).Magnitude * RIEMANN_STEP
	end
	return length
end

function Spline:SolveRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))

	local cf0, cf1, cf2, cf3 do
		if self.ClassName == "CFrameSpline" then
			cf0, cf1, cf2, cf3 = self.k0, self.k1, self.k2, self.k3
		elseif self.ClassName == "VectorSpline" then
			cf0, cf1, cf2, cf3 = CFrame.new(self.k0), CFrame.new(self.k1), CFrame.new(self.k2), CFrame.new(self.k3)
		end
	end

	local position = self:SolvePosition(alpha)
	local tangent = self:SolveVelocity(alpha)
	local qw, qx, qy, qz = Squad(cf0, cf1, cf2, cf3, alpha)
	local quaternionToCFrame = CFrame.new(0, 0, 0, qx, qy, qz, qw)
	return CFrame.lookAt(position, position + tangent, quaternionToCFrame.UpVector)
end

---- START GENERATED METHODS
function Spline:SolveUniformPosition(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolvePosition(alpha)
end
function Spline:SolveUniformVelocity(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveVelocity(alpha)
end
function Spline:SolveUniformAcceleration(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveAcceleration(alpha)
end
function Spline:SolveUniformTangent(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveTangent(alpha)
end
function Spline:SolveUniformNormal(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveNormal(alpha)
end
function Spline:SolveUniformBinormal(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveBinormal(alpha)
end
function Spline:SolveUniformCurvature(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveCurvature(alpha)
end
function Spline:SolveUniformCFrame(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveCFrame(alpha)
end
function Spline:SolveUniformLength(a: number?, b: number?)
	a = self:_Reparameterize(a)
	b = self:_Reparameterize(b)
	return self:SolveLength(a, b)
end
function Spline:SolveUniformRotCFrame(alpha: number)
	alpha = self:_Reparameterize(alpha)
	return self:SolveRotCFrame(alpha)
end
---- END GENERATED METHODS

return Spline