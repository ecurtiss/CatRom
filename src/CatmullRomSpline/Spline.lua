local Squad = require(script.Parent.Squad)
local Types = require(script.Parent.Types)

local Spline = {}
Spline.__index = Spline

local DEFAULT_TENSION = 0
local DEFAULT_KNOT_PARAMETERIZATION = 0.5
local RIEMANN_STEP = 1e-3

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

	-- https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
	local t0 = 0
	local t1 = (p1 - p0).Magnitude ^ alpha + t0
	local t2 = (p2 - p1).Magnitude ^ alpha + t1
	local t3 = (p3 - p2).Magnitude ^ alpha + t2
	local m1 = (1 - tension) * (t2 - t1) * ((p1 - p0)/(t1 - t0) - (p2 - p0)/(t2 - t0) + (p2 - p1)/(t2 - t1))
	local m2 = (1 - tension) * (t2 - t1) * ((p2 - p1)/(t2 - t1) - (p3 - p1)/(t3 - t1) + (p3 - p2)/(t3 - t2))
	local a: Vector3 = 2 * (p1 - p2) + m1 + m2
	local b: Vector3 = 3 * (p2 - p1) - 2*m1 - m2
	local c: Vector3 = m1
	local d: Vector3 = p1

	local self = setmetatable({
		ClassName = className,
		Length = nil,

		-- knots
		k0 = k0,
		k1 = k1,
		k2 = k2,
		k3 = k3,

		-- coefficient vectors for position/velocity/acceleration polynomials
		a = a,
		b = b,
		c = c,
		d = d
	}, Spline)
	self.Length = self:SolveLength()

	return self
end

-- Internal methods
function Spline:_ToArcLengthAlpha(alpha: number)
	if alpha == 0 or alpha == 1 then return alpha end

	local length = self.Length
	local goalLength = length * alpha
	local runningLength = 0
	local lastPosition = self:SolvePosition(0)

	for i = 1, 1 / RIEMANN_STEP do
		i *= RIEMANN_STEP
		local thisPosition = self:SolvePosition(i)
		runningLength += (thisPosition - lastPosition).Magnitude
		--runningLength += ((thisPosition - lastPosition) / (Vector3.new(1, 1, 1) * RIEMANN_STEP)).Magnitude * RIEMANN_STEP
		--runningLength += (dp / DT).Magnitude * RIEMANN_STEP
		lastPosition = thisPosition
		if runningLength >= goalLength then
			return i -- FIX: Snaps every 1/RIEMANN_STEP
		end
	end
end

-- Methods
function Spline:SolvePosition(alpha: number)
	return self.a*alpha^3 + self.b*alpha^2 + self.c*alpha + self.d
end

function Spline:SolveVelocity(alpha: number)
	return 3*self.a*alpha^2 + 2*self.b*alpha + self.c
end

function Spline:SolveAcceleration(alpha: number)
	return 6*self.a*alpha + 2*self.b
end

function Spline:SolveUnitTangent(alpha: number)
	return self:SolveVelocity(alpha).Unit -- T(t) = r'(t) / |r'(t)|
end

function Spline:SolveUnitNormal(alpha: number)
	-- r is conventionally position
	-- p means prime (as in derivative)
	-- pp means prime prime (as in second derivative)
	local rp = self:SolveVelocity(alpha) -- r'(t)
	local rpp = self:SolveAcceleration(alpha) -- r''(t)
	-- N(t) = T'(t) / |T'(t)| =  (r'(t) / |r'(t)|)' / |(r'(t) / |r'(t)|)'|
	-- the following is equivalent to the rightmost expression
	return (rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude^3).Unit
end

function Spline:SolveUnitBinormal(alpha: number)
	return self:SolveUnitTangent(alpha):Cross(self:SolveUnitTangent(alpha))
end

function Spline:SolveCurvature(alpha: number)
	local rp = self:SolveVelocity(alpha)
	local rpp = self:SolveAcceleration(alpha)
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
	for i = 1, (b - a) / RIEMANN_STEP do
		i /= (b - a) / RIEMANN_STEP
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

-- Generate arc methods on runtime
do
	local arcMethods = {}

	for methodName, method in pairs(Spline) do
		if string.sub(methodName, 1, 5) == "Solve" then
			arcMethods["SolveArc" .. string.sub(methodName, 6)] = function(self, ...)
				local inputs = table.pack(...)
				for i, alpha in ipairs(inputs) do
					inputs[i] = self:_ToArcLengthAlpha(alpha)
				end
				return method(self, table.unpack(inputs))
			end
		end
	end

	for methodName, method in pairs(arcMethods) do
		Spline[methodName] = method
	end
end

return Spline