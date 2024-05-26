local GaussLegendre = require(script.Parent.GaussLegendre)
local Squad = require(script.Parent.Squad)

local MAX_NEWTON_ITERATIONS = 16
local EPSILON = 2e-7

local Spline = {}
Spline.__index = Spline

function Spline.new(trans0, trans1, trans2, trans3, alpha, tension)
	-- Parametrize
	-- https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
	local pos0 = trans0[1]
	local pos1 = trans1[1]
	local pos2 = trans2[1]
	local pos3 = trans3[1]

	local pos1_pos0 = pos1 - pos0
	local pos2_pos1 = pos2 - pos1
	local pos3_pos2 = pos3 - pos2

	local a, b, c
	local t0 = 0
	local t1 = pos1_pos0.Magnitude ^ alpha + t0
	local t2 = pos2_pos1.Magnitude ^ alpha + t1
	local t3 = pos3_pos2.Magnitude ^ alpha + t2

	local scalar = (1 - tension) * (t2 - t1)

	local m1 = scalar * (pos1_pos0 / (t1 - t0) - (pos2 - pos0) / (t2 - t0) + pos2_pos1 / (t2 - t1))
	local m2 = scalar * (pos2_pos1 / (t2 - t1) - (pos3 - pos1) / (t3 - t1) + pos3_pos2 / (t3 - t2))

	a = 2 * -pos2_pos1 + m1 + m2
	b = 3 * pos2_pos1 - 2 * m1 - m2
	c = m1
	
	local self = setmetatable({
		arcLengthParamsLUT = nil,
		length = nil,
		type = if trans0[2] then "CFrame" else typeof(pos0),

		-- Rotations (nil if type is Vector2 or Vector3)
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],

		-- Coefficient vectors for position/velocity/acceleration polynomials
		a = a,
		b = b,
		c = c,
		d = pos1,
	}, Spline)
	self.length = self:SolveLength()
	
	return self
end

function Spline.fromPoint(trans)
	local pos = trans[1]
	local zero = pos * 0

	return setmetatable({
		arcLengthParamsLUT = nil,
		length = 0,
		type = if trans[2] then "CFrame" else typeof(pos),

		rot0 = trans[2],

		a = zero,
		b = zero,
		c = zero,
		d = trans[1],
	}, Spline)
end

function Spline.fromLine(trans0, trans1, trans2, trans3)
	local pos1 = trans1[1]
	local zero = pos1 * 0
	local pos2_pos1 = trans2[1] - pos1

	return setmetatable({
		arcLengthParamsLUT = nil,
		length = pos2_pos1.Magnitude,
		type = if trans0[2] then "CFrame" else typeof(pos1),
		
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],

		a = zero,
		b = zero,
		c = pos2_pos1,
		d = pos1,
	}, Spline)
end

function Spline:SolvePosition(t: number)
	-- r(t) using Horner's method
	return self.d + t * (self.c + t * (self.b + t * self.a))
end

function Spline:SolveVelocity(t: number)
	-- r'(t) using Horner's method
	return self.c + t * (2 * self.b + t * 3 * self.a)
end

function Spline:SolveAcceleration(t: number)
	-- r''(t)
	return 6 * self.a * t + 2 * self.b
end

function Spline:SolveJerk()
	-- r'''(t)
	return 6 * self.a
end

function Spline:SolveTangent(t: number)
	-- T(t) = r'(t) / ||r'(t)||
	return self:SolveVelocity(t).Unit
end

function Spline:SolveNormal(t: number, unitSpeed: boolean?)
	if self.type == "Vector2" then
		local tangent = self:SolveTangent(t)
		return Vector2.new(-tangent.Y, tangent.X)
	elseif unitSpeed then
		return self:SolveAcceleration(t).Unit
	else
		-- N(t) = T'(t) / ||T'(t)||
		-- The return is equivalent to N(t) when the derivatives are carried out.
		-- In particular, the vector being normalized is T'(t) * ||r'(t)|| ^ 3, but
		-- the ||r'(t)|| ^ 3 scaling doesn't affect the result because we normalize
		-- it anyway. This scaled version is faster to compute.
		local vel = self:SolveVelocity(t) -- p for prime (1st derivative)
		local acc = self:SolveAcceleration(t) -- pp for prime prime (2nd derivative)
		return (acc * vel.Magnitude ^ 2 - vel * acc:Dot(vel)).Unit
	end
end

function Spline:SolveBinormal(t: number, unitSpeed: boolean?)
	assert(self.type ~= "Vector2", "SolveBinormal is undefined on Vector2 splines")

	-- T(t) x N(t)
	return self:SolveTangent(t):Cross(self:SolveNormal(t))
end

function Spline:SolveCurvature(t: number)
	local rp = self:SolveVelocity(t)
	local rpp = self:SolveAcceleration(t)
	local rpMag = rp.Magnitude
	local tangentp = rpp / rpMag - rp * rp:Dot(rpp) / rpMag ^ 3

	-- Curvature = ||T'(t)|| / ||r'(t)||
	-- N(t) is the direction of curvature
	local curvature = tangentp.Magnitude / rpMag
	local unitNormal = tangentp.Unit
	return curvature, unitNormal
end

function Spline:SolveCFrame(t: number)
	local position = self:SolvePosition(t)
	local tangent = self:SolveVelocity(t)

	if tangent.Magnitude == 0 then
		local rot = self.rot0
		if rot then
			return CFrame.new(position.X, position.Y, position.Z, rot[2], rot[3], rot[4], rot[1])
		else
			return CFrame.new(position)
		end
	end

	return CFrame.lookAlong(position, tangent)
end

function Spline:SolveRotCFrame(t: number)
	local rot0 = self.rot0
	if rot0 then -- CFrameCatRom
		local rot1 = self.rot1
		if rot1 then
			local pos = self:SolvePosition(t)
			local qw, qx, qy, qz = Squad(rot0, rot1, self.rot2, self.rot3, t)
			return CFrame.new(pos.X, pos.Y, pos.Z, qx, qy, qz, qw)
		else -- 1 point
			return self:SolveCFrame(t)
		end
	else -- VectorCatRom
		return self:SolveCFrame(t)
	end
end

function Spline:SolveLength(a: number?, b: number?)
	a = a or 0
	b = b or 1

	if a == 0 and b == 1 and self.length then
		return self.length
	end

	return GaussLegendre.Ten(function(x)
		return self:SolveVelocity(x).Magnitude
	end, a, b)
end

-- Reparametrizes s in terms of arc length, i.e., returns the input t that
-- yields the point s of the way along the spline
function Spline:Reparametrize(s: number)
	if s == 0 or s == 1 then
		return s
	elseif self.length == 0 then
		return 0
	end

	local arcLengthParamsLUT = self.arcLengthParamsLUT
	if arcLengthParamsLUT then
		local numIntervals = #self.arcLengthParamsLUT - 1
		local intervalIndex = math.floor(s * numIntervals) + 1
		local t = s * numIntervals - intervalIndex + 1
		return arcLengthParamsLUT[intervalIndex] * (1 - t)
			 + arcLengthParamsLUT[intervalIndex + 1] * t
	else
		return self:_ReparametrizeHybrid(s)
	end
end

-- Performs the actual arc length parametrization
-- s = \int_{0}^{t} ||r'(t)||dt = F(t) - F(0) = F(t)
-- where t is solved as the root-finding problem F(t) - s = 0.
function Spline:_ReparametrizeHybrid(s: number)
	if s == 0 or s == 1 then
		return s
	elseif self.length == 0 then
		return 0
	end

	-- Hybrid of Newton's method and bisection
	-- https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
	local integrand = function(x)
		return self:SolveVelocity(x).Magnitude
	end

	local t = s
	local lower = 0
	local upper = 1

	for _ = 1, MAX_NEWTON_ITERATIONS do
		local f =  GaussLegendre.Ten(integrand, 0, t) / self.length - s
		if math.abs(f) < EPSILON then
			return t
		end

		local g = self:SolveVelocity(t).Magnitude / self.length
		local candidate = t - f / g

		if f > 0 then
			-- Solution is below the current t
			upper = t
			t = candidate <= lower and (upper + lower) / 2 or candidate
		else
			-- Solution is above the current t
			lower = t
			t = candidate >= upper and (upper + lower) / 2 or candidate
		end
	end

	warn("Failed to reparametrize; falling back to input")
	return s
end

-- Precomputes a lookup table of numIntervals + 1 evenly spaced arc length
-- parameters that are used to piecewise linearly interpolate the real
-- parametrization function
function Spline:_PrecomputeArcLengthParams(numIntervals: number)
	if self.length == 0 then
		return
	end

	local arcLengthParamsLUT = table.create(numIntervals + 1)
	arcLengthParamsLUT[1] = 0
	arcLengthParamsLUT[numIntervals + 1] = 1
	for i = 2, numIntervals do
		arcLengthParamsLUT[i] = self:_ReparametrizeHybrid((i - 1) / numIntervals)
	end

	self.arcLengthParamsLUT = arcLengthParamsLUT
end

return Spline