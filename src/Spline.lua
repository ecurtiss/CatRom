local GaussLegendre = require(script.Parent.GaussLegendre)
local Squad = require(script.Parent.Squad)

local MAX_NEWTON_ITERATIONS = 16

local Spline = {}
Spline.__index = Spline

function Spline.new(trans0, trans1, trans2, trans3, alpha, tension)
	-- Parameterize
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
		-- Rotations (nil if VectorCatRom)
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],
		
		length = nil,

		-- Coefficient vectors for position/velocity/acceleration polynomials
		a = a,
		b = b,
		c = c,
		d = pos1
	}, Spline)
	self.length = self:SolveLength()
	
	return self
end

function Spline.fromPoint(trans)
	local pos = trans[1]
	local zero = pos * 0

	return setmetatable({
		rot0 = trans[2],

		length = 0,

		a = zero,
		b = zero,
		c = zero,
		d = trans[1]
	}, Spline)
end

function Spline.fromLine(trans0, trans1, trans2, trans3)
	local pos1 = trans1[1]
	local zero = pos1 * 0
	local pos2_pos1 = trans2[1] - pos1

	return setmetatable({
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],

		length = pos2_pos1.Magnitude,

		a = zero,
		b = zero,
		c = pos2_pos1,
		d = pos1
	}, Spline)
end

-- Methods
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

function Spline:SolveTangent(t: number)
	-- T(t) = r'(t) / ||r'(t)||
	return self:SolveVelocity(t).Unit
end

function Spline:SolveNormal(t: number)
	-- N(t) = T'(t) / ||T'(t)||
	-- The return is equivalent to N(t) when the derivatives are carried out.
	-- In particular, the vector being unitized is T'(t) * ||r'(t)|| ^ 3, but
	-- the ||r'(t)|| ^ 3 scaling doesn't affect the result because we unitize it
	-- anyway. This scaled version is faster to compute.
	local rp = self:SolveVelocity(t) -- p for prime (1st deriv.)
	local rpp = self:SolveAcceleration(t) -- pp for prime prime (2nd deriv.)
	return (rpp * rp.Magnitude ^ 2 - rp * rpp:Dot(rp)).Unit
end

function Spline:SolveBinormal(t: number)
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

	return CFrame.lookAt(position, position + tangent)
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

function Spline:Reparameterize(alpha: number)
	-- Newton's method
	if alpha == 0 or alpha == 1 then
		return alpha
	end

	local function f(b)
		return GaussLegendre.Five(function(x)
			return self:SolveVelocity(x).Magnitude
		end, 0, b) / self.length - alpha
	end

	local iterations = 0
	local x = alpha
	local y = f(x)
	while math.abs(y) > 1e-4 and iterations < MAX_NEWTON_ITERATIONS do
		iterations += 1
		local denom = self:SolveVelocity(x).Magnitude / self.length
		if denom == 0 then
			error("Divide by 0 error")
		end
		x -= y / denom
		y = f(x)
	end

	if iterations >= MAX_NEWTON_ITERATIONS then
		error("Failed to reparameterize spline")
	end

	return x
end

---- START GENERATED METHODS
function Spline:SolveUniformPosition(t: number)
	return self:SolvePosition(self:Reparameterize(t))
end
function Spline:SolveUniformVelocity(t: number)
	return self:SolveVelocity(self:Reparameterize(t))
end
function Spline:SolveUniformAcceleration(t: number)
	return self:SolveAcceleration(self:Reparameterize(t))
end
function Spline:SolveUniformTangent(t: number)
	return self:SolveTangent(self:Reparameterize(t))
end
function Spline:SolveUniformNormal(t: number)
	return self:SolveNormal(self:Reparameterize(t))
end
function Spline:SolveUniformBinormal(t: number)
	return self:SolveBinormal(self:Reparameterize(t))
end
function Spline:SolveUniformCurvature(t: number)
	return self:SolveCurvature(self:Reparameterize(t))
end
function Spline:SolveUniformCFrame(t: number)
	return self:SolveCFrame(self:Reparameterize(t))
end
function Spline:SolveUniformRotCFrame(t: number)
	return self:SolveRotCFrame(self:Reparameterize(t))
end
function Spline:SolveUniformLength(a: number?, b: number?)
	return self:SolveLength(self:Reparameterize(a), self:Reparameterize(b))
end
---- END GENERATED METHODS

return Spline