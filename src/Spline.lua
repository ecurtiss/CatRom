local GaussLegendre = require(script.Parent.GaussLegendre)
local Squad = require(script.Parent.Squad)

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
function Spline:SolvePosition(alpha: number)
	-- r(t) using Horner's method
	return self.d + alpha * (self.c + alpha * (self.b + alpha * self.a))
end

function Spline:SolveVelocity(alpha: number)
	-- r'(t) using Horner's method
	return self.c + alpha * (2 * self.b + alpha * 3 * self.a)
end

function Spline:SolveAcceleration(alpha: number)
	-- r''(t)
	return 6 * self.a * alpha + 2 * self.b
end

function Spline:SolveTangent(alpha: number)
	-- T(t) = r'(t) / ||r'(t)||
	return self:SolveVelocity(alpha).Unit
end

function Spline:SolveNormal(alpha: number)
	-- N(t) = T'(t) / ||T'(t)||
	-- The return is equivalent to N(t) when the derivatives are carried out.
	-- In particular, the vector being unitized is T'(t) * ||r'(t)|| ^ 3, but
	-- the ||r'(t)|| ^ 3 scaling doesn't affect the result because we unitize it
	-- anyway. This scaled version is simply faster to compute.
	local rp = self:SolveVelocity(alpha) -- p for prime (1st deriv.)
	local rpp = self:SolveAcceleration(alpha) -- pp for prime prime (2nd deriv.)
	return (rpp * rp.Magnitude ^ 2 - rp * rpp:Dot(rp)).Unit
end

function Spline:SolveBinormal(alpha: number)
	-- T(t) x N(t)
	return self:SolveTangent(alpha):Cross(self:SolveNormal(alpha))
end

function Spline:SolveCurvature(alpha: number)
	local rp = self:SolveVelocity(alpha)
	local rpp = self:SolveAcceleration(alpha)
	local rpMag = rp.Magnitude
	local tangentp = rpp / rpMag - rp * rp:Dot(rpp) / rpMag ^ 3

	-- Curvature = ||T'(t)|| / ||r'(t)||
	-- N(t) is the direction of curvature
	local curvature = tangentp.Magnitude / rpMag
	local unitNormal = tangentp.Unit
	return curvature, unitNormal
end

function Spline:SolveCFrame(alpha: number)
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveVelocity(alpha)

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

function Spline:SolveRotCFrame(alpha: number)
	local rot0 = self.rot0
	if rot0 then -- CFrameCatRom
		local rot1 = self.rot1
		if rot1 then
			local pos = self:SolvePosition(alpha)
			local qw, qx, qy, qz = Squad(rot0, rot1, self.rot2, self.rot3, alpha)
			return CFrame.new(pos.X, pos.Y, pos.Z, qx, qy, qz, qw)
		else -- 1 point
			return self:SolveCFrame(alpha)
		end
	else -- VectorCatRom
		return self:SolveCFrame(alpha)
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

-- TODO: Reparameterize (this will allow for Uniform methods)

---- START GENERATED METHODS
---- END GENERATED METHODS

return Spline