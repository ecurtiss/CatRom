local FuzzyEq = require(script.Parent.FuzzyEq)
local Integrate = require(script.Parent.Integrate)
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

	if FuzzyEq(pos1, pos2) then
		-- The inner control points are at the same position. In order to avoid
		-- NaNs, we simply know that the position must be pos1.
		a = pos1 * 0 -- Either the zero Vector2 or the zero Vector3
		b = a
		c = a
	elseif FuzzyEq(pos0, pos1) or FuzzyEq(pos2, pos3) then
		-- If either of the outside control points are equal, there is no
		-- defined way to handle this curve. Instead, we default to linear
		-- interpolation between the inner control points.
		-- FIX: Try a quadratic Catmull-Rom?
		a = pos1 * 0
		b = a
		c = pos2_pos1
	else
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
	end
	
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

-- Methods
function Spline:SolvePosition(alpha: number)
	-- Horner's method applied to
	-- self.a * alpha ^ 3 + self.b * alpha ^ 2 + self.c * alpha + self.d
	return self.d + alpha * (self.c + alpha * (self.b + alpha * self.a))
end

function Spline:SolveVelocity(alpha: number)
	-- dr / dt
	-- Horner's method applied to
	-- 3 * self.a * alpha ^ 2 + 2 * self.b * alpha + self.c
	return self.c + alpha * (2 * self.b + alpha * 3 * self.a)
end

function Spline:SolveAcceleration(alpha: number)
	-- d^2r / dt^2
	return 6 * self.a * alpha + 2 * self.b
end

function Spline:SolveTangent(alpha: number)
	-- T(t) = r'(t) / ||r'(t)||
	return self:SolveVelocity(alpha).Unit
end

function Spline:SolveNormal(alpha: number)
	-- N(t) = T'(t) / |T'(t)|
	-- The return statement is equivalent to T'(t) / |T'(t)| when the
	-- derivatives are carried out.
	local tan = self:SolveTangent(alpha)
	local acc = self:SolveAcceleration(alpha)
	-- return (rp * (rpp:Dot(rp)) - rpp * rp.Magnitude ^ 2).Unit
	return (tan * acc:Dot(tan) - acc).Unit
end

function Spline:SolveBinormal(alpha: number)
	return self:SolveTangent(alpha):Cross(self:SolveNormal(alpha))
end

function Spline:SolveCurvature(alpha: number)
	local rp = self:SolveVelocity(alpha)
	local rpp = self:SolveAcceleration(alpha)
	-- non-unitized normal
	local tangentp = rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude ^ 3

	local curvature = tangentp.Magnitude / rp.Magnitude
	local unitNormal = tangentp.Unit

	return curvature, unitNormal
end

function Spline:SolveCFrame(alpha: number)
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveVelocity(alpha)
	return CFrame.lookAt(position, position + tangent)
end

function Spline:SolveRotCFrame(alpha: number)
	local rot0 = self.rot0
	if rot0 then -- CFrameCatRom
		local position = self:SolvePosition(alpha)
		local tangent = self:SolveVelocity(alpha)
		local qw, qx, qy, qz = Squad(rot0, self.rot1, self.rot2, self.rot3, alpha)
		-- FIX: Hand compute the up vector?
		local cf = CFrame.new(0, 0, 0, qx, qy, qz, qw)
		return CFrame.lookAt(position, position + tangent, cf.UpVector)
	else -- VectorCatRom
		return self:SolveCFrame(alpha)
	end
end

function Spline:SolveLength(a: number?, b: number?)
	if self.length and (not a and not b or a == 0 and b == 1) then
		return self.length
	end

	a = a or 0
	b = b or 1
	
	-- Sum the distances between points.
	-- FIX: Replace this with a better numerical integration on ||v(t)||
	-- local lastPos = self:SolvePosition(a)
	-- local steps = (b - a) * 100
	-- local length = 0
	-- for i = 1, steps do
	-- 	i /= steps
	-- 	local thisPos = self:SolvePosition(a + (b - a) * i)
	-- 	length += (thisPos - lastPos).Magnitude
	-- 	lastPos = thisPos
	-- end

	-- return length

	return Integrate.Simp38Comp(function(x)
		return self:SolveVelocity(x).Magnitude
	end, a, b, 5)
end

-- TODO: Reparameterize (this will allow for Uniform methods)

---- START GENERATED METHODS
---- END GENERATED METHODS

return Spline