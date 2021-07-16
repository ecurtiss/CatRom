-- TODO: Support inputting single numbers
-- TODO: Support inputting arbitrary tuples (n_0, n_1, ..., n_k)
-- TODO: Support variable caching of GetSplineFromAlpha depending on #splines in chain
-- TODO: Cache GetArcLengthAlpha
-- TODO: Add get nearest point on spline
-- TODO: Add get alpha from length
-- TODO: Fix name shadowing with alpha. alpha is a spline parameter but also percent along spline
-- FIX: Shitty transitions between nodes on Arc methods

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local t = require(ReplicatedStorage.t)

local DEFAULT_TENSION = 0
local DEFAULT_KNOT_PARAMETERIZATION = 0.5 -- centripetal
local RIEMANN_STEP = 1e-3
local EPSILON = 1e-4
-- local DT = Vector3.new(1, 1, 1) * RIEMANN_STEP

local SplineMetatable = {}
SplineMetatable.__index = SplineMetatable
local ChainMetatable = {}
ChainMetatable.__index = ChainMetatable
local CatmullRomSpline = {Spline = SplineMetatable, Chain = ChainMetatable}

---- Math
---- spherical quadrangle interpolation (SQUAD) (by fractality)
local function InverseLogProduct(w0, x0, y0, z0, w1, x1, y1, z1)
	local w = w0*w1 + x0*x1 + y0*y1 + z0*z1
	local x = w0*x1 - x0*w1 + y0*z1 - z0*y1
	local y = w0*y1 - x0*z1 - y0*w1 + z0*x1
	local z = w0*z1 + x0*y1 - y0*x1 - z0*w1

	local v = math.sqrt(x^2 + y^2 + z^2)
	local s = v > EPSILON and math.atan2(v, w)/(4*v) or 8/21 + w*(-27/140 + w*(8/105 - w/70))
	return x*s, y*s, z*s
end
local function GetControlRotation(w0, x0, y0, z0, w1, x1, y1, z1, w2, x2, y2, z2)
	if w0*w1 + x0*x1 + y0*y1 + z0*z1 < 0 then
		w0, x0, y0, z0 = -w0, -x0, -y0, -z0
	end
	if w2*w1 + x2*x1 + y2*y1 + z2*z1 < 0 then
		w2, x2, y2, z2 = -w2, -x2, -y2, -z2
	end

	local bx0, by0, bz0 = InverseLogProduct(w0, x0, y0, z0, w1, x1, y1, z1)
	local bx1, by1, bz1 = InverseLogProduct(w2, x2, y2, z2, w1, x1, y1, z1)

	local mx = bx0 + bx1
	local my = by0 + by1
	local mz = bz0 + bz1

	local n = math.sqrt(mx*mx + my*my + mz*mz)
	local m = n > EPSILON and math.sin(n)/n or 1 + n*n*(n*n/120 - 1/6)

	local ew = math.cos(n)
	local ex = m*mx
	local ey = m*my
	local ez = m*mz

	return ew*w1 - ex*x1 - ey*y1 - ez*z1,
		ex*w1 + ew*x1 - ez*y1 + ey*z1,
		ey*w1 + ez*x1 + ew*y1 - ex*z1,
		ez*w1 - ey*x1 + ex*y1 + ew*z1
end
local function Slerp(s, w0, x0, y0, z0, w1, x1, y1, z1, d)
	local t0, t1
	if d < 1 - EPSILON then
		local d0 = y0*x1 + w0*z1 - x0*y1 - z0*w1
		local d1 = y0*w1 - w0*y1 + z0*x1 - x0*z1
		local d2 = y0*z1 - w0*x1 - z0*y1 + x0*w1
		local theta = math.atan2(math.sqrt(d0^2 + d1^2 + d2^2), d)
		local rsa = math.sqrt(1 - d*d)
		t0, t1 = math.sin((1 - s)*theta)/rsa, math.sin(s*theta)/rsa
	else
		t0, t1 = 1 - s, s
	end
	return w0*t0 + w1*t1,
		x0*t0 + x1*t1,
		y0*t0 + y1*t1,
		z0*t0 + z1*t1
end
local function Squad(q0, q1, q2, q3, alpha)
	local q1w, q1x, q1y, q1z = q1[1], q1[2], q1[3], q1[4]
	local q2w, q2x, q2y, q2z = q2[1], q2[2], q2[3], q2[4]

	local p0w, p0x, p0y, p0z = GetControlRotation(
		q0[1], q0[2], q0[3], q0[4],
		q1w, q1x, q1y, q1z,
		q2w, q2x, q2y, q2z
	)
	local p1w, p1x, p1y, p1z = GetControlRotation(
		q1w, q1x, q1y, q1z,
		q2w, q2x, q2y, q2z,
		q3[1], q3[2], q3[3], q3[4]
	)

	local dq = q1w*q2w + q1x*q2x + q1y*q2y + q1z*q2z
	local dp = math.abs(p0w*p1w + p0x*p1x + p0y*p1y + p0z*p1z)
	if dq < 0 then
		p1w, p1x, p1y, p1z = -p1w, -p1x, -p1y, -p1z
		q2w, q2x, q2y, q2z = -q2w, -q2x, -q2y, -q2z
		dq = -dq
	end

	local w0, x0, y0, z0 = Slerp(alpha, q1w, q1x, q1y, q1z, q2w, q2x, q2y, q2z, dq)
	local w1, x1, y1, z1 = Slerp(alpha, p0w, p0x, p0y, p0z, p1w, p1x, p1y, p1z, dp)
	return Slerp(2*alpha*(1 - alpha), w0, x0, y0, z0, w1, x1, y1, z1, w0*w1 + x0*x1 + y0*y1 + z0*z1)
end
local function CFrameToQuaternion(cframe)
	local _, _, _, m00, m01, m02, m10, m11, m12, m20, m21, m22 = cframe:GetComponents()
	local trace = m00 + m11 + m22
	if trace > 0 then
		local s = math.sqrt(1 + trace)
		local reciprocal = 0.5 / s
		local quaternion = table.create(4)
		quaternion[1] = s / 2
		quaternion[2] = (m21 - m12) * reciprocal
		quaternion[3] = (m02 - m20) * reciprocal
		quaternion[4] = (m10 - m01) * reciprocal
		return quaternion
	else
		local big = math.max(m00, m11, m22)
		if big == m00 then
			local s = math.sqrt(1 + m00 - m11 - m22)
			local reciprocal = 0.5 / s
			local quaternion = table.create(4)
			quaternion[1] = (m21 - m12) * reciprocal
			quaternion[2] = s / 2
			quaternion[3] = (m10 + m01) * reciprocal
			quaternion[4] = (m02 + m20) * reciprocal
			return quaternion
		elseif big == m11 then
			local s = math.sqrt(1 - m00 + m11 - m22)
			local reciprocal = 0.5 / s
			local quaternion = table.create(4)
			quaternion[1] = (m02 - m20) * reciprocal
			quaternion[2] = (m10 + m01) * reciprocal
			quaternion[3] = s / 2
			quaternion[4] = (m21 + m12) * reciprocal
			return quaternion
		elseif big == m22 then
			local s = math.sqrt(1 - m00 - m11 + m22 )
			local reciprocal = 0.5 / s
			local quaternion = table.create(4)
			quaternion[1] = (m10 - m01) * reciprocal
			quaternion[2] = (m02 + m20) * reciprocal
			quaternion[3] = (m21 + m12) * reciprocal
			quaternion[4] = s / 2
			return quaternion
		else
			return table.create(4, nil)
		end
	end
end

local FuzzyEq do
	local function fuzzyEqVector2(v1: Vector2, v2: Vector2)
		local v1x, v2x = v1.X, v2.X
		if v1x == v2x or math.abs(v1x - v2x) <= (math.abs(v1x) + 1) * EPSILON then
			local v1y, v2y = v1.Y, v2.Y
			if v1y == v2y or math.abs(v1y - v2y) <= (math.abs(v1y) + 1) * EPSILON then
				return true
			end
		end
		return false
	end

	local function fuzzyEqCFrame(cf1: CFrame, cf2: CFrame)
		if cf1.Position:FuzzyEq(cf2.Position, EPSILON)
			and cf1.RightVector:FuzzyEq(cf2.RightVector, EPSILON)
			and cf1.UpVector:FuzzyEq(cf2.UpVector, EPSILON)
			and cf1.LookVector:FuzzyEq(cf2.LookVector, EPSILON) then
			return true
		end
		return false
	end

	function FuzzyEq(a, b)
		local aType = typeof(a)
		if aType == "number" then
			return a == b or math.abs(a - b) <= (math.abs(a) + 1) * EPSILON
		elseif aType == "Vector2" then
			return fuzzyEqVector2(a, b)
		elseif aType == "Vector3" then
			return a:FuzzyEq(b, EPSILON)
		elseif aType == "CFrame" then
			return fuzzyEqCFrame(a, b)
		end
	end
end

---- Spline
-- types
type Knot = Vector2 | Vector3 | CFrame
local tUnitInterval = t.numberConstrained(0, 1)
local tOptionalUnitInterval = t.optional(tUnitInterval)
local function tKnots(k0: Knot, k1: Knot, k2: Knot, k3: Knot)
	local p0Type = t[typeof(k0)]
	assert(p0Type(k1))
	assert(p0Type(k2))
	assert(p0Type(k3))
end

-- constructor
function SplineMetatable.new(k0: Knot, k1: Knot, k2: Knot, k3: Knot, alpha: number?, tension: number?)
	assert(tOptionalUnitInterval(alpha))
	tKnots(k0, k1, k2, k3)
	alpha = alpha or DEFAULT_KNOT_PARAMETERIZATION
	tension = tension or DEFAULT_TENSION
	
	local p0, p1, p2, p3, className
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
		k0 = k0, -- knots
		k1 = k1,
		k2 = k2,
		k3 = k3,
		a = a, -- coefficient vectors for position/velocity/acceleration polynomials
		b = b,
		c = c,
		d = d
	}, SplineMetatable)
	self.Length = self:SolveLength()

	return self
end

-- internal methods
function SplineMetatable:_ToArcLengthAlpha(alpha: number)
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

-- methods
function SplineMetatable:SolvePosition(alpha: number)
	return self.a*alpha^3 + self.b*alpha^2 + self.c*alpha + self.d
end
function SplineMetatable:SolveVelocity(alpha: number)
	return 3*self.a*alpha^2 + 2*self.b*alpha + self.c
end
function SplineMetatable:SolveAcceleration(alpha: number)
	return 6*self.a*alpha + 2*self.b
end
function SplineMetatable:SolveUnitTangent(alpha: number)
	return self:SolveVelocity(alpha).Unit -- T(t) = r'(t) / |r'(t)|
end
function SplineMetatable:SolveUnitNormal(alpha: number)
	local rp = self:SolveVelocity(alpha) -- r'(t)
	local rpp = self:SolveAcceleration(alpha) -- r''(t)
	-- N(t) = T'(t) / |T'(t)| =  (r'(t) / |r'(t)|)' / |(r'(t) / |r'(t)|)'|
	-- the following is equivalent to the rightmost expression
	return (rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude^3).Unit
end
function SplineMetatable:SolveUnitBinormal(alpha: number)
	return self:SolveUnitTangent(alpha):Cross(self:SolveUnitTangent(alpha))
end
function SplineMetatable:SolveCurvature(alpha: number)
	local rp = self:SolveVelocity(alpha)
	local rpp = self:SolveAcceleration(alpha)
	local tangentp = rpp / rp.Magnitude - rp * rp:Dot(rpp) / rp.Magnitude^3

	local curvature = tangentp.Magnitude / rp.Magnitude
	local unitNormal = tangentp.Unit
	
	return curvature, unitNormal
end
function SplineMetatable:SolveCFrame(alpha: number)
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveVelocity(alpha)
	return CFrame.lookAt(position, position + tangent)
end
function SplineMetatable:SolveLength(a: number?, b: number?)
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
function SplineMetatable:SolveRotCFrame(alpha: number)
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
	local qw, qx, qy, qz = Squad(
		CFrameToQuaternion(cf0),
		CFrameToQuaternion(cf1),
		CFrameToQuaternion(cf2),
		CFrameToQuaternion(cf3),
		alpha
	)
	local quaternionToCFrame = CFrame.new(0, 0, 0, qx, qy, qz, qw)
	return CFrame.lookAt(position, position + tangent, quaternionToCFrame.UpVector)
end

-- arc methods
do
	local arcMethods = {}

	for methodName, method in pairs(SplineMetatable) do
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
		SplineMetatable[methodName] = method
	end
end

---- Chain
-- constructor
function ChainMetatable.new(points: {Knot}, alpha: number?, tension: number?)
	assert(#points >= 2, "At least 2 points are required to create a CatmullRomSpline.")
	assert(tOptionalUnitInterval(alpha))
	local knotType = typeof(points[1])
	for _, knot in ipairs(points) do
		if typeof(knot) ~= knotType then
			error(string.format("%s%s",
				"All points must be of the same type. ",
				"Expected \"" .. knotType .. "\", got \"" .. typeof(knot) .. "\"."
			))
		end
	end

	alpha = alpha or DEFAULT_KNOT_PARAMETERIZATION
	tension = tension or DEFAULT_TENSION
	
	local numPoints = #points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	if FuzzyEq(firstPoint, lastPoint) then -- loops
		table.insert(points, points[2]) -- last control point
		table.insert(points, 1, points[numPoints - 1]) -- first control point
	else
		table.insert(points, points[numPoints - 1]:Lerp(lastPoint, 2)) -- last control point
		table.insert(points, 1, points[2]:Lerp(firstPoint, 2)) -- first control point
	end

	local splines = table.create(numPoints + 1)
	local chainLength = 0
	for i = 1, numPoints - 1 do
		local spline = SplineMetatable.new(
			points[i],
			points[i + 1],
			points[i + 2],
			points[i + 3],
			alpha,
			tension
		)
		splines[i] = spline
		chainLength += spline.Length
	end

	local splineIntervals = table.create(numPoints - 1)
	local splineFromAlphaCache = table.create(100)
	local runningChainLength = 0
	for i, spline in ipairs(splines) do
		local intervalStart = runningChainLength / chainLength
		runningChainLength += spline.Length
		local intervalEnd = runningChainLength / chainLength
		splineIntervals[i] = {Start = intervalStart, End = intervalEnd}
		local endAlpha = math.floor(intervalEnd * 100) - math.ceil(intervalStart * 100)
		for alpha = 0, endAlpha do
			local newAlpha = math.ceil(intervalStart * 100) / 100 + alpha / 100
			splineFromAlphaCache[string.format("%.2f", newAlpha)] = i
		end
	end

	return setmetatable({
		ClassName = splines[1].ClassName .. "Chain",
		Length = chainLength,
		Points = points, -- FIX: Should this still include the first and last control points?
		Splines = splines,
		SplineFromAlphaCache = splineFromAlphaCache,
		SplineIntervals = splineIntervals
	}, ChainMetatable)
end

-- internal methods
function ChainMetatable:_AlphaToSpline(alpha: number)
	local startInterval = self.SplineFromAlphaCache[string.format("%.2f", alpha)]
	local splineIntervals = self.SplineIntervals

	for i = startInterval, #splineIntervals do
		local splineInterval = splineIntervals[i]
		if alpha >= splineInterval.Start and alpha <= splineInterval.End then
			local splineAlpha = (alpha - splineInterval.Start) / (splineInterval.End - splineInterval.Start)
			return self.Splines[i], splineAlpha, splineInterval
		end
	end
end

-- methods
function ChainMetatable:SolveLength(a: number?, b: number?)
	if not a and not b then return self.Length end
	assert(tOptionalUnitInterval(a))
	assert(tOptionalUnitInterval(b))
	a = a or 0
	b = b or 1
	local length = 0
	-- TODO: implement this
end
function ChainMetatable:SolveArcLength(a: number?, b: number?)
	if not a and not b then return self.Length end
	assert(tOptionalUnitInterval(a))
	assert(tOptionalUnitInterval(b))
	a = a or 0
	b = b or 1
	local length = 0
	-- TODO: implement this
end
for methodName, _ in pairs(SplineMetatable) do -- wraps every SplineMetatable method
	if string.sub(methodName, 1, 5) == "Solve" and not ChainMetatable[methodName] then
		ChainMetatable[methodName] = function(self, alpha: number)
			assert(tUnitInterval(alpha))
			local spline, splineAlpha = self:_AlphaToSpline(alpha)
			return spline[methodName](spline, splineAlpha)
		end
	end
end

return CatmullRomSpline