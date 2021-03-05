-- TODO: Support inputting numbers
-- TODO: Support inputting position arrays (X, Y, Z)
-- TODO: Support inputting arbitrary arrays (n_0, n_1, ..., n_k)
-- TODO: Support variable caching of GetSplineFromAlpha depending on #splines in chain
-- TODO: Cache GetArcLengthAlpha
-- TODO: Get nearest point on spline
-- TODO: Get alpha from length

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local t = require(ReplicatedStorage.t)

local DEFAULT_TENSION = 0
local DEFAULT_KNOT_PARAMETERIZATION = 0.5 -- centripetal
local RIEMANN_STEP = 1e-2
local EPSILON = 1e-4
-- local DT = Vector3.new(1, 1, 1) * RIEMANN_STEP

local CatmullRomSpline = {Spline = {}, Chain = {}}
CatmullRomSpline.Chain.__index = CatmullRomSpline.Chain
local VectorSplineMetatable = {}
local CFrameSplineMetatable = setmetatable({}, VectorSplineMetatable)
VectorSplineMetatable.__index = VectorSplineMetatable
CFrameSplineMetatable.__index = CFrameSplineMetatable

---- Spherical quadrangle interpolation (SQUAD) (by fractality)
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

---- Math
local function FuzzyEq(a: number, b: number)
	return a == b or math.abs(a - b) <= (math.abs(a) + 1) * EPSILON
end

local function FuzzyEqVector(v1: Vector2 | Vector3, v2: Vector2 | Vector3)
	if not FuzzyEq(v1.X, v2.Y) then return false end
	if not FuzzyEq(v1.Y, v2.Y) then return false end
	if typeof(v1) == "Vector3" then
		if not FuzzyEq(v1.Z, v2.Z) then return false end
	end
	return true
end

local function FuzzyEqCFrame(cf1: CFrame, cf2: CFrame)
	if FuzzyEqVector(cf1.Position, cf2.Position)
	and FuzzyEqVector(cf1.RightVector, cf2.RightVector)
	and FuzzyEqVector(cf1.UpVector, cf2.UpVector)
	and FuzzyEqVector(cf1.LookVector, cf2.LookVector) then
		return true
	end
	return false
end

---- Global
function CatmullRomSpline.SetRiemannStep(riemannStep: number)
	assert(t.numberMin(0)(riemannStep))
	RIEMANN_STEP = riemannStep
end

---- Spline
-- types
type Knot = Vector2 | Vector3 | CFrame
local tUnitInterval = t.numberConstrained(0, 1)
local tOptionalUnitInterval = t.optional(tUnitInterval)
local function tSplineKnots(k0: Knot, k1: Knot, k2: Knot, k3: Knot)
	local p0Type = t[typeof(k0)]
	assert(p0Type(k1))
	assert(p0Type(k2))
	assert(p0Type(k3))
end

-- methods
function CatmullRomSpline.Spline.new(k0: Knot, k1: Knot, k2: Knot, k3: Knot, alpha: number?, tension: number?)
	tSplineKnots(k0, k1, k2, k3) -- checks that they are all the same knot type
	assert(tOptionalUnitInterval(alpha))
	alpha = alpha or DEFAULT_KNOT_PARAMETERIZATION
	tension = tension or DEFAULT_TENSION

	local p0, p1, p2, p3, className, metatable
	if typeof(k0) == "Vector3" or typeof(k0) == "Vector2" then
		p0, p1, p2, p3 = k0, k1, k2, k3
		className = "VectorSpline"
		metatable = VectorSplineMetatable
	elseif typeof(k0) == "CFrame" then
		p0, p1, p2, p3 = k0.Position, k1.Position, k2.Position, k3.Position
		className = "CFrameSpline"
		metatable = CFrameSplineMetatable
	else
		error("Invalid knot type: ", typeof(k0))
	end

	local t0 = 0
	local t1 = (p1 - p0).Magnitude ^ alpha + t0
	local t2 = (p2 - p1).Magnitude ^ alpha + t1
	local t3 = (p3 - p2).Magnitude ^ alpha + t2
	local m1 = (1 - 0) * (t2 - t1) * ((p1 - p0)/(t1 - t0) - (p2 - p0)/(t2 - t0) + (p2 - p1)/(t2 - t1))
	local m2 = (1 - 0) * (t2 - t1) * ((p2 - p1)/(t2 - t1) - (p3 - p1)/(t3 - t1) + (p3 - p2)/(t3 - t2))
	local a = 2 * (p1 - p2) + m1 + m2
	local b = 3 * (p2 - p1) - 2*m1 - m2
	local c = m1
	local d = p1

	local self = setmetatable({
		ClassName = className,
		k0 = k0,
		k1 = k1,
		k2 = k2,
		k3 = k3,
		t1 = t1,
		t2 = t2,
		t3 = t3,
		a = a,
		b = b,
		c = c,
		d = d,
		Length = nil
	}, metatable)
	self.Length = self:SolveLength()

	return self
end

function VectorSplineMetatable:SolvePosition(alpha: number)
	return self.a * alpha^3 + self.b * alpha^2 + self.c * alpha + self.d
end

function VectorSplineMetatable:SolveTangent(alpha: number)
	local p0, p1, p2, p3 = self.k0, self.k1, self.k2, self.k3
	if self.ClassName == "CFrameSpline" then
		p0, p1, p2, p3 = p0.Position, p1.Position, p2.Position, p3.Position
	end
	local t0, t1, t2, t3 = 0, self.t1, self.t2, self.t3
	local s = t1 + alpha * (t2 - t1) -- s instead of t because t is the typechecker :(

	local d1 = t1 - t0 -- d for denominator
	local d2 = t2 - t1
	local d3 = t3 - t2
	local d4 = t2 - t0
	local d5 = t3 - t1

	local a1 = (p0 * (t1 - s) + p1 * (s - t0)) / d1
	local a2 = (p1 * (t2 - s) + p2 * (s - t1)) / d2
	local a3 = (p2 * (t3 - s) + p3 * (s - t2)) / d3
	local b1 = (a1 * (t2 - s) + a2 * (s - t0)) / d4
	local b2 = (a2 * (t3 - s) + a3 * (s - t1)) / d5

	local a1p = (p1 - p0) / d1 -- p for prime (derivative)
	local a2p = (p2 - p1) / d2
	local a3p = (p3 - p2) / d3
	local b1p = (t2*a1p - t0*a2p + a2 - a1 + s*(a2p - a1p)) / d4
	local b2p = (t3*a2p - t1*a3p + a3 - a2 + s*(a3p - a2p)) / d5
	local cp  = (t2*b1p - t1*b2p + b2 - b1 + s*(b2p - b1p)) / d2

	return cp
end

function VectorSplineMetatable:SolveCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveTangent(alpha)
	return CFrame.lookAt(position, position + tangent)
end

function VectorSplineMetatable:SolveLength(a: number?, b: number?)
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

function VectorSplineMetatable:SolveCurvature(alpha: number)
	local p0, p1, p2, p3 = self.k0, self.k1, self.k2, self.k3
	if self.ClassName == "CFrameSpline" then
		p0, p1, p2, p3 = p0.Position, p1.Position, p2.Position, p3.Position
	end
	local t0, t1, t2, t3 = 0, self.t1, self.t2, self.t3
	local s = t1 + alpha * (t2 - t1) -- s instead of t because t is the typechecker :(

	local d1 = t1 - t0 -- d for denominator
	local d2 = t2 - t1
	local d3 = t3 - t2
	local d4 = t2 - t0
	local d5 = t3 - t1

	local a1 = (p0 * (t1 - s) + p1 * (s - t0)) / d1
	local a2 = (p1 * (t2 - s) + p2 * (s - t1)) / d2
	local a3 = (p2 * (t3 - s) + p3 * (s - t2)) / d3
	local b1 = (a1 * (t2 - s) + a2 * (s - t0)) / d4
	local b2 = (a2 * (t3 - s) + a3 * (s - t1)) / d5

	local a1p = (p1 - p0) / d1 -- p for prime (derivative)
	local a2p = (p2 - p1) / d2
	local a3p = (p3 - p2) / d3
	local b1p = (t2*a1p - t0*a2p + a2 - a1 + s*(a2p - a1p)) / d4
	local b2p = (t3*a2p - t1*a3p + a3 - a2 + s*(a3p - a2p)) / d5
	local cp  = (t2*b1p - t1*b2p + b2 - b1 + s*(b2p - b1p)) / d2

	local b1pp = 2 * (a2p - a1p) / d4 -- pp for prime prime (second derivative)
	local b2pp = 2 * (a3p - a2p) / d5
	local cpp  = (t2*b1pp - t1*b2pp + 2*b2p - 2*b1p + s*(b2pp - b1pp)) / d2

	-- local tangent = cp.Unit
	local tangentp = (cpp / cp.Magnitude) - (cp * cp:Dot(cpp)) / cp.Magnitude^3

	return tangentp, tangentp.Magnitude / cp.Magnitude
end

function CFrameSplineMetatable:SolveRotCFrame(alpha: number)
	assert(self.ClassName == "CFrameSpline", "Spline points must be CFrames to call SolveRotCFrame.")
	assert(tUnitInterval(alpha))
	local position = self:SolvePosition(alpha)
	local tangent = self:SolveTangent(alpha)
	local qw, qx, qy, qz = Squad(
		CFrameToQuaternion(self.k0),
		CFrameToQuaternion(self.k1),
		CFrameToQuaternion(self.k2),
		CFrameToQuaternion(self.k3),
		alpha
	)
	local quaternionToCFrame = CFrame.new(0, 0, 0, qx, qy, qz, qw)
	return CFrame.lookAt(position, position + tangent, quaternionToCFrame.UpVector)
end

---- Chain
function CatmullRomSpline.Chain.new(points: {Knot}, tau: number?)
	assert(#points >= 2, "At least 2 points are needed for a spline chain.")
	assert(tOptionalUnitInterval(tau))
	tau = tau or 0

	local numPoints = #points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	local firstControlPoint, lastControlPoint
	if typeof(firstPoint) == "Vector3" or typeof(firstPoint) == "Vector2" then
		if FuzzyEqVector(firstPoint, lastPoint) then -- loops
			firstControlPoint, lastControlPoint = points[numPoints - 1], points[2]
		else
			firstControlPoint = points[2]:Lerp(firstPoint, 2)
			lastControlPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
		end
	elseif typeof(firstPoint) == "CFrame" then
		if FuzzyEqCFrame(firstPoint, lastPoint) then -- loops
			firstControlPoint, lastControlPoint = points[numPoints - 1], points[2]
		else
			firstControlPoint = points[2]:Lerp(firstPoint, 2)
			lastControlPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
		end
	end

	local splines = table.create(numPoints - 1)
	local chainLength
	if numPoints == 2 then
		local firstSpline = CatmullRomSpline.Spline.new(
			firstControlPoint,
			points[1],
			points[2],
			lastControlPoint,
			tau
		)
		chainLength = firstSpline.Length
		splines[1] = firstSpline
	else
		local firstSpline = CatmullRomSpline.Spline.new(
			firstControlPoint,
			points[1],
			points[2],
			points[3],
			tau
		)
		local lastSpline = CatmullRomSpline.Spline.new(
			points[numPoints - 2],
			points[numPoints - 1],
			lastPoint,
			lastControlPoint,
			tau
		)
		chainLength = firstSpline.Length + lastSpline.Length
		splines[1] = firstSpline
		splines[numPoints - 1] = lastSpline
		for i = 1, numPoints - 3 do
			local spline = CatmullRomSpline.Spline.new(
				points[i],
				points[i + 1],
				points[i + 2],
				points[i + 3],
				tau
			)
			splines[i + 1] = spline
			chainLength += spline.Length
		end
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
		Points = points,
		Splines = splines,
		SplineFromAlphaCache = splineFromAlphaCache,
		SplineIntervals = splineIntervals
	}, CatmullRomSpline.Chain)
end

-- internal methods
function CatmullRomSpline.Chain:_GetSplineFromAlpha(alpha)
	local startAlpha = math.floor(alpha * 100) / 100
	local startInterval = self.SplineFromAlphaCache[string.format("%.2f", startAlpha)]
	local numIntervals = #self.SplineIntervals
	for i = startInterval, numIntervals do
		local splineInterval = self.SplineIntervals[i]
		if alpha >= splineInterval.Start and alpha <= splineInterval.End then
			local splineAlpha =
				(alpha - splineInterval.Start) / (splineInterval.End - splineInterval.Start)
			return self.Splines[i], splineAlpha, splineInterval
		end
	end
end
function CatmullRomSpline.Chain:_GetArcLengthAlpha(alpha)
	if alpha == 0 or alpha == 1 then return alpha end

	local length = self.Length
	local spline, _, splineInterval = self:_GetSplineFromAlpha(alpha)
	local goalLength = length * alpha
	local runningLength = length * splineInterval.Start
	local lastPosition = spline:SolvePosition(0)

	for i = 1, 1 / RIEMANN_STEP do
		i *= RIEMANN_STEP
		local thisPosition = spline:SolvePosition(i)
		runningLength += (thisPosition - lastPosition).Magnitude
		--runningLength += ((thisPosition - lastPosition) / (Vector3.new(1, 1, 1) * RIEMANN_STEP)).Magnitude * RIEMANN_STEP
		--runningLength += (dp / DT).Magnitude * RIEMANN_STEP
		lastPosition = thisPosition
		if runningLength >= goalLength then
			return splineInterval.Start + i * (splineInterval.End - splineInterval.Start)
		end
	end
end

-- methods
function CatmullRomSpline.Chain:GetPosition(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_GetSplineFromAlpha(alpha)
	return spline:SolvePosition(splineAlpha)
end
function CatmullRomSpline.Chain:GetCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_GetSplineFromAlpha(alpha)
	return spline:SolveCFrame(splineAlpha)
end
function CatmullRomSpline.Chain:GetRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	assert(self.ClassName == "CFrameSplineChain",
		"Spline chain points must be CFrames to call GetRotCFrame.")
	local spline, splineAlpha = self:_GetSplineFromAlpha(alpha)
	return spline:SolveRotCFrame(splineAlpha)
end
function CatmullRomSpline.Chain:GetCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_GetSplineFromAlpha(alpha)
	return spline:SolveCurvature(splineAlpha)
end
function CatmullRomSpline.Chain:GetArcPosition(alpha: number)
	assert(tUnitInterval(alpha))
	return self:GetPosition(self:_GetArcLengthAlpha(alpha))
end
function CatmullRomSpline.Chain:GetArcCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	return self:GetCFrame(self:_GetArcLengthAlpha(alpha))
end
function CatmullRomSpline.Chain:GetArcRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	assert(self.ClassName == "CFrameSplineChain",
		"Spline chain points must be CFrames to call GetArcRotCFrame.")
	return self:GetRotCFrame(self:_GetArcLengthAlpha(alpha))
end
function CatmullRomSpline.Chain:GetArcCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	return self:GetCurvature(self:_GetArcLengthAlpha(alpha))
end

return CatmullRomSpline