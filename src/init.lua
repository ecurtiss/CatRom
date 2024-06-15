local TweenService = game:GetService("TweenService")
local SegmentFactory = require(script.SegmentFactory)
local Types = require(script.Types)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_CHEB_DEGREE = 8
local DEFAULT_RMF_PRECOMPUTES = 4
local EPSILON = 2e-7

--[=[
	@class CatRom
	A Catmull-Rom spline.

	:::note
	The `Vector2`, `Vector3`, and `CFrame` tags indicate whether a method is
	defined for the type of control points used to create the [CatRom].
	:::
	:::tip
	You should call [CatRom:PrecomputeUnitSpeedData] immediately after
	construction if you intend to do many computations with `unitSpeed` true
	(where "many" is roughly 8 per segment). Doing so will drastically improve
	the performance of reparametrizations with a small loss to accuracy.
	:::
]=]
local CatRom: Types.CatRomMt = {} :: Types.CatRomMt
CatRom.__index = CatRom

--- @type Vector Vector2 | Vector3
--- @within CatRom

--- @prop knots {number}
--- @within CatRom
--- @private
--- The knot vector of the spline.

--- @prop length number
--- @within CatRom
--- The arc length of the spline.

--- @prop points {Point}
--- @within CatRom
--- The control points of the spline. This table will not necessarily match the
--- table of control points passed into the constructor, as adjacent points
--- that are fuzzy-equal are removed during construction.

--- @prop segments {Segment}
--- @within CatRom
--- The chain of interpolating segments that make up the full spline.

-- Removes adjacent points that are fuzzy-equal
local function getUniquePoints(points: {Types.Point}): {Types.Point}
	local prevPoint = points[1]
	local uniquePoints = {prevPoint}
	local i = 2

	for j = 2, #points do
		local point = points[j]
		if not point:FuzzyEq(prevPoint, EPSILON) then
			uniquePoints[i] = point
			i += 1
			prevPoint = point
		end
	end

	return uniquePoints
end

--[=[
	Instantiates a [CatRom].

	@param points {Vector2} | {Vector3} | {CFrame} -- A list of points of the same type
	@param alpha -- A number (usually in [0, 1]) that loosely affects the curvature of the spline at the control points (default: 0.5)
	@param tension -- A number (usually in [0, 1]) that changes how taut the spline is (1 gives straight lines) (default: 0)
	@param loops -- Whether the spline should form a smooth, closed loop
	@return CatRom
	@tag Vector2
	@tag Vector3
	@tag CFrame
]=]
function CatRom.new(points: {Types.Point}, alpha: number?, tension: number?, loops: boolean?): Types.CatRom
	alpha = alpha or DEFAULT_ALPHA -- Parametrization exponent
	tension = tension or DEFAULT_TENSION
	loops = not not loops

	-- Check types
	assert(type(points) == "table", "Points must be a table")
	assert(type(alpha) == "number", "Alpha must be a number")
	assert(type(tension) == "number", "Tension must be a number")
	assert(#points > 0, "Points table cannot be empty")
	
	local pointType = typeof(points[1])
	assert(pointType == "Vector2" or pointType == "Vector3" or pointType == "CFrame",
		"Points must be a table of Vector2s, Vector3s, or CFrames")
	for _, point in points do
		assert(typeof(point) == pointType, "All points must have the same type")
	end

	-- Get points
	points = getUniquePoints(points)

	if loops and not points[1]:FuzzyEq(points[#points], EPSILON) then
		table.insert(points, points[1])
	end

	-- Early exits
	if #points <= 2 then
		local segment = if #points == 1
			then SegmentFactory.CreatePointSegment(points[1], pointType)
			else SegmentFactory.CreateLineSegment(points[1], points[2], pointType)

		return setmetatable({
			knots = {0, 1},
			length = segment.length,
			points = points,
			segments = {segment},
		}, CatRom)
	end

	-- Create segments
	local segments = SegmentFactory.CreateSegments(points, alpha, tension, loops, pointType)
	local numSegments = #segments

	-- Tally length
	local totalLength = 0
	for _, segment in segments do
		totalLength += segment.length
	end

	-- Create knot vector
	local knots = table.create(numSegments + 1)
	knots[numSegments + 1] = 1

	local runningLength = 0
	for i, segment in segments do
		knots[i] = runningLength / totalLength
		runningLength += segment.length
	end

	return setmetatable({
		knots = knots,
		length = totalLength,
		points = points,
		segments = segments,
	}, CatRom)
end

--------------------------------------------------------------------------------
-- Proxy methods ---------------------------------------------------------------
--------------------------------------------------------------------------------

--[=[
	Returns the position at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolvePosition(t: number, unitSpeed: boolean?): Types.Vector
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolvePosition(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the velocity at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveVelocity(t: number, unitSpeed: boolean?): Types.Vector
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveVelocity(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the acceleration at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveAcceleration(t: number, unitSpeed: boolean?): Types.Vector
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveAcceleration(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the jerk at time `t`.

	@param t -- Time
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveJerk(t: number): Types.Vector
	return self:GetSegmentAtTime(t):SolveJerk()
end

--[=[
	Returns the unit tangent vector at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveTangent(t: number, unitSpeed: boolean?): Types.Vector
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveTangent(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the unit normal vector of the Frenet frame at time `t`. Returns
	`(nan, nan, nan)` if the Frenet frame does not exist (i.e., when curvature
	is 0).

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveNormal(t: number, unitSpeed: boolean?): Types.Vector
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveNormal(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the unit binormal vector of the Frenet frame at time `t`. Returns
	`(nan, nan, nan)` if the Frenet frame does not exist (i.e., when curvature
	is 0).

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return Vector3
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveBinormal(t: number, unitSpeed: boolean?): Vector3
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveBinormal(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the curvature at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveCurvature(t: number, unitSpeed: boolean?): number
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveCurvature(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns the torsion at time `t`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveTorsion(t: number, unitSpeed: boolean?): number
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveTorsion(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns a CFrame at time `t` looking along the tangent vector of the spline
	with the goal up vector `upVector`.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@param upVector -- The goal up vector (default: `Vector3.yAxis`)
	@tag Vector3
	@tag CFrame
	@tag Moving frames
]=]
function CatRom:SolveCFrameLookAlong(t: number, unitSpeed: boolean?, upVector: Vector3?): CFrame
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveCFrameLookAlong(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime, upVector)
end

--[=[
	Returns a CFrame at time `t` with LookVector, UpVector, and RightVector
	being the unit tangent, normal, and binormal vectors of the Frenet frame,
	respectively. Returns a CFrame with NaNs in its orientation if the Frenet
	frame does not exist (i.e., when curvature is 0). The Frenet frame is also
	prone to sudden twists when curvature is near 0.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector3
	@tag CFrame
	@tag Moving frames
]=]
function CatRom:SolveCFrameFrenet(t: number, unitSpeed: boolean?): CFrame
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveCFrameFrenet(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--[=[
	Returns a CFrame at time `t` that smoothly interpolates the orientations of
	the control points using spherical quadrangle interpolation (SQUAD).

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag CFrame
	@tag Moving frames
]=]
function CatRom:SolveCFrameSquad(t: number, unitSpeed: boolean?): CFrame
	local segment, segmentTime = self:GetSegmentAtTime(t)
	return segment:SolveCFrameSquad(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
end

--------------------------------------------------------------------------------
-- Rotation-minimizing frames --------------------------------------------------
--------------------------------------------------------------------------------

--[=[
	Returns an approximate rotation-minimizing frame (RMF), i.e., a CFrame that
	has minimal torsion when swept along the spline. This method is not useful
	on its own, but it underlies the more powerful [CatRom:Transport] and
	[CatRom:SlerpNormals] (and their bulk alternatives
	[CatRom:GetTransportInterpolant] and [CatRom:GetSlerpNormalsInterpolant]).

	If you are tweening an RMF over time, then you can supply the RMF from the
	previous point in time for a better approximation. Doing so also avoids
	calling [CatRom:PrecomputeRMFs], which helps performance. Otherwise, if you
	do not supply a previous frame and you have not yet called
	[CatRom:PrecomputeRMFs], then [CatRom:PrecomputeRMFs] will be called for
	only the necessary segments.

	@param t -- Time
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@param prevFrame -- A previous, nearby RMF
	@param numFramesPerSegment -- The number of discrete approximations to pass to [CatRom:PrecomputeRMFs] if necessary (default: 4)
	@tag Vector3
	@tag CFrame
	@tag Moving frames
]=]
function CatRom:SolveCFrameRMF(t: number, unitSpeed: boolean?, prevFrame: CFrame?, numFramesPerSegment: number?): CFrame
	local segment, segmentTime, segmentIndex = self:GetSegmentAtTime(t)
	local segments = self.segments

	if prevFrame then
		return segment:SolveCFrameRMF(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime, prevFrame)
	end

	-- Precompute only the necessary RMF LUTs
	if segment.rmfLUT == nil then
		if segments[1].rmfLUT == nil then
			self:PrecomputeRMFs(numFramesPerSegment, 1, segmentIndex)
		else
			local i = segmentIndex - 1
			while segments[i].rmfLUT == nil do
				i -= 1
			end
			self:PrecomputeRMFs(numFramesPerSegment, i + 1, segmentIndex)
		end
	end

	return segment:SolveCFrameRMF(if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime, prevFrame)
end

--[=[
	Sweeps a [Vector3] or [CFrame] along the spline such that the result has
	been twisted minimally around the spline.

	You should call [CatRom:PrecomputeRMFs] beforehand if you want to control
	the accuracy of the approximation—otherwise it will be called for you with
	default values.

	:::tip
	See [CatRom:GetTransportInterpolant] if you intend to use this method in
	bulk.
	:::

	@param data -- The data to sweep
	@param from -- The time where the data comes from
	@param to -- The time to sweep to
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:Transport(
	data: Vector3 | CFrame,
	from: number?,
	to: number?,
	unitSpeed: boolean?
): Vector3 | CFrame
	from = from or 0
	to = to or 1
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")

	local dataType = typeof(data)
	local dataIsVector3 = dataType == "Vector3"
	assert(dataIsVector3 or dataType == "CFrame", "Bad data: Can only parallel transport Vector3s and CFrames")

	local fromFrame = self:SolveCFrameRMF(from, unitSpeed)
	local toFrame = self:SolveCFrameRMF(to, unitSpeed)

	if dataIsVector3 then
		return toFrame:VectorToWorldSpace(fromFrame:VectorToObjectSpace(data))
	else
		return toFrame * fromFrame:Inverse() * data
	end
end

--[=[
	Returns a function that sweeps a [Vector3] or [CFrame] along the spline with
	minimal torsion. You should use this method instead of [CatRom:Transport] if
	you are doing transports in bulk, as the interpolant saves on CFrame
	multiplies.
	
	You should call [CatRom:PrecomputeRMFs] beforehand if you want to control
	the accuracy of approximation—otherwise it will be called for you with
	default values.

	@param data -- The data to sweep
	@param from -- The time where the data comes from
	@param to -- The time to sweep to
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return (number) -> Vector3 | CFrame -- A function that returns the transported data at a given time
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:GetTransportInterpolant(
	data: Vector3 | CFrame,
	from: number?,
	to: number?,
	unitSpeed: boolean?
): (t: number) -> Vector3 | CFrame
	from = from or 0
	to = to or 1
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")
	
	local dataType = typeof(data)
	local dataIsVector3 = dataType == "Vector3"
	assert(dataIsVector3 or dataType == "CFrame", "Bad data: Can only parallel transport Vector3s and CFrames")

	local initialFrame = self:SolveCFrameRMF(from, unitSpeed)
	local totalTime = to - from

	if dataIsVector3 then
		local localVector = initialFrame:VectorToObjectSpace(data)
		return function(t: number): Vector3
			return self:SolveCFrameRMF((t - from) / totalTime, unitSpeed):VectorToWorldSpace(localVector)
		end
	else
		local localCFrame = initialFrame:Inverse() * data
		return function(t: number): CFrame
			return self:SolveCFrameRMF((t - from) / totalTime, unitSpeed) * localCFrame
		end
	end
end

local function getSlerpNormalsData(
	self: Types.CatRom,
	from: number,
	fromVector: Vector3,
	to: number,
	toVector: Vector3,
	unitSpeed: boolean?
)
	local frameA = self:SolveCFrameRMF(from, unitSpeed)
	local normalA = (fromVector - frameA.LookVector:Dot(fromVector) * frameA.LookVector).Unit
	local angleA = normalA:Angle(frameA.RightVector, frameA.LookVector)

	if normalA.Magnitude > EPSILON then
		normalA = normalA.Unit
	else
		error("fromVector cannot be tangent to the curve")
	end

	local frameB = self:SolveCFrameRMF(to, unitSpeed)
	local normalB = (toVector - frameB.LookVector:Dot(toVector) * frameB.LookVector).Unit
	local angleB = normalB:Angle(frameB.RightVector, frameB.LookVector)

	if normalB.Magnitude > EPSILON then
		normalB = normalB.Unit
	else
		error("toVector cannot be tangent to the curve")
	end

	local delta = (angleB - angleA) % (2 * math.pi)
	if delta > math.pi then
		delta -= 2 * math.pi
	end

	return angleA, delta
end

--[=[
	Slerps the normal vectors `fromVector` and `toVector` at times `from` and
	`to`, respectively, with progress `t` in [0, 1]. `fromVector` and `toVector`
	are projected to ensure that they are normal to the spline.
	
	You should call [CatRom:PrecomputeRMFs] beforehand if you want to control
	the accuracy of the approximation—otherwise it will be called for you with
	default values.

	:::tip
	Try [CatRom:GetSlerpNormalsInterpolant] if you intend to use this method in
	bulk.
	:::

	@error Bad fromVector -- fromVector cannot be tangent to the spline
	@error Bad toVector -- toVector cannot be tangent to the spline
	@param from -- Time
	@param fromVector -- A vector that is normal to the spline at time `from`
	@param to -- Time
	@param toVector -- A vector that is normal to the spline at time `to`
	@param t -- The progress of the slerp from `fromVector` to `toVector` in [0, 1]
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:SlerpNormals(
	from: number,
	fromVector: Vector3,
	to: number,
	toVector: Vector3,
	t: number,
	unitSpeed: boolean?
): Vector3
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")

	local angleA, delta = getSlerpNormalsData(self, from, fromVector, to, toVector, unitSpeed)
	local totalTime = to - from

	local frameT = self:SolveCFrameRMF(from + t * totalTime, unitSpeed)
	local angleT = angleA + t * delta
	local normalT = math.cos(angleT) * frameT.RightVector + math.sin(angleT) * frameT.UpVector

	return normalT
end

--[=[
	Returns a function that interpolates the normal vectors `fromVector` and
	`toVector` at times `from` and `to`, respectively. `fromVector` and
	`toVector` are projected to ensure that they are normal to the spline.
	You should use this method instead of [CatRom:SlerpNormals] if you are
	doing slerps in bulk, as the interpolant saves on Vector operations.

	You should call [CatRom:PrecomputeRMFs] beforehand if you want to control
	the accuracy of the approximation—otherwise it will be called for you with
	default values.

	@error Bad fromVector -- fromVector cannot be tangent to the spline
	@error Bad toVector -- toVector cannot be tangent to the spline
	@param from -- Time
	@param fromVector -- A vector that is normal to the spline at time `from`
	@param to -- Time
	@param toVector -- A vector that is normal to the spline at time `to`
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@return (number) -> Vector3 -- A function that returns the interpolated normal at a given time
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:GetSlerpNormalsInterpolant(
	from: number,
	fromVector: Vector3,
	to: number,
	toVector: Vector3,
	unitSpeed: boolean?
): (t: number) -> Vector3
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")

	local angleA, delta = getSlerpNormalsData(self, from, fromVector, to, toVector, unitSpeed)
	local totalTime = to - from

	return function(t: number): Vector3
		local frameT = self:SolveCFrameRMF(from + t * totalTime, unitSpeed)
		local angleT = angleA + t * delta
		local normalT = math.cos(angleT) * frameT.RightVector + math.sin(angleT) * frameT.UpVector
		return normalT
	end
end

--[=[
	Precomputes a discrete approximation of a moving rotation-minimizing frame
	(RMF) along the spline.

	@param numFramesPerSegment -- The number of discrete approximations in each segment (default: 4)
	@param firstSegmentIndex -- The index of the first segment to compute RMFs on (default: 1)
	@param lastSegmentIndex -- The index of the last segment to compute RMFs on (default: #segments)
	@tag Vector3
	@tag CFrame
	@tag Precomputes
]=]
function CatRom:PrecomputeRMFs(numFramesPerSegment: number?, firstSegmentIndex: number?, lastSegmentIndex: number?)
	numFramesPerSegment = if numFramesPerSegment then math.max(1, numFramesPerSegment) else DEFAULT_RMF_PRECOMPUTES
	firstSegmentIndex = firstSegmentIndex or 1
	lastSegmentIndex = lastSegmentIndex or #self.segments

	local prevFrame
	if firstSegmentIndex == 1 then
		local pos = self:SolvePosition(0)
		prevFrame = if self.length == 0 then CFrame.new(pos) else CFrame.lookAlong(pos, self:SolveTangent(0))
	else
		prevFrame = self.segments[firstSegmentIndex - 1].rmfLUT[numFramesPerSegment + 1]
	end

	for i = firstSegmentIndex, lastSegmentIndex do
		local segment = self.segments[i]
		segment:PrecomputeRMFs(numFramesPerSegment, prevFrame)
		prevFrame = segment.rmfLUT[numFramesPerSegment + 1]
	end
end

--------------------------------------------------------------------------------
-- Piecewise methods -----------------------------------------------------------
--------------------------------------------------------------------------------

--[=[
	In CatRom, a spline S is a piecewise function of n interpolants defined on
	the interval [0, 1]. We partition [0, 1] into n subintervals of the form
	[k_i, k_{i+1}) where 0 = k1 < k2 < ... < kn < k_{n+1} = 1. The n+1 numbers
	k1, ..., k_{n+1} are called "knots".
	
	The i^th interpolant S_i is defined on the subinterval [k_i, k_{i+1}), where
	the width k_i - k_{i-1} is proportional to the arc length of S_i. That is,
	k_i - k_{i-1} is the arc length of S_i divided by the total arc length of S.
	
	Below is an example with n = 5. We can infer that S2 has a short arc length
	while S4 has a long arc length.

	k1               k2     k3           k4                       k5         k6
	|       S1       |  S2  |     S3     |           S4           |    S5    |
	0 ---------------------------------------------------------------------- 1

	To compute S(t), we find the subinterval [a, b) containing t and evaluate
	its corresponding interpolant S_j at t. However, in implementation, S_j is
	actually defined on [0, 1), so we first map [a, b) -> [0, 1). Hence
	S(t) = S_j((t - a) / (b - a)). Finally, for completeness, S(1) = S_n(1) with
	S_n having domain [0, 1] instead of [0, 1).

	@error Failed to get segment -- Should never happen
	@return Segment -- The interpolant S_j
	@return number -- The time (t - a) / (b - a)
	@return number -- The index j
	@tag Vector2
	@tag Vector3
	@tag CFrame
]=]
function CatRom:GetSegmentAtTime(t: number): (Types.Segment, number, number)
	assert(t >= 0 and t <= 1, "Time must be in [0, 1]")

	local segments = self.segments
	local knots = self.knots
	local numSegments = #segments

	-- Special cases
	if numSegments == 1 then
		return segments[1], t, 1
	elseif t == 0 then
		return segments[1], 0, 1
	elseif t == 1 then
		return segments[numSegments], 1, numSegments
	end

	-- Binary search for the segment containing t
	local left = 1
	local right = numSegments

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = knots[mid]

		if t >= intervalStart then
			local intervalEnd = knots[mid + 1]

			if t <= intervalEnd then
				local segment = segments[mid]
				local segmentTime = (t - intervalStart) / (intervalEnd - intervalStart)
				return segment, segmentTime, mid
			else
				left = mid + 1
			end
		else
			right = mid - 1
		end
	end

	-- This should never happen
	error("Failed to get segment")
end

--[=[
	Makes `unitSpeed` methods significantly faster but slightly less accurate.
	It does this by approximating an expensive root-find on the arc length
	function of a segment with a Chebyshev polynomial that interpolates the
	inverse of the arc length function. Once the cheb has been computed, you can
	either use it as a lookup table ("fast" strategy) or evaluate it as a
	polynomial ("accurate" strategy). The "fast" strategy is roughly an order of
	magnitude faster than the "accurate" strategy but is less accurate.
		
	You should precompute when you are doing more `unitSpeed` calls per
	segment in the spline than the `degree` of the segment's cheb
	(default: 8).

	@param when -- When the cheb should be computed: "now" is right now (useful for doing bulk solves immediately), "on demand" is as-needed (useful for doing bulk method calls over time)
	@param strategy -- Whether the cheb gets used as a lookup table ("fast") or evaluated as a polynomial ("accurate")
	@param degree -- The degree of the chebyshev polynomial; higher is more accurate but slower (default: 8)
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Precomputes
]=]
function CatRom:PrecomputeUnitSpeedData(when: "now" | "on demand"?, strategy: "fast" | "accurate"?,  degree: number?)
	when = when or "now"
	strategy = strategy or "fast"
	degree = degree or DEFAULT_CHEB_DEGREE

	assert(when == "now" or when == "on demand", "when must be \"now\" or \"on demand\"")
	assert(strategy == "fast" or strategy == "accurate", "strategy must be \"fast\" or \"accurate\"")
	assert(type(degree) == "number" and degree >= 1 and degree % 2 == 0, "degree must be an integer >= 1")

	local precomputeNow = when == "now"
	local useChebAsLUT = strategy == "fast"

	for _, segment in self.segments do
		segment:PrecomputeUnitSpeedData(precomputeNow, useChebAsLUT, degree)
	end
end

--[=[
	Computes the arc length of the spline between the times `from` and `to`.

	@param from -- The time to start at (default: 0)
	@param to -- The time to end at (default: 1)
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Basics
]=]
function CatRom:SolveLength(from: number?, to: number?): number
	local a = from or 0
	local b = to or 1
	a, b = math.min(a, b), math.max(a, b)
	
	if a == 0 and b == 1 then
		return self.length
	end

	assert(a >= 0 and b <= 1, "Times must be in [0, 1]")
	
	local segmentA, segmentATime, segmentAIndex = self:GetSegmentAtTime(a)
	local segmentB, segmentBTime, segmentBIndex = self:GetSegmentAtTime(b)

	if segmentAIndex == segmentBIndex then
		return segmentA:SolveLength(segmentATime, segmentBTime)
	end

	local lengthA = segmentA:SolveLength(segmentATime, 1)
	local lengthB = segmentB:SolveLength(0, segmentBTime)

	local intermediateLengths = 0
	for i = segmentAIndex + 1, segmentBIndex - 1 do
		intermediateLengths += self.segments[i].length
	end

	return lengthA + intermediateLengths + lengthB
end

--[=[
	A helper method for doing the same computation at many uniformly-spaced
	times. Has better performance than writing a for loop manually.

	@param f (Segment, number) -> () -- A function accepting a segment and a time
	@param numSamples -- The number of uniformly-spaced samples (includes the two samples at the boundaries)
	@param from -- The time to start at (default: 0)
	@param to -- The time to end at (default: 1)
	@param unitSpeed -- Whether the spline has unit speed (default: `false`)
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:SolveBulk(
	f: (segment: Types.Segment, t: number) -> (),
	numSamples: number,
	from: number?,
	to: number?,
	unitSpeed: boolean?
)
	local a = from or 0
	local b = to or 1
	assert(a <= b, "Time \"from\" cannot be greater than time \"to\"")
	assert(a >= 0 and b <= 1, "Times must be in [0, 1]")
	assert(type(numSamples) == "number", "Bad numSamples")
	
	numSamples = math.round(numSamples)
	if numSamples < 1 then
		return
	end

	local segments = self.segments
	local knots = self.knots

	local segmentA, segmentATime, segmentAIndex = self:GetSegmentAtTime(a)
	local segmentB, segmentBTime, segmentBIndex = self:GetSegmentAtTime(b)

	-- First sample
	f(segmentA, segmentATime)
	
	-- Samples 2, ..., numSamples - 1
	local lerpIncrement = (b - a) / (numSamples - 1)
	local previousDomainMax = knots[segmentAIndex]
	local nextSampleTime = a + lerpIncrement
	local nextSampleIndex = 2

	for i = segmentAIndex, segmentBIndex do
		local segment = segments[i]
		local domainMin = previousDomainMax
		local domainMax = knots[i + 1]
		local domainWidth = domainMax - domainMin

		-- Run all samples in this segment
		while nextSampleTime <= domainMax and nextSampleIndex < numSamples do
			local t = (nextSampleTime - domainMin) / domainWidth
			f(segment, if unitSpeed then segment:Reparametrize(t) else t)
			nextSampleTime = a + nextSampleIndex * lerpIncrement
			nextSampleIndex += 1
		end

		if nextSampleIndex >= numSamples then
			break
		end

		previousDomainMax = domainMax
	end

	-- Last sample
	if numSamples > 1 then
		f(segmentB, segmentBTime)
	end
end

--[=[
	Returns an axis-aligned box that bounds the spline.

	@return Vector -- The minimum corner
	@return Vector -- The maximum corner
	@tag Vector2
	@tag Vector3
	@tag CFrame
]=]
function CatRom:SolveBoundingBox(): (Types.Vector, Types.Vector)
	local segments = self.segments
	local n = #segments - 1

	local firstMin, firstMax = segments[1]:SolveBoundingBox()
	local minima = table.create(n)
	local maxima = table.create(n)

	for i = 1, n do
		minima[i], maxima[i] = segments[i + 1]:SolveBoundingBox()
	end

	local min = firstMin:Min(table.unpack(minima))
	local max = firstMax:Max(table.unpack(maxima))

	return min, max
end

--[=[
	A wrapper around TweenService that executes a callback function. The
	callback accepts a segment and a time to pass into the segment's
	methods. You must manually assign instance properties yourself.
	
	```lua
	-- Tweens a part's CFrame from time 0 to time 1 with unit speed
	local part = Instance.new("Part", workspace)
	local spline = CatRom.new({...})

	local tweenInfo = TweenInfo.new()
	local callback = function(segment, t)
		part.CFrame = segment:SolveCFrameRMF(t)
	end

	local tween = spline:CreateTween(tweenInfo, callback, 0, 1, true)
	tween:Play()
	```
	
	@tag Vector2
	@tag Vector3
	@tag CFrame
	@tag Extensions
]=]
function CatRom:CreateTween(
	tweenInfo: TweenInfo,
	callback: (Types.Segment, number) -> (),
	from: number?,
	to: number?,
	unitSpeed: boolean?
): Tween
	from = from or 0
	to = to or 1

	local numberValue = Instance.new("NumberValue")
	numberValue.Value = from
	
	local tween = TweenService:Create(numberValue, tweenInfo, { Value = to })

	local conn = numberValue:GetPropertyChangedSignal("Value"):Connect(function()
		-- TODO: GetSegmentAtTime can benefit from knowing the previous segment
		-- and time
		local t = numberValue.Value
		local segment, segmentTime = self:GetSegmentAtTime(t)
		callback(segment, if unitSpeed then segment:Reparametrize(segmentTime) else segmentTime)
	end)

	tween.Completed:Once(function()
		conn:Disconnect()
	end)

	return tween
end

return CatRom