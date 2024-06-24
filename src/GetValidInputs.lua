local Constants = require(script.Parent.Constants)
local Types = require(script.Parent.Types)
local Utils = require(script.Parent.Utils)

return function(points: {Types.Point}, props: Types.CatRomProps?): (number, number, boolean, {number}?)
	assert(typeof(props) == "table", "props must be a table")
	props = props or {}
	assert(props ~= nil, "Appease type system")

	local alpha = props.alpha or Constants.DEFAULT_ALPHA
	local tension = props.tension or Constants.DEFAULT_TENSION
	local loops = props.loops or false
	local keyframes = props.keyframes

	assert(type(points) == "table", "points must be a table")
	assert(type(alpha) == "number", "alpha must be a number")
	assert(type(tension) == "number", "tension must be a number")
	assert(#points > 0, "points cannot be empty")
	
	local pointType = typeof(points[1])
	assert(pointType == "number" or pointType == "Vector2" or pointType == "Vector3" or pointType == "CFrame",
		"points must be a table of numbers, Vector2s, Vector3s, or CFrames")
	for _, point in points do
		assert(typeof(point) == pointType, "All points must have the same type")
	end

	if keyframes then
		assert(type(keyframes) == "table", "keyframes must be a table")
		assert(#points == #keyframes, "#keyframes must equal #points")
		assert(alpha == 0.5, "alpha must be 0.5 for a spline with keyframes")
		
		local prevKeyframe = keyframes[1]
		assert(prevKeyframe == 0, "First keyframe must be 0")

		-- Assert that the keyframes are ascending and greater than MACHINE_EPS
		-- apart, thus guaranteeing no adjacent points that are fuzzy-equal
		for i = 2, #keyframes do
			local keyframe = keyframes[i]
			assert(keyframe > prevKeyframe + Constants.MACHINE_EPSILON, "keyframes must be ascending")
			prevKeyframe = keyframe
		end

		if #keyframes > 1 then
			assert(prevKeyframe == 1, "Last keyframe must be 1")
		end

		-- In a looping keyframe spline, it does not make temporal sense to add
		-- an extra segment connecting the last keyframe back to the first.
		-- Therefore, we assert that the first and last points already match.
		-- Then, in SegmentFactory, we do some extra work to ensure a smooth
		-- loop in the time dimension as well.
		if loops then
			assert(
				Utils.FuzzyEq(points[1], points[#points], Constants.MACHINE_EPSILON),
				"First and last points must be equal in a looping keyframe spline"
			)
		end
	end

	return alpha, tension, loops, keyframes
end