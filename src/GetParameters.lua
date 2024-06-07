local Types = require(script.Parent.Types)

local GetParameters = {}

-- alpha = 0
function GetParameters.Uniform(p0: Types.Vector, p1: Types.Vector, p2: Types.Vector, p3: Types.Vector, tension: number): (number, number, number)
	local p10 = p1 - p0
	local p20 = p2 - p0
	local p21 = p2 - p1
	local p31 = p3 - p1
	local p32 = p3 - p2

	local scalar = (1 - tension)

	local m1 = scalar * (p10 - p20 / 2 + p21)
	local m2 = scalar * (p21 - p31 / 2 + p32)

	local a = -2 * p21 + m1 + m2
	local b = 3 * p21 - 2 * m1 - m2
	local c = m1

	return a, b, c
end

-- alpha = 0.5
function GetParameters.Centripetal(p0: Types.Vector, p1: Types.Vector, p2: Types.Vector, p3: Types.Vector, tension: number): (number, number, number)
	local p10 = p1 - p0
	local p20 = p2 - p0
	local p21 = p2 - p1
	local p31 = p3 - p1
	local p32 = p3 - p2

	local p10Mag = math.sqrt(p10.Magnitude)
	local p21Mag = math.sqrt(p21.Magnitude)
	local p32Mag = math.sqrt(p32.Magnitude)

	local scalar = (1 - tension) * p21Mag

	local m1 = scalar * (p10 / p10Mag - p20 / p10Mag            + p21 / p21Mag)
	local m2 = scalar * (p21 / p21Mag - p31 / (p21Mag + p32Mag) + p32 / p32Mag)

	local a = -2 * p21 + m1 + m2
	local b = 3 * p21 - 2 * m1 - m2
	local c = m1

	return a, b, c
end

-- alpha = 1
function GetParameters.Chordal(p0: Types.Vector, p1: Types.Vector, p2: Types.Vector, p3: Types.Vector, tension: number): (number, number, number)
	local p10 = p1 - p0
	local p20 = p2 - p0
	local p21 = p2 - p1
	local p31 = p3 - p1
	local p32 = p3 - p2

	local p10Mag = p10.Magnitude
	local p21Mag = p21.Magnitude
	local p32Mag = p32.Magnitude

	local scalar = (1 - tension) * p21Mag

	local m1 = scalar * (p10.Unit - p20 / p10Mag            + p21.Unit)
	local m2 = scalar * (p21.Unit - p31 / (p21Mag + p32Mag) + p32.Unit)

	local a = -2 * p21 + m1 + m2
	local b = 3 * p21 - 2 * m1 - m2
	local c = m1

	return a, b, c
end

-- https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
function GetParameters.General(p0: Types.Vector, p1: Types.Vector, p2: Types.Vector, p3: Types.Vector, tension: number, alpha: number): (number, number, number)
	local p10 = p1 - p0
	local p20 = p2 - p0
	local p21 = p2 - p1
	local p31 = p3 - p1
	local p32 = p3 - p2

	local p10Mag = p10.Magnitude ^ alpha
	local p21Mag = p21.Magnitude ^ alpha
	local p32Mag = p32.Magnitude ^ alpha

	local scalar = (1 - tension) * p21Mag

	local m1 = scalar * (p10 / p10Mag - p20 / p10Mag            + p21 / p21Mag)
	local m2 = scalar * (p21 / p21Mag - p31 / (p21Mag + p32Mag) + p32 / p32Mag)

	local a = -2 * p21 + m1 + m2
	local b = 3 * p21 - 2 * m1 - m2
	local c = m1

	return a, b, c
end

-- tension = 1
function GetParameters.Taut(_, p1: Types.Vector, p2: Types.Vector, _, _): (number, number, number)
	local p21 = p2 - p1

	local a = -2 * p21
	local b = 3 * p21
	local c = 0

	return a, b, c
end

return GetParameters